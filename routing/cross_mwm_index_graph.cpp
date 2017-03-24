#include "routing/cross_mwm_index_graph.hpp"

#include "routing/cross_mwm_road_graph.hpp"
#include "routing/osrm_path_segment_factory.hpp"

#include "indexer/scales.hpp"

#include "geometry/mercator.hpp"

#include "base/macros.hpp"
#include "base/stl_helpers.hpp"

#include <algorithm>
#include <utility>

using namespace platform;
using namespace routing;
using namespace std;

namespace
{
struct NodeIds
{
  TOsrmNodeId m_directNodeId;
  TOsrmNodeId m_reverseNodeId;
};

double constexpr kTransitionEqualityDistM = 20.0;

/// \returns FtSeg by |segment|.
OsrmMappingTypes::FtSeg GetFtSeg(Segment const & segment)
{
  return OsrmMappingTypes::FtSeg(segment.GetFeatureId(), segment.GetMinPointId(),
                                 segment.GetMaxPointId());
}

/// \returns a pair of direct and reverse(backward) node id by |segment|.
/// The first member of the pair is direct node id and the second is reverse node id.
NodeIds GetDirectAndReverseNodeId(OsrmFtSegMapping const & segMapping, Segment const & segment)
{
  OsrmFtSegMapping::TFtSegVec const ftSegs = {GetFtSeg(segment)};
  OsrmFtSegMapping::OsrmNodesT osrmNodes;
  segMapping.GetOsrmNodes(ftSegs, osrmNodes);
  CHECK_LESS(osrmNodes.size(), 2, ());
  if (osrmNodes.empty())
    return {INVALID_NODE_ID, INVALID_NODE_ID};

  NodeIds const forwardNodeIds = {osrmNodes.begin()->second.first,
                                  osrmNodes.begin()->second.second};
  NodeIds const backwardNodeIds = {osrmNodes.begin()->second.second,
                                   osrmNodes.begin()->second.first};

  return segment.IsForward() ? forwardNodeIds : backwardNodeIds;
}

bool GetFirstValidSegment(OsrmFtSegMapping const & segMapping, NumMwmId numMwmId,
                          TWrittenNodeId nodeId, Segment & segment)
{
  auto const range = segMapping.GetSegmentsRange(nodeId);
  for (size_t segmentIndex = range.first; segmentIndex != range.second; ++segmentIndex)
  {
    OsrmMappingTypes::FtSeg seg;
    // The meaning of node id in osrm is an edge between two joints.
    // So, it's possible to consider the first valid segment from the range which returns by GetSegmentsRange().
    segMapping.GetSegmentByIndex(segmentIndex, seg);

    if (!seg.IsValid())
      continue;

    CHECK_NOT_EQUAL(seg.m_pointStart, seg.m_pointEnd, ());
    segment = Segment(numMwmId, seg.m_fid, min(seg.m_pointStart, seg.m_pointEnd), seg.IsForward());
    return true;
  }
  LOG(LDEBUG, ("No valid segments in the range returned by OsrmFtSegMapping::GetSegmentsRange(",
               nodeId, "). Num mwm id:", numMwmId));
  return false;
}

void FillTransitionSegments(OsrmFtSegMapping const & segMapping, TWrittenNodeId nodeId,
                            NumMwmId numMwmId, ms::LatLon const & latLon,
                            map<Segment, vector<ms::LatLon>> & transitionSegments)
{
  Segment key;
  if (!GetFirstValidSegment(segMapping, numMwmId, nodeId, key))
    return;

  transitionSegments[key].push_back(latLon);
}

void AddSegmentEdge(NumMwmIds const & numMwmIds, OsrmFtSegMapping const & segMapping,
                    CrossWeightedEdge const & osrmEdge, bool isOutgoing, NumMwmId numMwmId,
                    vector<SegmentEdge> & edges)
{
  BorderCross const & target = osrmEdge.GetTarget();
  CrossNode const & crossNode = isOutgoing ? target.fromNode : target.toNode;

  if (!crossNode.mwmId.IsAlive())
    return;

  NumMwmId const crossNodeMwmId =
      numMwmIds.GetId(crossNode.mwmId.GetInfo()->GetLocalFile().GetCountryFile());
  CHECK_EQUAL(numMwmId, crossNodeMwmId, ());

  Segment segment;
  if (!GetFirstValidSegment(segMapping, crossNodeMwmId, crossNode.node, segment))
    return;

  // OSRM and AStar have different car models, therefore AStar heuristic doesn't work for OSRM node
  // ids (edges).
  // This factor makes index graph (used in AStar) edge weight smaller than node ids weight.
  //
  // As a result large cross mwm routes with connectors works as Dijkstra, but short and medium routes
  // without connectors works as AStar.
  // Most of routes don't use leaps, therefore it is important to keep AStar performance.
  double constexpr kAstarHeuristicFactor = 100000;
  edges.emplace_back(segment,
                     osrmEdge.GetWeight() * kOSRMWeightToSecondsMultiplier * kAstarHeuristicFactor);
}

m2::RectD GetMwmCrossingNodeEqualityRect(m2::PointD const & point)
{
  double constexpr kMwmCrossingNodeEqualityDegrees = kTransitionEqualityDistM * MercatorBounds::degreeInMetres;
  return m2::RectD(point.x - kMwmCrossingNodeEqualityDegrees, point.y - kMwmCrossingNodeEqualityDegrees,
                   point.x + kMwmCrossingNodeEqualityDegrees, point.y + kMwmCrossingNodeEqualityDegrees);
}
}  // namespace

namespace routing
{
CrossMwmIndexGraph::CrossMwmIndexGraph(Index & index, shared_ptr<NumMwmIds> numMwmIds,
                                       shared_ptr<VehicleModelFactory> vehicleModelFactory,
                                       RoutingIndexManager & indexManager)
  : m_index(index), m_numMwmIds(numMwmIds), m_vehicleModelFactory(vehicleModelFactory),
    m_indexManager(indexManager)
{
  ResetCrossMwmGraph();
}

CrossMwmIndexGraph::~CrossMwmIndexGraph() {}

bool CrossMwmIndexGraph::IsTransition(Segment const & s, bool isOutgoing)
{
  // Index graph based cross-mwm information.
  if (DoesCrossMwmSectionExist(s.GetMwmId()))
  {
    CrossMwmConnector const & c = GetCrossMwmConnectorWithTransitions(s.GetMwmId());
    return c.IsTransition(s, isOutgoing);
  }

  // Checking if a segment |s| is a transition segment by osrm cross mwm sections.
  TransitionSegments const & t = GetSegmentMaps(s.GetMwmId());

  if (isOutgoing)
    return t.m_outgoing.count(s) != 0;
  return t.m_ingoing.count(s) != 0;
}

void CrossMwmIndexGraph::GetTwins(Segment const & s, bool isOutgoing, vector<Segment> & twins)
{
  CHECK(IsTransition(s, isOutgoing), ("The segment", s, "is not a transition segment for isOutgoing ==", isOutgoing));
  // Note. There's an extremely rare case when a segment is ingoing and outgoing at the same time.
  // |twins| is not filled for such cases. For details please see a note in CrossMwmIndexGraph::GetEdgeList().
  if (IsTransition(s, !isOutgoing))
    return;

  twins.clear();

  // Note. The code below looks for twins based on geometry index. This code works for
  // any combination mwms with different cross mwm sections.
  // It's possible to implement a faster version for two special cases:
  // * all neighbouring mwms have cross_mwm section
  // * all neighbouring mwms have osrm cross mwm sections
  vector<m2::PointD> const transitions = GetTransitionPoints(s, isOutgoing);
  for (m2::PointD const & p : transitions)
  {
    m_index.ForEachInRect([&](FeatureType & ft){
      if (ft.GetID().m_mwmId.GetInfo()->GetType() != MwmInfo::COUNTRY)
        return;

      NumMwmId const numMwmId = m_numMwmIds->GetId(CountryFile(ft.GetID().GetMwmName()));
      if (numMwmId == s.GetMwmId())
        return;

      ft.ParseGeometry(FeatureType::BEST_GEOMETRY);
      if (!m_vehicleModelFactory->GetVehicleModelForCountry(ft.GetID().GetMwmName())->IsRoad(ft))
        return;

      vector<Segment> twinCandidates;
      GetTransitions(ft, !isOutgoing, twinCandidates);
      for (Segment const & tc : twinCandidates)
      {
        vector<m2::PointD> const twinPoints = GetTransitionPoints(tc, !isOutgoing);
        for (m2::PointD const & tp : twinPoints)
        {
          if (MercatorBounds::DistanceOnEarth(p, tp) <= kTransitionEqualityDistM)
            twins.push_back(tc);
        }
      }
    }, GetMwmCrossingNodeEqualityRect(p), scales::GetUpperScale());
  }

  my::SortUnique(twins);

  for (Segment const & t : twins)
    CHECK_NOT_EQUAL(s.GetMwmId(), t.GetMwmId(), ());
}

void CrossMwmIndexGraph::GetEdgeList(Segment const & s, bool isOutgoing,
                                     vector<SegmentEdge> & edges)
{
  CHECK(IsTransition(s, !isOutgoing), ("The segment is not a transition segment. IsTransition(",
                                       s, ",", !isOutgoing, ") returns false."));
  edges.clear();

  // Osrm based cross-mwm information.

  // Note. According to cross-mwm OSRM sections there're node id which could be ingoing and outgoing
  // at the same time. For example in Berlin mwm on Nordlicher Berliner Ring (A10) near crossing
  // with A11
  // there's such node id. It's an extremely rare case. There're probably several such node id
  // for the whole Europe. Such cases are not processed in WorldGraph::GetEdgeList() for the time
  // being.
  // To prevent filling |edges| with twins instead of leap edges and vice versa in
  // WorldGraph::GetEdgeList()
  // CrossMwmIndexGraph::GetEdgeList() does not fill |edges| if |s| is a transition segment which
  // corresponces node id described above.
  if (IsTransition(s, isOutgoing))
    return;

  // Index graph based cross-mwm information.
  if (DoesCrossMwmSectionExist(s.GetMwmId()))
  {
    CrossMwmConnector const & c = CrossMwmIndexGraph::GetCrossMwmConnectorWithWeights(s.GetMwmId());
    c.GetEdgeList(s, isOutgoing, edges);
    return;
  }

  auto const fillEdgeList = [&](TRoutingMappingPtr const & mapping) {
    vector<BorderCross> borderCrosses;
    GetBorderCross(mapping, s, isOutgoing, borderCrosses);

    for (BorderCross const & v : borderCrosses)
    {
      vector<CrossWeightedEdge> adj;
      if (isOutgoing)
        m_crossMwmGraph->GetOutgoingEdgesList(v, adj);
      else
        m_crossMwmGraph->GetIngoingEdgesList(v, adj);

      for (CrossWeightedEdge const & edge : adj)
        AddSegmentEdge(*m_numMwmIds, mapping->m_segMapping, edge, isOutgoing, s.GetMwmId(), edges);
    }
  };
  LoadWith(s.GetMwmId(), fillEdgeList);

  my::SortUnique(edges);
}

void CrossMwmIndexGraph::Clear()
{
  ResetCrossMwmGraph();
  m_transitionCache.clear();
  m_mappingGuards.clear();
  m_crossMwmIndexGraph.clear();
}

void CrossMwmIndexGraph::ResetCrossMwmGraph()
{
  m_crossMwmGraph = make_unique<CrossMwmGraph>(m_indexManager);
}

void CrossMwmIndexGraph::InsertWholeMwmTransitionSegments(NumMwmId numMwmId)
{
  if (m_transitionCache.count(numMwmId) != 0)
    return;

  auto const fillAllTransitionSegments = [&](TRoutingMappingPtr const & mapping) {
    TransitionSegments transitionSegments;
    mapping->m_crossContext.ForEachOutgoingNode([&](OutgoingCrossNode const & node)
    {
      FillTransitionSegments(mapping->m_segMapping, node.m_nodeId, numMwmId,
                             node.m_point, transitionSegments.m_outgoing);
    });
    mapping->m_crossContext.ForEachIngoingNode([&](IngoingCrossNode const & node)
    {
      FillTransitionSegments(mapping->m_segMapping, node.m_nodeId, numMwmId,
                             node.m_point, transitionSegments.m_ingoing);
    });
    auto const p = m_transitionCache.emplace(numMwmId, move(transitionSegments));
    UNUSED_VALUE(p);
    ASSERT(p.second, ("Mwm num id:", numMwmId, "has been inserted before. Country file name:",
                      mapping->GetCountryName()));
  };

  if (!LoadWith(numMwmId, fillAllTransitionSegments))
    m_transitionCache.emplace(numMwmId, TransitionSegments());
}

void CrossMwmIndexGraph::GetBorderCross(TRoutingMappingPtr const & mapping, Segment const & s,
                                        bool isOutgoing, vector<BorderCross> & borderCrosses)
{
  // ingoing edge
  NodeIds const nodeIdsTo = GetDirectAndReverseNodeId(mapping->m_segMapping, s);

  vector<ms::LatLon> const & transitionPoints =
      isOutgoing ? GetIngoingTransitionPoints(s) : GetOutgoingTransitionPoints(s);
  CHECK(!transitionPoints.empty(), ("Segment:", s, ", isOutgoing:", isOutgoing));

  // If |isOutgoing| == true |nodes| is "to" cross nodes, otherwise |nodes| is "from" cross nodes.
  vector<CrossNode> nodes;
  for (ms::LatLon const & p : transitionPoints)
    nodes.emplace_back(nodeIdsTo.m_directNodeId, nodeIdsTo.m_reverseNodeId, mapping->GetMwmId(), p);

  // outgoing edge
  vector<Segment> twins;
  GetTwins(s, !isOutgoing, twins);
  if (twins.empty())
    return;

  for (Segment const & twin : twins)
  {
    // If |isOutgoing| == true |otherSideMapping| is mapping outgoing (to) border cross.
    // If |isOutgoing| == false |mapping| is mapping ingoing (from) border cross.
    auto const fillBorderCrossOut = [&](TRoutingMappingPtr const & otherSideMapping) {
      NodeIds const nodeIdsFrom = GetDirectAndReverseNodeId(otherSideMapping->m_segMapping, twin);
      vector<ms::LatLon> const & otherSideTransitionPoints =
          isOutgoing ? GetOutgoingTransitionPoints(twin) : GetIngoingTransitionPoints(twin);

      CHECK(!otherSideTransitionPoints.empty(), ("Segment:", s, ", isOutgoing:", isOutgoing));
      for (CrossNode const & node : nodes)
      {
        // If |isOutgoing| == true |otherSideNodes| is "from" cross nodes, otherwise
        // |otherSideNodes| is "to" cross nodes.
        BorderCross bc;
        if (isOutgoing)
          bc.toNode = node;
        else
          bc.fromNode = node;

        for (ms::LatLon const & ll : otherSideTransitionPoints)
        {
          CrossNode & resultNode = isOutgoing ? bc.fromNode : bc.toNode;
          resultNode = CrossNode(nodeIdsFrom.m_directNodeId, nodeIdsFrom.m_reverseNodeId,
                                 otherSideMapping->GetMwmId(), ll);
          borderCrosses.push_back(bc);
        }
      }
    };
    LoadWith(twin.GetMwmId(), fillBorderCrossOut);
  }
}

CrossMwmIndexGraph::TransitionSegments const & CrossMwmIndexGraph::GetSegmentMaps(NumMwmId numMwmId)
{
  auto it = m_transitionCache.find(numMwmId);
  if (it == m_transitionCache.cend())
  {
    InsertWholeMwmTransitionSegments(numMwmId);
    it = m_transitionCache.find(numMwmId);
  }
  CHECK(it != m_transitionCache.cend(), ("Mwm ", numMwmId, "has not been downloaded."));
  return it->second;
}

CrossMwmConnector const & CrossMwmIndexGraph::GetCrossMwmConnectorWithTransitions(NumMwmId numMwmId)
{
  ASSERT(DoesCrossMwmSectionExist(numMwmId), ());

  auto const it = m_crossMwmIndexGraph.find(numMwmId);
  if (it != m_crossMwmIndexGraph.cend())
    return it->second;

  return Deserialize(numMwmId,
                     CrossMwmConnectorSerializer::DeserializeTransitions<ReaderSource<FilesContainerR::TReader>>);
}

CrossMwmConnector const & CrossMwmIndexGraph::GetCrossMwmConnectorWithWeights(NumMwmId numMwmId)
{
  GetCrossMwmConnectorWithTransitions(numMwmId);

  auto const it = m_crossMwmIndexGraph.find(numMwmId);
  CHECK(it != m_crossMwmIndexGraph.cend(), ());
  CrossMwmConnector const & c = it->second;
  if (c.WeightsWereLoaded())
    return c;

  return Deserialize(numMwmId,
                     CrossMwmConnectorSerializer::DeserializeWeights<ReaderSource<FilesContainerR::TReader>>);
}

vector<ms::LatLon> const & CrossMwmIndexGraph::GetIngoingTransitionPoints(Segment const & s)
{
  auto const & ingoingSeg = GetSegmentMaps(s.GetMwmId()).m_ingoing;
  auto const it = ingoingSeg.find(s);
  CHECK(it != ingoingSeg.cend(), ("Segment:", s));
  return it->second;
}

vector<ms::LatLon> const & CrossMwmIndexGraph::GetOutgoingTransitionPoints(Segment const & s)
{
  auto const & outgoingSeg = GetSegmentMaps(s.GetMwmId()).m_outgoing;
  auto const it = outgoingSeg.find(s);
  CHECK(it != outgoingSeg.cend(), ("Segment:", s));
  return it->second;
}

vector<m2::PointD> CrossMwmIndexGraph::GetTransitionPoints(Segment const & s, bool isOutgoing)
{
  if (DoesCrossMwmSectionExist(s.GetMwmId()))
  {
    CrossMwmConnector const & connectors = GetCrossMwmConnectorWithTransitions(s.GetMwmId());
    // In case of transition segments of index graph cross-mwm section the front point of segemnt
    // is used as a point which corresponds to the segment.
    return vector<m2::PointD>({connectors.GetPoint(s, true /* front */)});
  }

  vector<ms::LatLon> const & latLons = isOutgoing ? GetOutgoingTransitionPoints(s)
                                                  : GetIngoingTransitionPoints(s);
  vector<m2::PointD> points;
  points.reserve(latLons.size());
  for (auto const & ll : latLons)
    points.push_back(MercatorBounds::FromLatLon(ll));
  return points;
}

MwmValue & CrossMwmIndexGraph::GetValue(NumMwmId numMwmId)
{
  MwmSet::MwmHandle const & handle = m_index.GetMwmHandleByCountryFile(m_numMwmIds->GetFile(numMwmId));
  CHECK(handle.IsAlive(), ());
  MwmValue * value = handle.GetValue<MwmValue>();
  CHECK(value != nullptr, ("Country file:", m_numMwmIds->GetFile(numMwmId)));
  return *value;
}

bool CrossMwmIndexGraph::DoesCrossMwmSectionExist(NumMwmId numMwmId)
{
  if (m_crossMwmIndexGraph.count(numMwmId) != 0)
    return true;

  return GetValue(numMwmId).m_cont.IsExist(CROSS_MWM_FILE_TAG);
}

void CrossMwmIndexGraph::GetTransitions(FeatureType const & ft, bool isOutgoing, vector<Segment> & transitions)
{
  NumMwmId const numMwmId = m_numMwmIds->GetId(CountryFile(ft.GetID().GetMwmName()));

  for (uint32_t segIdx = 0; segIdx < ft.GetPointsCount() - 1; ++segIdx)
  {
    Segment const segForward(numMwmId, ft.GetID().m_index, segIdx, true /* forward */);
    if (IsTransition(segForward, isOutgoing))
      transitions.push_back(segForward);

    if (m_vehicleModelFactory->GetVehicleModelForCountry(ft.GetID().GetMwmName())->IsOneWay(ft))
      return;

    Segment const segBackward(numMwmId, ft.GetID().m_index, segIdx, false /* forward */);
    if (IsTransition(segBackward, isOutgoing))
      transitions.push_back(segBackward);
  }
}
}  // namespace routing
