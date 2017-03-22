#include "drape_frontend/read_metaline_task.hpp"

#include "drape_frontend/drape_api.hpp"
#include "drape_frontend/map_data_provider.hpp"
#include "drape_frontend/message_subclasses.hpp"
#include "drape_frontend/metaline_manager.hpp"
#include "drape_frontend/threads_commutator.hpp"

#include "indexer/feature_decl.hpp"
#include "indexer/scales.hpp"

#include "base/string_utils.hpp"

#include <algorithm>
#include <fstream>

namespace
{
// Temporary!
df::MetalineModel ReadMetalinesFromFile(MwmSet::MwmId const & mwmId)
{
  df::MetalineModel model;
  std::ifstream f("road_segments.txt");
  if (!f.is_open())
    return model;
  while (!f.eof())
  {
    std::string s;
    f >> s;
    auto ids = strings::Tokenize(s, ",");
    df::MetalineData data;
    for (auto const & id : ids)
    {
      uint64_t i = 0;
      strings::to_uint64(id, i);
      if (i == 0 || i > 629886)
        break;
      data.m_features.push_back(FeatureID(mwmId, (uint32_t)i));
    }
    model.push_back(std::move(data));
  }
  f.close();
  return model;
}

vector<dp::Color> colorList = { dp::Color(255, 0, 0, 255), dp::Color(0, 255, 0, 255), dp::Color(0, 0, 255, 255),
                                dp::Color(255, 255, 0, 255), dp::Color(0, 255, 255, 255), dp::Color(255, 0, 255, 255),
                                dp::Color(100, 0, 0, 255), dp::Color(0, 100, 0, 255), dp::Color(0, 0, 100, 255),
                                dp::Color(100, 100, 0, 255), dp::Color(0, 100, 100, 255), dp::Color(100, 0, 100, 255)
};

std::vector<m2::PointD> MergePoints(std::vector<std::vector<m2::PointD>> & points)
{
  std::vector<m2::PointD> result;
  std::vector<double> dists;
  dists.resize(4);
  for (size_t i = 1; i < points.size(); i++)
  {
    auto & p1 = points[i - 1];
    auto & p2 = points[i];

    dists[0] = (p1.front() - p2.front()).Length();
    dists[1] = (p1.back() - p2.front()).Length();
    dists[2] = (p1.back() - p2.back()).Length();
    dists[3] = (p1.front() - p2.back()).Length();
    auto minIndex = std::distance(dists.begin(), std::min_element(dists.begin(), dists.end()));
    if (dists[minIndex] > 0.001)
      minIndex = -1;
    switch (minIndex)
    {
    case 0:
      std::reverse(p1.begin(), p1.end());
      break;
    case 1:
      // Nothing to reverse.
      break;
    case 2:
      std::reverse(p2.begin(), p2.end());
      break;
    case 3:
      std::reverse(p1.begin(), p1.end());
      std::reverse(p2.begin(), p2.end());
      break;
    default:
      // Points are not connected.
      return std::vector<m2::PointD>();
    }
    for (auto const & pt : p1)
    {
      if (result.empty() || !result.back().EqualDxDy(pt, 1e-5))
        result.push_back(pt);
    }
  }
  return result;
}
}  // namespace

namespace df
{
ReadMetalineTask::ReadMetalineTask(MapDataProvider & model)
  : m_model(model)
{}

void ReadMetalineTask::Init(ref_ptr<ThreadsCommutator> commutator)
{
  m_commutator = commutator;
}

void ReadMetalineTask::Reset()
{
}

bool ReadMetalineTask::IsCancelled() const
{
  return IRoutine::IsCancelled();
}

void ReadMetalineTask::Do()
{
  // Temporary! Load metaline data from file.
  MwmSet::MwmId mwmId;
  m_model.ReadFeaturesID([&mwmId](FeatureID const & fid)
  {
    if (!mwmId.IsAlive())
      mwmId = fid.m_mwmId;
  }, m2::RectD(m2::PointD(37.5380324, 67.5384520), m2::PointD(37.5380325, 67.5384521)), 17);
  auto metalines = ReadMetalinesFromFile(mwmId);
  if (!mwmId.IsAlive() || mwmId.GetInfo()->GetCountryName() != "Russia_Moscow")
    return;

  size_t colorIndex = 0;
  for (auto const & metaline : metalines)
  {
    bool failed = false;
    for (auto const & fid : metaline.m_features)
    {
      if (m_cache.find(fid) != m_cache.end())
      {
        failed = true;
        break;
      }
    }
    if (failed)
      continue;

    size_t curIndex = 0;
    std::vector<std::vector<m2::PointD>> points;
    points.reserve(5);
    m_model.ReadFeatures([&metaline, &failed, &curIndex, &points](FeatureType const & ft)
    {
      if (failed)
        return;
      if (ft.GetID() != metaline.m_features[curIndex])
      {
        failed = true;
        return;
      }
      std::vector<m2::PointD> featurePoints;
      featurePoints.reserve(5);
      ft.ForEachPoint([&featurePoints](m2::PointD const & pt)
      {
        if (!featurePoints.back().EqualDxDy(pt, 1e-5))
          featurePoints.push_back(pt);
      }, scales::GetUpperScale());
      if (featurePoints.size() < 2)
      {
        failed = true;
        return;
      }
      points.push_back(std::move(featurePoints));
      curIndex++;
    }, metaline.m_features);

    if (failed || points.empty())
      continue;

    std::vector<m2::PointD> mergedPoints = MergePoints(points);
    if (mergedPoints.empty())
      continue;

    //if (!metaline.m_features.empty() && metaline.m_features.front().m_index == 19392)
    //{
    //  DrapeApi::TLines lines;
    //  size_t const cc = colorIndex++ % colorList.size();
    //  lines.insert(std::make_pair(DebugPrint(metaline.m_features.front()),
    //                              df::DrapeApiLineData(mergedPoints, colorList[cc]).Width(3.0f).ShowPoints(true)));
    //  m_commutator->PostMessage(ThreadsCommutator::ResourceUploadThread,
    //                            make_unique_dp<DrapeApiAddLinesMessage>(lines),
    //                            MessagePriority::Normal);
    //}

    m2::SharedSpline spline(mergedPoints);
    for (auto const & fid : metaline.m_features)
      m_cache[fid] = spline;
  }
}

ReadMetalineTask * ReadMetalineTaskFactory::GetNew() const
{
  return new ReadMetalineTask(m_model);
}

} // namespace df
