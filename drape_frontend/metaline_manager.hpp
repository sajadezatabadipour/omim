#pragma once

#include "drape_frontend/read_metaline_task.hpp"

#include "drape/object_pool.hpp"
#include "drape/pointers.hpp"

#include "indexer/feature_decl.hpp"

#include <vector>

namespace df
{

struct MetalineData
{
  std::vector<FeatureID> m_features;
};

using MetalineModel = std::vector<MetalineData>;

class ThreadsCommutator;

class MetalineManager
{
public:
  MetalineManager(ref_ptr<ThreadsCommutator> commutator, MapDataProvider & model);
  ~MetalineManager();

private:
  void OnTaskFinished(threads::IRoutine * task);

  using TasksPool = ObjectPool<ReadMetalineTask, ReadMetalineTaskFactory>;

  MapDataProvider & m_model;
  TasksPool m_tasksPool;
  drape_ptr<threads::ThreadPool> m_threadsPool;
  ref_ptr<ThreadsCommutator> m_commutator;
};

}  // namespace df
