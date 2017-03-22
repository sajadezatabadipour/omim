#include "drape_frontend/metaline_manager.hpp"

#include "drape_frontend/map_data_provider.hpp"
#include "drape_frontend/threads_commutator.hpp"

#include <functional>

namespace df
{

MetalineManager::MetalineManager(ref_ptr<ThreadsCommutator> commutator, MapDataProvider & model)
  : m_model(model)
  , m_tasksPool(4, ReadMetalineTaskFactory(m_model))
  , m_threadsPool(make_unique_dp<threads::ThreadPool>(2, std::bind(&MetalineManager::OnTaskFinished,
                                                                   this, std::placeholders::_1)))
  , m_commutator(commutator)
{
  ReadMetalineTask * task = m_tasksPool.Get();
  task->Init(m_commutator);
  m_threadsPool->PushBack(task);
}

MetalineManager::~MetalineManager()
{
  m_threadsPool->Stop();
  m_threadsPool.reset();
}

void MetalineManager::OnTaskFinished(threads::IRoutine * task)
{
  ASSERT(dynamic_cast<ReadMetalineTask *>(task) != nullptr, ());
  ReadMetalineTask * t = static_cast<ReadMetalineTask *>(task);

  t->Reset();
  m_tasksPool.Return(t);
}

}  // namespace df
