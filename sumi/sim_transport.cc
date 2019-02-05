/**
Copyright 2009-2018 National Technology and Engineering Solutions of Sandia, 
LLC (NTESS).  Under the terms of Contract DE-NA-0003525, the U.S.  Government 
retains certain rights in this software.

Sandia National Laboratories is a multimission laboratory managed and operated
by National Technology and Engineering Solutions of Sandia, LLC., a wholly 
owned subsidiary of Honeywell International, Inc., for the U.S. Department of 
Energy's National Nuclear Security Administration under contract DE-NA0003525.

Copyright (c) 2009-2018, NTESS

All rights reserved.

Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.

    * Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Questions? Contact sst-macro-help@sandia.gov
*/

#include <cstring>
#include <sumi/transport.h>
#include <sumi/allreduce.h>
#include <sumi/reduce_scatter.h>
#include <sumi/reduce.h>
#include <sumi/allgather.h>
#include <sumi/allgatherv.h>
#include <sumi/alltoall.h>
#include <sumi/alltoallv.h>
#include <sumi/communicator.h>
#include <sumi/bcast.h>
#include <sumi/gather.h>
#include <sumi/scatter.h>
#include <sumi/gatherv.h>
#include <sumi/scatterv.h>
#include <sumi/scan.h>
#include <sprockit/stl_string.h>
#include <sprockit/sim_parameters.h>
#include <sprockit/keyword_registration.h>
#include <sstmac/common/event_callback.h>
#include <sstmac/software/api/api.h>

RegisterKeywords(
{ "lazy_watch", "whether failure notifications can be receive without active pinging" },
{ "eager_cutoff", "what message size in bytes to switch from eager to rendezvous" },
{ "use_put_protocol", "whether to use a put or get protocol for pt2pt sends" },
{ "algorithm", "the specific algorithm to use for a given collecitve" },
{ "comm_sync_stats", "whether to track synchronization stats for communication" },
{ "smp_single_copy_size", "the minimum size of message for single-copy protocol" },
{ "max_eager_msg_size", "the maximum size for using eager pt2pt protocol" },
{ "max_vshort_msg_size", "the maximum size for mailbox protocol" },
{ "post_rdma_delay", "the time it takes to post an RDMA operation" },
{ "post_header_delay", "the time it takes to send an eager message" },
{ "poll_delay", "the time it takes to poll for an incoming message" },
{ "rdma_pin_latency", "the latency for each RDMA pin information" },
{ "rdma_page_delay", "the per-page delay for RDMA pinning" },
);

#include <sstmac/common/sstmac_config.h>
#if SSTMAC_INTEGRATED_SST_CORE
#include <sst/core/event.h>
#endif
#include <sumi/sim_transport.h>
#include <sumi/message.h>
#include <sstmac/software/process/app.h>
#include <sstmac/software/process/operating_system.h>
#include <sstmac/software/process/key.h>
#include <sstmac/software/launch/job_launcher.h>
#include <sstmac/hardware/node/node.h>
#include <sstmac/common/event_callback.h>
#include <sstmac/common/runtime.h>
#include <sstmac/common/stats/stat_spyplot.h>
#include <sprockit/output.h>

using namespace sprockit::dbg;
using sstmac::Timestamp;

RegisterDebugSlot(sumi);

namespace sumi {

const int options::initial_context = -2;

class SumiServer :
  public sstmac::sw::Service
{

 public:
  SumiServer(SimTransport* tport)
    : Service(tport->serverLibname(),
       sstmac::sw::SoftwareId(-1, -1), //belongs to no application
       tport->parent()->os())
  {
  }

  void registerProc(int rank, SimTransport* proc){
    int app_id = proc->sid().app_;
    debug_printf(sprockit::dbg::sumi,
                 "SumiServer registering rank %d for app %d",
                 rank, app_id);
    SimTransport*& slot = procs_[app_id][rank];
    if (slot){
      spkt_abort_printf("SumiServer: already registered rank %d for app %d on node %d",
                        rank, app_id, os_->addr());
    }
    slot = proc;

    auto iter = pending_.begin();
    auto end = pending_.end();
    while (iter != end){
      auto tmp = iter++;
      Message* msg = *tmp;
      if (msg->targetRank() == rank && msg->aid() == proc->sid().app_){
        pending_.erase(tmp);
        proc->incomingMessage(msg);
      }
    }
  }

  bool unregisterProc(int rank, Transport* proc){
    int app_id = proc->sid().app_;
    auto iter = procs_.find(app_id);
    auto& subMap = iter->second;
    subMap.erase(rank);
    if (subMap.empty()){
      procs_.erase(iter);
    }
    return procs_.empty();
  }

  void incomingEvent(sstmac::Event* ev) override {
    sstmac::sw::Service::incomingEvent(ev);
  }

  void incomingRequest(sstmac::Request *req) override {
    Message* smsg = safe_cast(Message, req);
    debug_printf(sprockit::dbg::sumi,
                 "SumiServer %d: incoming %s",
                 os_->addr(), smsg->toString().c_str());
    SimTransport* tport = procs_[smsg->aid()][smsg->targetRank()];
    if (!tport){
      debug_printf(sprockit::dbg::sumi,
                  "SumiServer %d: message pending to app %d, target %d",
                  os_->addr(), smsg->aid(), smsg->targetRank());
      pending_.push_back(smsg);
    } else {
      tport->incomingMessage(smsg);
    }
  }

 private:
  std::map<int, std::map<int, SimTransport*>> procs_;
  std::list<Message*> pending_;

};


Transport* Transport::get()
{
  return sstmac::sw::OperatingSystem::currentThread()->getApi<sumi::SimTransport>("sumi");
}

Transport::~Transport()
{
  if (engine_) delete engine_;
}

SimTransport::SimTransport(SST::Params& params, sstmac::sw::App* parent, SST::Component* comp) :
  //the name of the transport itself should be mapped to a unique name
  API(params, parent, comp),
  Transport("sumi", parent->sid(), parent->os()->addr()),
  //the server is what takes on the specified libname
  spy_num_messages_(nullptr),
  spy_bytes_(nullptr),
  completion_queues_(1),
  default_progress_queue_(parent->os()),
  nic_ioctl_(parent->os()->nicDataIoctl()),
  node_(parent->os()->node())
{
  completion_queues_[0] = std::bind(&DefaultProgressQueue::incoming,
                                    &default_progress_queue_, 0, std::placeholders::_1);

  rank_ = sid().task_;
  auto* server_lib = parent_->os()->lib(server_libname_);
  SumiServer* server;
  // only do one server per app per node
  if (server_lib == nullptr) {
    server = new SumiServer(this);
    server->start();
  } else {
    server = safe_cast(SumiServer, server_lib);
  }

  post_rdma_delay_ = Timestamp(params.findUnits("post_rdma_delay", "0s").toDouble());
  post_header_delay_ = Timestamp(params.findUnits("post_header_delay", "0s").toDouble());
  poll_delay_ = Timestamp(params.findUnits("poll_delay", "0s").toDouble());

  rdma_pin_latency_ = Timestamp(params.findUnits("rdma_pin_latency", "0s").toDouble());
  rdma_page_delay_ = Timestamp(params.findUnits("rdma_page_delay", "0s").toDouble());
  pin_delay_ = rdma_pin_latency_.ticks() || rdma_page_delay_.ticks();
  page_size_ = params.findUnits("rdma_page_size", "4096").getRoundedValue();

  rank_mapper_ = sstmac::sw::TaskMapping::globalMapping(sid().app_);
  nproc_ = rank_mapper_->nproc();

  server->registerProc(rank_, this);

  /** TODO - stats
  spy_num_messages_ = sstmac::optionalStats<sstmac::StatSpyplot>(desScheduler(),
        params, "traffic_matrix", "ascii", "num_messages");
  spy_bytes_ = sstmac::optionalStats<sstmac::StatSpyplot>(desScheduler(),
        params, "traffic_matrix", "ascii", "bytes");
  */
  engine_ = new CollectiveEngine(params, this);
}

void
SimTransport::allocateCq(int id, std::function<void(Message*)>&& f)
{
  completion_queues_[id] = std::move(f);
  auto iter = held_.find(id);
  if (iter != held_.end()){
    auto& list = iter->second;
    for (Message* m : list){
      f(m);
    }
    held_.erase(iter);
  }
}

void
SimTransport::init()
{
  //THIS SHOULD ONLY BE CALLED AFTER RANK and NPROC are known
}

void
SimTransport::finish()
{
  //this should really loop through and kill off all the pings
  //so none of them execute
}

SimTransport::~SimTransport()
{
  SumiServer* server = safe_cast(SumiServer, parent_->os()->lib(server_libname_));
  bool del = server->unregisterProc(rank_, this);
  if (del) delete server;

  //if (spy_bytes_) delete spy_bytes_;
  //if (spy_num_messages_) delete spy_num_messages_;
}

void
SimTransport::pinRdma(uint64_t bytes)
{
  int num_pages = bytes / page_size_;
  if (bytes % page_size_) ++num_pages;
  sstmac::Timestamp pin_delay = rdma_pin_latency_ + num_pages*rdma_page_delay_;
  compute(pin_delay);
}

void
SimTransport::memcopy(uint64_t bytes)
{
  parent_->computeBlockMemcpy(bytes);
}

void
SimTransport::incomingEvent(sstmac::Event *ev)
{
  spkt_abort_printf("sumi_transport::incoming_event: should not directly handle events");
}

int*
SimTransport::nidlist() const
{
  //just cast an int* - it's fine
  //the types are the same size and the bits can be
  //interpreted correctly
  return (int*) rank_mapper_->rankToNode().data();
}

void
SimTransport::compute(sstmac::Timestamp t)
{
  parent_->compute(t);
}


void
SimTransport::send(Message* m)
{
#if SSTMAC_COMM_SYNC_STATS
  msg->setTimeSent(wall_time());
#endif
  if (spy_num_messages_) spy_num_messages_->addOne(m->sender(), m->recver());
  if (spy_bytes_){
    switch(m->sstmac::hw::NetworkMessage::type()){
    case sstmac::hw::NetworkMessage::payload:
      spy_bytes_->addData(m->sender(), m->recver(), m->byteLength());
      break;
    case sstmac::hw::NetworkMessage::rdma_get_request:
    case sstmac::hw::NetworkMessage::rdma_put_payload:
      spy_bytes_->addData(m->sender(), m->recver(), m->payloadBytes());
      break;
    default:
      break;
    }
  }

  switch(m->sstmac::hw::NetworkMessage::type()){
    case sstmac::hw::NetworkMessage::payload:
      if (m->recver() == rank_){
        //deliver to self
        debug_printf(sprockit::dbg::sumi,
          "Rank %d SUMI sending self message", rank_);
        if (m->needsRecvAck()){
          completion_queues_[m->recvCQ()](m);
        }
        if (m->needsSendAck()){
          auto* ack = m->cloneInjectionAck();
          completion_queues_[m->sendCQ()](static_cast<Message*>(ack));
        }
      } else {
        if (post_header_delay_.ticks()) {
          parent_->compute(post_header_delay_);
        }
        nic_ioctl_(m);
      }
      break;
    case sstmac::hw::NetworkMessage::rdma_get_request:
    case sstmac::hw::NetworkMessage::rdma_put_payload:
      if (post_rdma_delay_.ticks()) {
        parent_->compute(post_rdma_delay_);
      }
      nic_ioctl_(m);
      break;
    default:
      spkt_abort_printf("attempting to initiate send with invalid type %d",
                        m->type())
  }
}

void
SimTransport::smsgSendResponse(Message* m, uint64_t size, void* buffer, int local_cq, int remote_cq)
{
  //reverse both hardware and software info
  m->sstmac::hw::NetworkMessage::reverse();
  m->reverse();
  m->setupSmsg(buffer, size);
  m->setSendCq(local_cq);
  m->setRecvCQ(remote_cq);
  m->sstmac::hw::NetworkMessage::setType(Message::payload);
  send(m);
}

void
SimTransport::rdmaGetRequestResponse(Message* m, uint64_t size,
                                     void* local_buffer, void* remote_buffer,
                                     int local_cq, int remote_cq)
{
  //do not reverse send/recver - this is hardware reverse, not software reverse
  m->sstmac::hw::NetworkMessage::reverse();
  m->setupRdmaGet(local_buffer, remote_buffer, size);
  m->setSendCq(remote_cq);
  m->setRecvCQ(local_cq);
  m->sstmac::hw::NetworkMessage::setType(Message::rdma_get_request);
  send(m);
}

void
SimTransport::rdmaGetResponse(Message* m, uint64_t size, int local_cq, int remote_cq)
{
  smsgSendResponse(m, size, nullptr, local_cq, remote_cq);
}

void
SimTransport::rdmaPutResponse(Message* m, uint64_t payload_bytes,
                 void* loc_buffer, void* remote_buffer, int local_cq, int remote_cq)
{
  m->reverse();
  m->sstmac::hw::NetworkMessage::reverse();
  m->setupRdmaPut(loc_buffer, remote_buffer, payload_bytes);
  m->setSendCq(local_cq);
  m->setRecvCQ(remote_cq);
  m->sstmac::hw::NetworkMessage::setType(Message::rdma_put_payload);
  send(m);
}

uint64_t
SimTransport::allocateFlowId()
{
  return parent_->os()->node()->allocateUniqueId();
}

void
SimTransport::incomingMessage(Message *msg)
{
#if SSTMAC_COMM_SYNC_STATS
  if (msg){
    msg->get_payload()->setTimeArrived(wall_time());
  }
#endif

  int cq = msg->isNicAck() ? msg->sendCQ() : msg->recvCQ();
  if (cq != Message::no_ack){
    if (cq >= completion_queues_.size()){
      debug_printf(sprockit::dbg::sumi, "No CQ yet for %s", msg->toString().c_str());
      held_[cq].push_back(msg);
    } else {
      completion_queues_[cq](msg);
    }
  } else {
    debug_printf(sprockit::dbg::sumi, "Dropping message without CQ: %s", msg->toString().c_str());
    delete msg;
  }
}

CollectiveEngine::CollectiveEngine(SST::Params& params, Transport *tport) :
  system_collective_tag_(-1), //negative tags reserved for special system work
  eager_cutoff_(512),
  use_put_protocol_(false),
  global_domain_(nullptr),
  tport_(tport)
{
  global_domain_ = new GlobalCommunicator(tport);
  eager_cutoff_ = params.find<int>("eager_cutoff", 512);
  use_put_protocol_ = params.find<bool>("use_put_protocol", false);
}

CollectiveEngine::~CollectiveEngine()
{
  if (global_domain_) delete global_domain_;
}

void
CollectiveEngine::notifyCollectiveDone(int rank, Collective::type_t ty, int tag)
{
  Collective* coll = collectives_[ty][tag];
  if (!coll){
    spkt_throw_printf(sprockit::value_error,
      "transport::notify_collective_done: invalid collective of type %s, tag %d",
       Collective::tostr(ty), tag);
  }
  finishCollective(coll, rank, ty, tag);
}

void
CollectiveEngine::deadlockCheck()
{
  collective_map::iterator it, end = collectives_.end();
  for (it=collectives_.begin(); it != end; ++it){
    tag_to_collective_map& next = it->second;
    tag_to_collective_map::iterator cit, cend = next.end();
    for (cit=next.begin(); cit != cend; ++cit){
      Collective* coll = cit->second;
      if (!coll->complete()){
        coll->deadlockCheck();
      }
    }
  }
}

CollectiveDoneMessage*
CollectiveEngine::skipCollective(Collective::type_t ty,
  int cq_id, Communicator* comm,
  void* dst, void *src,
  int nelems, int type_size,
  int tag)
{
  if (!comm) comm = global_domain_;
  if (comm->nproc() == 1){
    if (dst && src && (dst != src)){
      ::memcpy(dst, src, nelems*type_size);
    }
    return new CollectiveDoneMessage(tag, ty, comm, cq_id);
  } else {
    return nullptr;
  }
}

CollectiveDoneMessage*
CollectiveEngine::allreduce(void* dst, void *src, int nelems, int type_size, int tag, reduce_fxn fxn,
                             int cq_id, Communicator* comm)
{
 auto* msg = skipCollective(Collective::allreduce, cq_id, comm, dst, src, nelems, type_size, tag);
 if (msg) return msg;

  if (!comm) comm = global_domain_;
  DagCollective* coll = new WilkeHalvingAllreduce(this, dst, src, nelems, type_size, tag, fxn, cq_id, comm);
  return startCollective(coll);
}

sumi::CollectiveDoneMessage*
CollectiveEngine::reduceScatter(void* dst, void *src, int nelems, int type_size, int tag, reduce_fxn fxn,
                                  int cq_id, Communicator* comm)
{
  auto* msg = skipCollective(Collective::reduce_scatter, cq_id, comm, dst, src, nelems, type_size, tag);
  if (msg) return msg;

  if (!comm) comm = global_domain_;
  DagCollective* coll = new HalvingReduceScatter(this, dst, src, nelems, type_size, tag, fxn, cq_id, comm);
  return startCollective(coll);
}

sumi::CollectiveDoneMessage*
CollectiveEngine::scan(void* dst, void* src, int nelems, int type_size, int tag, reduce_fxn fxn,
                        int cq_id, Communicator* comm)
{
  auto* msg = skipCollective(Collective::scan, cq_id, comm, dst, src, nelems, type_size, tag);
  if (msg) return msg;

  if (!comm) comm = global_domain_;
  DagCollective* coll = new SimultaneousBtreeScan(this, dst, src, nelems, type_size, tag, fxn, cq_id, comm);
  return startCollective(coll);
}


CollectiveDoneMessage*
CollectiveEngine::reduce(int root, void* dst, void *src, int nelems, int type_size, int tag, reduce_fxn fxn,
                          int cq_id, Communicator* comm)
{
  auto* msg = skipCollective(Collective::reduce, cq_id, comm, dst, src, nelems, type_size, tag);
  if (msg) return msg;

  if (!comm) comm = global_domain_;
  DagCollective* coll = new WilkeHalvingReduce(this, root, dst, src, nelems, type_size, tag, fxn, cq_id, comm);
  return startCollective(coll);
}

CollectiveDoneMessage*
CollectiveEngine::bcast(int root, void *buf, int nelems, int type_size, int tag,
                         int cq_id, Communicator* comm)
{
  auto* msg = skipCollective(Collective::bcast, cq_id, comm, buf, buf, nelems, type_size, tag);
  if (msg) return msg;

  if (!comm) comm = global_domain_;
  DagCollective* coll = new BinaryTreeBcastCollective(this, root, buf, nelems, type_size, tag, cq_id, comm);
  return startCollective(coll);
}

CollectiveDoneMessage*
CollectiveEngine::gatherv(int root, void *dst, void *src,
                   int sendcnt, int *recv_counts,
                   int type_size, int tag, int cq_id, Communicator* comm)
{
  auto* msg = skipCollective(Collective::gatherv, cq_id, comm, dst, src, sendcnt, type_size, tag);
  if (msg) return msg;

  if (!comm) comm = global_domain_;
  DagCollective* coll = new BtreeGatherv(this, root, dst, src, sendcnt, recv_counts, type_size, tag, cq_id, comm);
  sprockit::abort("gatherv");
  return startCollective(coll);
}

CollectiveDoneMessage*
CollectiveEngine::gather(int root, void *dst, void *src, int nelems, int type_size, int tag,
                          int cq_id, Communicator* comm)
{
  auto* msg = skipCollective(Collective::gather, cq_id, comm, dst, src, nelems, type_size, tag);
  if (msg) return msg;

  if (!comm) comm = global_domain_;
  DagCollective* coll = new BtreeGather(this, root, dst, src, nelems, type_size, tag, cq_id, comm);
  return startCollective(coll);
}

CollectiveDoneMessage*
CollectiveEngine::scatter(int root, void *dst, void *src, int nelems, int type_size, int tag,
                           int cq_id, Communicator* comm)
{
  auto* msg = skipCollective(Collective::scatter, cq_id, comm, dst, src, nelems, type_size, tag);
  if (msg) return msg;

  if (!comm) comm = global_domain_;
  DagCollective* coll = new BtreeScatter(this, root, dst, src, nelems, type_size, tag, cq_id, comm);
  return startCollective(coll);
}

CollectiveDoneMessage*
CollectiveEngine::scatterv(int root, void *dst, void *src, int* send_counts, int recvcnt, int type_size, int tag,
                            int cq_id, Communicator* comm)
{
  auto* msg = skipCollective(Collective::scatterv, cq_id, comm, dst, src, recvcnt, type_size, tag);
  if (msg) return msg;

  if (!comm) comm = global_domain_;
  DagCollective* coll = new BtreeScatterv(this, root, dst, src, send_counts, recvcnt, type_size, tag, cq_id, comm);
  sprockit::abort("scatterv");
  return startCollective(coll);
}

CollectiveDoneMessage*
CollectiveEngine::alltoall(void *dst, void *src, int nelems, int type_size, int tag,
                            int cq_id, Communicator* comm)
{
  auto* msg = skipCollective(Collective::alltoall, cq_id, comm, dst, src, nelems, type_size, tag);
  if (msg) return msg;

  if (!comm) comm = global_domain_;
  DagCollective* coll = new BruckAlltoallCollective(this, dst, src, nelems, type_size, tag, cq_id, comm);
  return startCollective(coll);
}

CollectiveDoneMessage*
CollectiveEngine::alltoallv(void *dst, void *src, int* send_counts, int* recv_counts, int type_size, int tag,
                             int cq_id, Communicator* comm)
{
  auto* msg = skipCollective(Collective::alltoallv, cq_id, comm, dst, src, send_counts[0], type_size, tag);
  if (msg) return msg;

  if (!comm) comm = global_domain_;
  DagCollective* coll = new DirectAlltoallvCollective(this, dst, src, send_counts, recv_counts, type_size, tag, cq_id, comm);
  return startCollective(coll);
}

CollectiveDoneMessage*
CollectiveEngine::allgather(void *dst, void *src, int nelems, int type_size, int tag,
                             int cq_id, Communicator* comm)
{
 auto* msg = skipCollective(Collective::allgather, cq_id, comm, dst, src, nelems, type_size, tag);
 if (msg) return msg;

  if (!comm) comm = global_domain_;
  DagCollective* coll = new BruckAllgatherCollective(
        Collective::allgather, this, dst, src, nelems, type_size, tag, cq_id, comm);
  return startCollective(coll);
}

CollectiveDoneMessage*
CollectiveEngine::allgatherv(void *dst, void *src, int* recv_counts, int type_size, int tag,
                              int cq_id, Communicator* comm)
{
  //if the allgatherv is skipped, we have a single recv count
  int nelems = *recv_counts;
  auto* msg = skipCollective(Collective::allgatherv, cq_id, comm, dst, src, nelems, type_size, tag);
  if (msg) return msg;

  if (!comm) comm = global_domain_;
  DagCollective* coll = new BruckAllgathervCollective(this, dst, src, recv_counts, type_size, tag, cq_id, comm);
  return startCollective(coll);
}

CollectiveDoneMessage*
CollectiveEngine::barrier(int tag, int cq_id, Communicator* comm)
{
  auto* msg = skipCollective(Collective::barrier, cq_id, comm, 0, 0, 0, 0, tag);
  if (msg) return msg;

  if (!comm) comm = global_domain_;
  DagCollective* coll = new BruckAllgatherCollective(Collective::barrier, this, nullptr, nullptr, 0, 0, tag, cq_id, comm);
  return startCollective(coll);
}

CollectiveDoneMessage*
CollectiveEngine::deliverPending(Collective* coll, int tag, Collective::type_t ty)
{
  std::list<CollectiveWorkMessage*> pending = pending_collective_msgs_[ty][tag];
  pending_collective_msgs_[ty].erase(tag);
  CollectiveDoneMessage* dmsg = nullptr;
  for (auto* msg : pending){
    dmsg = coll->recv(msg);
  }
  return dmsg;
}

void
CollectiveEngine::validateCollective(Collective::type_t ty, int tag)
{
  tag_to_collective_map::iterator it = collectives_[ty].find(tag);
  if (it == collectives_[ty].end()){
    return; // all good
  }

  Collective* coll = it->second;
  if (!coll){
   spkt_throw_printf(sprockit::illformed_error,
    "sumi_api::validate_collective: lingering null collective of type %s with tag %d",
    Collective::tostr(ty), tag);
  }

  if (coll->persistent() && coll->complete()){
    return; // all good
  }

  spkt_throw_printf(sprockit::illformed_error,
    "sumi_api::validate_collective: cannot overwrite collective of type %s with tag %d",
    Collective::tostr(ty), tag);
}

CollectiveDoneMessage*
CollectiveEngine::startCollective(Collective* coll)
{
  coll->initActors();
  int tag = coll->tag();
  Collective::type_t ty = coll->type();
  //validate_collective(ty, tag);
  Collective*& existing = collectives_[ty][tag];
  if (existing){
    coll->start();
    auto* msg = existing->addActors(coll);
    delete coll;
    return msg;
  } else {
    existing = coll;
    coll->start();
    return deliverPending(coll, tag, ty);
  }
}

void
CollectiveEngine::finishCollective(Collective* coll, int rank, Collective::type_t ty, int tag)
{
  bool deliver_cq_msg; bool delete_collective;
  coll->actorDone(rank, deliver_cq_msg, delete_collective);
  debug_printf(sprockit::dbg::sumi,
    "Rank %d finishing collective of type %s tag %d - deliver=%d",
    tport_->rank(), Collective::tostr(ty), tag, deliver_cq_msg);

  if (!deliver_cq_msg)
    return;

  coll->complete();
  if (delete_collective && !coll->persistent()){ //otherwise collective must exist FOREVER
    collectives_[ty].erase(tag);
    todel_.push_back(coll);
  }

  pending_collective_msgs_[ty].erase(tag);
  debug_printf(sprockit::dbg::sumi,
    "Rank %d finished collective of type %s tag %d",
    tport_->rank(), Collective::tostr(ty), tag);
}

void
CollectiveEngine::waitBarrier(int tag)
{
  if (tport_->nproc() == 1) return;
  barrier(tag, Message::default_cq);
  auto* dmsg = blockUntilNext(Message::default_cq);
}

void
CollectiveEngine::cleanUp()
{
  for (Collective* coll : todel_){
    delete coll;
  }
  todel_.clear();
}

CollectiveDoneMessage*
CollectiveEngine::incoming(Message* msg)
{
  cleanUp();

  CollectiveWorkMessage* cmsg = dynamic_cast<CollectiveWorkMessage*>(msg);
  if (cmsg->sendCQ() == -1 && cmsg->recvCQ() == -1){
    spkt_abort_printf("both CQs are invalid for %s", msg->toString().c_str())
  }
  int tag = cmsg->tag();
  Collective::type_t ty = cmsg->type();
  tag_to_collective_map::iterator it = collectives_[ty].find(tag);
  if (it == collectives_[ty].end()){
    debug_printf(sprockit::dbg::sumi_collective,
      "Rank %d, queuing %p %s from %d on tag %d for type %s",
      tport_->rank(), msg,
      Message::tostr(msg->classType()),
      msg->sender(),
      tag, Collective::tostr(ty));
      //message for collective we haven't started yet
      pending_collective_msgs_[ty][tag].push_back(cmsg);
      return nullptr;
  } else {
    Collective* coll = it->second;
    auto* dmsg = coll->recv(cmsg);
    return dmsg;
  }
}

CollectiveDoneMessage*
CollectiveEngine::blockUntilNext(int cq_id)
{
  CollectiveDoneMessage* dmsg = nullptr;
  while (dmsg == nullptr){
    debug_printf(sprockit::dbg::sumi_collective,
      "Rank %d, blocking collective until next message arrives on CQ %d", tport_->rank(), cq_id);
    auto* msg = tport_->blockingPoll(cq_id);
    debug_printf(sprockit::dbg::sumi_collective,
      "Rank %d, unblocking collective on CQ %d", tport_->rank(), cq_id);
    dmsg = incoming(msg);
  }
  debug_printf(sprockit::dbg::sumi_collective,
    "Rank %d, exiting collective progress on CQ %d", tport_->rank(), cq_id);
  return dmsg;
}


}
