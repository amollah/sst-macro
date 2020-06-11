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

#include <sstmac/hardware/router/router.h>
#include <sstmac/hardware/switch/network_switch.h>
#include <sstmac/hardware/topology/dragonfly.h>
#include <sstmac/hardware/topology/dragonfly_plus.h>
#include <sprockit/util.h>
#include <sprockit/sim_parameters.h>
#include <cmath>

#define ftree_rter_debug(...) \
  rter_debug("fat tree: %s", sprockit::printf(__VA_ARGS__).c_str())
using namespace std;
namespace sstmac {
namespace hw {


typedef int (*mytype)[MAX_ROUTER][MAX_PORT];
typedef double (*mytype2)[MAX_ROUTER][MAX_PORT];


class DragonflyPlusAlltoallMinimalRouter : public Router {
 public:
  struct header : public Packet::Header {};

  SST_ELI_REGISTER_DERIVED(
    Router,
    DragonflyPlusAlltoallMinimalRouter,
    "macro",
    "dragonfly_plus_alltoall_minimal",
    SST_ELI_ELEMENT_VERSION(1,0,0),
    "router implementing minimal routing for dragonfly+")

  DragonflyPlusAlltoallMinimalRouter(SST::Params& params, Topology *top,
                         NetworkSwitch *netsw)
    : Router(params, top, netsw)
  {
    dfly_ = safe_cast(DragonflyPlus, top);
    num_leaf_switches_ = dfly_->g() * dfly_->a();
    //stagger by switch id
    rotater_ = (my_addr_) % dfly_->a();

    ////covering_ = dfly_->h() / (dfly_->g() - 1);
    /*if (covering_ == 0){
      spkt_abort_printf("dragonfly+ minimal router for alltoall wiring does"
                        " not have full covering");
    }*/

    /*int mod = dfly_->h() % (dfly_->g() - 1);
    if (mod != 0){
      spkt_abort_printf("dragonfly+ group connections h=%d is not evenly divided by N-1 for N=%d groups",
                        dfly_->h(), dfly_->g());
    }*/

    grp_rotaters_.resize(dfly_->g());
    for (int i=0; i < dfly_->g(); ++i){
      grp_rotaters_[i] = 0;
    }

    my_g_ = (my_addr_%num_leaf_switches_) / dfly_->a();
    my_row_ = my_addr_ / num_leaf_switches_;

   //figure out which groups it is directly connected to and check of correctness
    std::vector<int> connected;
    my_a = dfly_->computeA(my_addr_);
    dfly_->groupWiring()->connectedRouters(my_a, my_g_, connected);
    /*for (int p=0; p < connected.size(); ++p){
      int my_expected_g = p / covering_;
      if (my_expected_g >= my_g_){
        ++my_expected_g;
      }
      int dst = connected[p];
      int actual_g = dfly_->computeG(dst);
      if (my_expected_g != actual_g){
        spkt_abort_printf("Router %d expected group %d on port %d, but got group %d",
                          my_addr_, my_expected_g, p, actual_g);
      }
    }*/

    static_route_ = params.find<bool>("static", false);
  }

  int numVC() const override {
    return 1;
  }

  std::string toString() const override {
    return "dragonfly+ minimal circulant router";
  }

  void route(Packet *pkt) override {
    auto* hdr = pkt->rtrHeader<header>();
    SwitchId ej_addr = pkt->toaddr() / dfly_->concentration();
    if (ej_addr == my_addr_){
      hdr->edge_port = pkt->toaddr() % dfly_->concentration() + dfly_->a();
      hdr->deadlock_vc = 0;
      return;
    }

    int dstG = (ej_addr % num_leaf_switches_) / dfly_->a();
	std::vector<std::pair<int,int>> groupConnections;
	if(my_g_ != dstG){
	dfly_->groupWiring()->connectedToGroup(my_g_, dstG, groupConnections);
	if (groupConnections.size() == 0){
        spkt_abort_printf("Got zero group connections from %d->%d", my_g_, dstG);
      }
	}
    if (my_row_ == 0){
      if (static_route_){
		  if(my_g_ == dstG)
			hdr->edge_port = ej_addr % dfly_->a();
          else
			hdr->edge_port = groupConnections[0].first;
      } else {
		if(my_g_ == dstG)
		{
			hdr->edge_port = rotater_;
            rotater_ = (rotater_ + 1) % dfly_->a();
		}
		else
		{
			hdr->edge_port = groupConnections[rotate_g].first;
            rotate_g = (rotate_g + 1) % groupConnections.size();
		}
      }
    } else if (my_g_ == dstG){
      int dstA = ej_addr % dfly_->a();
      hdr->edge_port = dstA;
    } else {
	  int port_no;
	  std::vector<int> con; 
	  for(int i=0;i<groupConnections.size();i++)
	  {
		  if(groupConnections[i].first == my_a)
		  {
			  con.push_back(groupConnections[i].second);
		  }
	  }
	  int con_index = rand()%con.size();
	  port_no = con[con_index];
	  int port = (port_no%dfly_->h())+ dfly_->a();
	  hdr->edge_port = port;
    }
    hdr->deadlock_vc = 0;
  }

 protected:
  int rotate_g = 0;
  int num_leaf_switches_;
  int rotater_;
  int my_a;
  int my_g_;
  int my_row_;
  std::vector<int> grp_rotaters_;
  int covering_;
  DragonflyPlus* dfly_;
  bool static_route_;
};

class DragonflyPlusParRouter : public DragonflyPlusAlltoallMinimalRouter {
  struct header : public DragonflyPlusAlltoallMinimalRouter::header {
    uint8_t stage_number : 4;
  };
 public:
  SST_ELI_REGISTER_DERIVED(
    Router,
    DragonflyPlusParRouter,
    "macro",
    "dragonfly_plus_par",
    SST_ELI_ELEMENT_VERSION(1,0,0),
    "router implementing PAR for dragonfly+")

  static const char initial_stage = 0;
  static const char valiant_stage1 = 1;
  static const char valiant_stage2 = 2;
  static const char final_stage = 3;

  std::string toString() const override {
    return "dragonfly+ PAR router";
  }

  DragonflyPlusParRouter(SST::Params& params, Topology *top,
                       NetworkSwitch *netsw)
    : DragonflyPlusAlltoallMinimalRouter(params, top, netsw)
  {
    dfly_ = safe_cast(DragonflyPlus, top);
    my_row_ = my_addr_ / dfly_->numLeafSwitches();
    my_g_ = (my_addr_ % dfly_->numLeafSwitches()) / dfly_->a();
	my_a = dfly_->computeA(my_addr_);
    //covering_ = dfly_->h() / (dfly_->g() - 1);
    grp_rotaters_.resize(dfly_->g());
    for (int i=0; i < dfly_->g(); ++i){
      grp_rotaters_[i] = 0;
    }
    up_rotater_ = 0;
	rotate_g = 0;
    num_leaf_switches_ = dfly_->numLeafSwitches();
  }

  void route(Packet *pkt) override {
    SwitchId ej_addr = pkt->toaddr() / dfly_->concentration();
	
	int dstG = (ej_addr % num_leaf_switches_) / dfly_->a();
	std::vector<std::pair<int,int>> groupConnections;
	if(my_g_ != dstG){
	dfly_->groupWiring()->connectedToGroup(my_g_, dstG, groupConnections);
	if (groupConnections.size() == 0){
        spkt_abort_printf("Got zero group connections from %d->%d", my_g_, dstG);
      }
	}
	
    auto hdr = pkt->rtrHeader<header>();
    if (my_row_ == 0){
      if (ej_addr == my_addr_){
        hdr->edge_port = pkt->toaddr() % dfly_->concentration() + dfly_->a();
        hdr->deadlock_vc = 0;
      } else {
        //gotta route up
        //rter_debug("routing up on %d", up_rotater_);
		if(my_g_ == dstG)
		{
			hdr->edge_port = up_rotater_;
            up_rotater_ = (up_rotater_ + 1) % dfly_->a();
		}
        else
		{
			hdr->edge_port = groupConnections[rotate_g].first;
            rotate_g = (rotate_g + 1) % groupConnections.size();
		}
        hdr->deadlock_vc = 0;
      }
    } else {
      if (my_g_ == dstG){
        //go down to the eject stage
        int dstA = ej_addr % dfly_->a();
        hdr->edge_port = dstA;
        //don't change the vc
        //rter_debug("routing down to %d", int(hdr->edge_port));
      } else if (hdr->stage_number == valiant_stage1) {
		  int port_no;
		  std::vector<int> con; 
	    for(int i=0;i<groupConnections.size();i++)
	    {
		  if(groupConnections[i].first == my_a)
		  {
			  con.push_back(groupConnections[i].second);
		  }
	    }
		if(con.size()==0){
		  //down route packet
		  int dstA = rand() % dfly_->a();
		  hdr->stage_number = valiant_stage2;
          hdr->edge_port = dstA;
		}
		else{
		int con_index = rand()%con.size();
	    port_no = con[con_index];
	    int port = (port_no%dfly_->h())+ dfly_->a();
	    hdr->edge_port = port;
		hdr->stage_number = final_stage;
		hdr->deadlock_vc = 1;
		}
      } else if(hdr->stage_number == valiant_stage2){
		int port_no;
	    std::vector<int> con; 
	    for(int i=0;i<groupConnections.size();i++)
	    {
		  if(groupConnections[i].first == my_a)
		  {
			  con.push_back(groupConnections[i].second);
		  }
	    }
	    int con_index = rand()%con.size();
	    port_no = con[con_index];
	    int port = (port_no%dfly_->h())+ dfly_->a();
	    hdr->edge_port = port;
		hdr->stage_number = final_stage;
		hdr->deadlock_vc = 1;
		
	  }else {
		int port_no;
	  std::vector<int> con; 
	  for(int i=0;i<groupConnections.size();i++)
	  {
		  if(groupConnections[i].first == my_a)
		  {
			  con.push_back(groupConnections[i].second);
		  }
	  }
	  int con_index = rand()%con.size();
	  port_no = con[con_index];
        int minimalPort = (port_no%dfly_->h())+ dfly_->a();
        if (dfly_->g() > 2){
          //we must have an intermediate group - otherwise we just have minimal
          //we must make a ugal decision here
          int valiantPort;
		  do{  
		  valiantPort = (rand()%dfly_->h()) + dfly_->a();
		  } while (valiantPort==minimalPort);
		  
          int valiantMetric = 2*netsw_->queueLength(valiantPort, all_vcs);
          int minimalMetric = netsw_->queueLength(minimalPort, all_vcs);

          //rter_debug("comparing minimal(%d) %d against non-minimal(%d) %d",
                     //minimalPort, minimalMetric, valiantPort, valiantMetric);

          if (minimalMetric <= valiantMetric){
            hdr->edge_port = minimalPort;
            hdr->stage_number = final_stage;
          } else {
            hdr->edge_port = valiantPort;
            hdr->stage_number = valiant_stage1;
          }
        } else { //no intermediate group - must go minimal
          hdr->edge_port = minimalPort;
          hdr->stage_number = final_stage;
        }
        hdr->deadlock_vc = 0;
      }
    }
  }

  int numVC() const override {
    return 2;
  }

 private:
  DragonflyPlus* dfly_;
  int rotate_g;
  int my_a;
  int my_row_;
  int my_g_;
  int up_rotater_;
  int num_leaf_switches_;
  std::vector<int> grp_rotaters_;
  int covering_;

};

//..........................................................
 class DragonflyPlusPiggyBackRouter1 : public DragonflyPlusAlltoallMinimalRouter {
  struct header : public DragonflyPlusAlltoallMinimalRouter::header {
    uint8_t stage_number : 4;
  };
 public:
  SST_ELI_REGISTER_DERIVED(
    Router,
    DragonflyPlusPiggyBackRouter1,
    "macro",
    "dragonfly_plus_piggyback1",
    SST_ELI_ELEMENT_VERSION(1,0,0),
    "router implementing piggyback1 for dragonfly+")

  static const char initial_stage = 0;
  static const char valiant_stage1 = 1;
  static const char valiant_stage2 = 2;
  static const char valiant_stage3 = 3;
  static const char final_stage = 4;

  std::string toString() const override {
    return "dragonfly+ piggyback1 router";
  }


  DragonflyPlusPiggyBackRouter1(SST::Params& params, Topology *top,
                       NetworkSwitch *netsw)
    : DragonflyPlusAlltoallMinimalRouter(params, top, netsw)
  {
    dfly_ = safe_cast(DragonflyPlus, top);
    my_row_ = my_addr_ / dfly_->numLeafSwitches();
    my_g_ = (my_addr_ % dfly_->numLeafSwitches()) / dfly_->a();
    my_a = dfly_->computeA(my_addr_);
    //covering_ = dfly_->h() / (dfly_->g() - 1);
    grp_rotaters_.resize(dfly_->g());
    for (int i=0; i < dfly_->g(); ++i){
      grp_rotaters_[i] = 0;
    }
    up_rotater_ = 0;
    rotate_r = 0;
    num_leaf_switches_ = dfly_->numLeafSwitches();

    all_link_state = dfly_->global_array();
    all_link_state_timestamp = dfly_->global_timestamp_array();
    PB_latency=params.find<double>("pb_latency", 0.0);

    rter_debug("At router constructor current time is %lf", netsw_->now().time.sec());
    rter_debug("Piggybacking latency is set to %lf seconds", PB_latency);

  }

  //~DragonflyPlusPiggyBackRouter1(){
  //  rter_debug("At router destructor current time is %lf", netsw_->now().time.sec());
  // }

  void route(Packet *pkt) override {
    //Global array to store piggyback information

    //typedef int (*mytype)[MAX_ROUTER][MAX_PORT];
    //typedef double (*mytype2)[MAX_ROUTER][MAX_PORT];


    

    SwitchId ej_addr = pkt->toaddr() / dfly_->concentration();
    auto hdr = pkt->rtrHeader<header>();
	
    int dstG = (ej_addr % num_leaf_switches_) / dfly_->a();
    std::vector<std::pair<int,int>> groupConnections;
    if(my_g_ != dstG){
    dfly_->groupWiring()->connectedToGroup(my_g_, dstG, groupConnections);
    if (groupConnections.size() == 0){
        spkt_abort_printf("Got zero group connections from %d->%d", my_g_, dstG);
      }
	}


    /*if (my_row_ == 0 && hdr->stage_number != valiant_stage3)
      hdr->deadlock_vc = 0;
    else hdr->deadlock_vc++;
    */
	

    //leaf level, source and destination  group routing	
    if (my_row_ == 0 && hdr->stage_number != valiant_stage3){

      hdr->deadlock_vc = 0;

      if (ej_addr == my_addr_){
        hdr->edge_port = pkt->toaddr() % dfly_->concentration() + dfly_->a();
      } else {
        //Intra-group routing, gotta route up
        if (my_g_ == dstG)
        {
            //rter_debug("routing up on %d", up_rotater_);
            hdr->edge_port = up_rotater_;
            up_rotater_ = (up_rotater_ + 1) % dfly_->a();
        }
        else{
            //piggyback stage
            int ct=0;
			//std::vector<std::pair<int,int>> uncongested;
            vector < int > uncongested;
            vector < int > val;
            //find the connected routers
			for(int i=0;i<groupConnections.size();i++)
			{
				int a = groupConnections[i].first;
				int r = dfly_->getUid(1,a,my_g_);
				int port_no = groupConnections[i].second;
				int port = (port_no%dfly_->h())+ dfly_->a();
				if((*all_link_state)[r][port]==1)
               {
                   ct++;
                   val.push_back(a);
               }
               else
               {
                   //uncongested.emplace_back(a,port);
				   uncongested.push_back(a);
               }
				
			}
            if(ct==groupConnections.size())
            {
                //all link are congested need to go valiant state
                //rter_debug("routing up on %d", up_rotater_);
                hdr->edge_port = up_rotater_;
                up_rotater_ = (up_rotater_ + 1) % dfly_->a();
                hdr->stage_number = valiant_stage1;
            }
            else
            {
                //randomly pick any K minimal global connected router
                int RANDOM = 2;
                vector < int >random_minimal;
                //srand(time(NULL));
                for(int k=0;k<RANDOM;k++)
                {
                    int port_index=rand()%uncongested.size();
                    random_minimal.push_back(uncongested[port_index]);
                }
                //rter_debug("..%d %d .. %d %d..",congested.size(),val.size(),random_minimal.size(),random_val.size());
                int min_random_minimal=random_minimal.at(0);
                int port1,port2;
                //calculate least locally congested random minimal port
                for(int k=1;k<RANDOM;k++)
                {
                    port1=min_random_minimal;
                    port2=random_minimal.at(k);
                    if(netsw_->queueLength(port1, all_vcs)>netsw_->queueLength(port2, all_vcs))
                    {
                        min_random_minimal=port2;
                    }

                }
				hdr->edge_port = min_random_minimal;
            }


        }
      }
    }
    else if (my_row_ == 0 && hdr->stage_number == valiant_stage3){
      //virtual channel not changed at leaf level of intermediate group
      hdr->edge_port = groupConnections[rotate_r].first;
      rotate_r = (rotate_r + 1) % groupConnections.size();
    }		
    else {
        //calculate the congestion every time it enters the global level
        int p=0;
        int total_queue_length=0;
        for(int i=0;i<dfly_->a();i++)
        {
            total_queue_length=total_queue_length+netsw_->queueLength(i+dfly_->a(),all_vcs);
        }
        float average_queue_length=total_queue_length/dfly_->a();
	for(int j=0;j<dfly_->a();j++)
        {
          double last_updated = (*all_link_state_timestamp)[my_addr_][j+dfly_->a()];
          if(netsw_->now().time.sec() >= last_updated + PB_latency){

                //copy from old, saved information
                (*all_link_state)[my_addr_][j+dfly_->a()]= (*all_link_state)[my_addr_][dfly_->maxNumPorts()+j+dfly_->a()];

                int x= netsw_->queueLength(j+dfly_->a(), all_vcs);

                if(x>2*average_queue_length)
                {
                    (*all_link_state)[my_addr_][dfly_->maxNumPorts()+j+dfly_->a()]=1; 
                }
                else
                {
                    (*all_link_state)[my_addr_][dfly_->maxNumPorts()+j+dfly_->a()]=0;
                }

                (*all_link_state_timestamp)[my_addr_][j+dfly_->a()] = netsw_->now().time.sec();
          }
        }
      //int dstG = (ej_addr % num_leaf_switches_) / dfly_->a();


      if (my_g_ == dstG){
        //go down to the eject stage
        hdr->deadlock_vc = 1;

        int dstA = ej_addr % dfly_->a();
        hdr->edge_port = dstA;
        //don't change the vc
        //rter_debug("routing down to %d", int(hdr->edge_port));
      }
      else if(hdr->stage_number == valiant_stage1)
      {
		int port = rand()%dfly_->h();
		hdr->stage_number = valiant_stage2;
		hdr->edge_port = port+dfly_->a(); 
		//rter_debug("continuing non-minimal path from %d to %d on port %d not port %d",my_g_,dstG,intermediate_groups[port_inter],port);
      }
      else if (hdr->stage_number == valiant_stage2){
            int port_no;
            std::vector<int> con; 
	    for(int i=0;i<groupConnections.size();i++)
	    {
		  if(groupConnections[i].first == my_a)
		  {
			  con.push_back(groupConnections[i].second);
		  }
	    }
	    if(con.size()==0){
		  //down route packet

                  hdr->deadlock_vc = 1;
		  int dstA = rand() % dfly_->a();
		  hdr->stage_number = valiant_stage3;
                  hdr->edge_port = dstA;
	    }
	    else{

              hdr->deadlock_vc = 1;
	      int con_index = rand()%con.size();
	      port_no = con[con_index];
	      int port = (port_no%dfly_->h())+ dfly_->a();
	      hdr->edge_port = port;
	      hdr->stage_number = final_stage;
		}
	   }
	  else if (hdr->stage_number == valiant_stage3) {

            hdr->deadlock_vc = 1;

            int port_no;
	    std::vector<int> con; 
	    for(int i=0;i<groupConnections.size();i++)
	    {
		  if(groupConnections[i].first == my_a)
		  {
			  con.push_back(groupConnections[i].second);
		  }
	    }
	    int con_index = rand()%con.size();
	    port_no = con[con_index];
	    int port = (port_no%dfly_->h())+ dfly_->a();
	    hdr->edge_port = port;
            hdr->stage_number = final_stage;
            //rter_debug("continuing non-minimal path from %d to %d on port %d ",my_g_,dstG,port);
      } else {

        int port_no;
	  std::vector<int> con; 
	  for(int i=0;i<groupConnections.size();i++)
	  {
		  if(groupConnections[i].first == my_a)
		  {
			  con.push_back(groupConnections[i].second);
		  }
	  }
	  int con_index = rand()%con.size();
	  port_no = con[con_index];
	    int port = (port_no%dfly_->h())+ dfly_->a();
	    hdr->edge_port = port;
        //hdr->deadlock_vc = 0;
     }
    }
  }

  int numVC() const override {
    return 2;
  }

 private:
  DragonflyPlus* dfly_;
  int rotate_r;
  int my_row_;
  int my_a;
  int my_g_;
  int up_rotater_;
  int num_leaf_switches_;
  std::vector<int> grp_rotaters_;
  int covering_;

  mytype all_link_state;
  mytype2 all_link_state_timestamp;
  double PB_latency;

};

//........................................................................................
class DragonflyPlusPiggyBackRouter2 : public DragonflyPlusAlltoallMinimalRouter {
  struct header : public DragonflyPlusAlltoallMinimalRouter::header {
    uint8_t stage_number : 4;
	int randomG;
  };
 public:
  SST_ELI_REGISTER_DERIVED(
    Router,
    DragonflyPlusPiggyBackRouter2,
    "macro",
    "dragonfly_plus_piggyback2",
    SST_ELI_ELEMENT_VERSION(1,0,0),
    "router implementing piggyback2 for dragonfly+")
  
  static const char initial_stage = 0;
  static const char valiant_stage1 = 1;
  static const char valiant_stage11 = 5;
  static const char valiant_stage2 = 2;
  static const char valiant_stage3 = 3;
  static const char final_stage = 4;

  std::string toString() const override {
    return "dragonfly+ piggyback2 router";
  }
DragonflyPlusPiggyBackRouter2(SST::Params& params, Topology *top,
                       NetworkSwitch *netsw)
    : DragonflyPlusAlltoallMinimalRouter(params, top, netsw)
  {
    dfly_ = safe_cast(DragonflyPlus, top);
    my_row_ = my_addr_ / dfly_->numLeafSwitches();
    my_g_ = (my_addr_ % dfly_->numLeafSwitches()) / dfly_->a();
	my_a = dfly_->computeA(my_addr_);
    //covering_ = dfly_->h() / (dfly_->g() - 1);
    grp_rotaters_.resize(dfly_->g());
    for (int i=0; i < dfly_->g(); ++i){
      grp_rotaters_[i] = 0;
    }
    up_rotater_ = 0;
	rotate_r = 0;
    num_leaf_switches_ = dfly_->numLeafSwitches();

    all_link_state = dfly_->global_array();
    all_link_state_timestamp = dfly_->global_timestamp_array();
    PB_latency=params.find<double>("pb_latency", 0.0);


  }
 void route(Packet *pkt) override {
     //Global array to store piggyback information

     mytype all_link_state = dfly_->global_array();
     mytype2 all_link_state_timestamp = dfly_->global_timestamp_array();

    SwitchId ej_addr = pkt->toaddr() / dfly_->concentration();
    auto hdr = pkt->rtrHeader<header>();
	
	int dstG = (ej_addr % num_leaf_switches_) / dfly_->a();
	std::vector<std::pair<int,int>> groupConnections;
	if(my_g_ != dstG){
	dfly_->groupWiring()->connectedToGroup(my_g_, dstG, groupConnections);
	if (groupConnections.size() == 0){
        spkt_abort_printf("Got zero group connections from %d->%d", my_g_, dstG);
      }
	}
	
	
    if (my_row_ == 0 && hdr->stage_number != valiant_stage3){
      hdr->deadlock_vc = 0;

      if (ej_addr == my_addr_){
        hdr->edge_port = pkt->toaddr() % dfly_->concentration() + dfly_->a();
      } else {
        //gotta route up
        if (my_g_ == dstG)
        {
            //rter_debug("routing up on %d", up_rotater_);
            hdr->edge_port = up_rotater_;
            up_rotater_ = (up_rotater_ + 1) % dfly_->a();
        }
        else
        {
            //piggyback stage
            int ct=0;
			//std::vector<std::pair<int,int>> uncongested;
            vector < int > uncongested;
            vector < int > val;
            //find the connected routers
			for(int i=0;i<groupConnections.size();i++)
			{
				int a = groupConnections[i].first;
				int r = dfly_->getUid(1,a,my_g_);
				int port_no = groupConnections[i].second;
				int port = (port_no%dfly_->h())+ dfly_->a();
				if((*all_link_state)[r][port]==1)
               {
                   ct++;
                   val.push_back(a);
               }
               else
               {
                   //uncongested.emplace_back(a,port);
				   uncongested.push_back(a);
               }
				
			}
	    if(ct==groupConnections.size())
            {
                //all link are congested need to go valiant state
                //rter_debug("routing up on %d", up_rotater_);
                hdr->edge_port = up_rotater_;
                up_rotater_ = (up_rotater_ + 1) % dfly_->a();
                hdr->stage_number = valiant_stage1;

            }
            else{
            //randomly pick any K minimal global connected router
            int RANDOM = 2;
            vector < int >random_minimal;
            //srand(time(NULL));
            for(int k=0;k<RANDOM;k++)
            {
                int port_index=rand()%uncongested.size();
                random_minimal.push_back(uncongested[port_index]);
            }
            //rter_debug("..%d %d .. %d %d..",congested.size(),val.size(),random_minimal.size(),random_val.size());
            int min_random_minimal=random_minimal.at(0);
            int port1,port2;
            //calculate least locally congested random minimal port
            for(int k=1;k<RANDOM;k++)
            {
                port1=min_random_minimal;
                port2=random_minimal.at(k);
                if(netsw_->queueLength(port1, all_vcs)>netsw_->queueLength(port2, all_vcs))
                {
                    min_random_minimal=port2;
                }

            }
			//random group selection
            int randG;
            do{
             randG=rand()%dfly_->g();
			 hdr->randomG = randG;
            }while(randG==dstG || randG==my_g_);

            int ct1=0;
            vector < int > uncongested1;
            vector < int > val1;
			std::vector<std::pair<int,int>> groupConnections1;
			dfly_->groupWiring()->connectedToGroup(my_g_, randG, groupConnections1);
            //find the connected routers
            for(int i=0;i<groupConnections1.size();i++)
			{
				int a = groupConnections1[i].first;
				int r = dfly_->getUid(1,a,my_g_);
				int port_no = groupConnections1[i].second;
				int port = (port_no%dfly_->h())+ dfly_->a();
				if((*all_link_state)[r][port]==1)
               {
                   ct1++;
                   val1.push_back(a);
               }
               else
               {
                   //uncongested.emplace_back(a,port);
				   uncongested1.push_back(a);
               }
				
			}
            if(ct1==groupConnections1.size())
            {
                hdr->edge_port = min_random_minimal;
            }
            else{
            //randomly pick any K minimal global connected router
            int RANDOM1 = 2;
            vector < int >random_minimal1;
            //srand(time(NULL));
            for(int k=0;k<RANDOM1;k++)
            {
                int port_index=rand()%uncongested1.size();
                random_minimal1.push_back(uncongested1[port_index]);
            }
            //rter_debug("..%d %d .. %d %d..",congested.size(),val.size(),random_minimal.size(),random_val.size());
            int min_random_minimal1=random_minimal1.at(0);
            int port11,port21;
            //calculate least locally congested random minimal port
            for(int k=1;k<RANDOM1;k++)
            {
                port11=min_random_minimal1;
                port21=random_minimal1.at(k);
                if(netsw_->queueLength(port11, all_vcs)>netsw_->queueLength(port21, all_vcs))
                {
                    min_random_minimal1=port21;
                }

            }
            //UGAL decision
            if(netsw_->queueLength(min_random_minimal, all_vcs)<=2*netsw_->queueLength(min_random_minimal1, all_vcs))
            {
                hdr->edge_port = min_random_minimal;
            }
            else
            {
                hdr->edge_port = min_random_minimal1;
                hdr->stage_number = valiant_stage11;
            }
            }
          }
        }
      }
    } 
	else if (my_row_ == 0 && hdr->stage_number == valiant_stage3){
		hdr->edge_port = groupConnections[rotate_r].first;
        rotate_r = (rotate_r + 1) % groupConnections.size();
	}
	else {
       //calculate the congestion every time it enters the global level
        int s=0;
        int total_queue_length=0;
        for(int i=0;i<dfly_->a();i++)
        {
            total_queue_length=total_queue_length+netsw_->queueLength(i+dfly_->a(),all_vcs);
        }
        float average_queue_length=total_queue_length/dfly_->a();
                for(int j=0;j<dfly_->a();j++)
        {
                int x= netsw_->queueLength(j+dfly_->a(), all_vcs);
                if(x>2*average_queue_length)
                {
                    (*all_link_state)[my_addr_][j+dfly_->a()]=1;
                }
                else
                {
                    (*all_link_state)[my_addr_][j+dfly_->a()]=0;
                }
        }
		//int dstG = (ej_addr % num_leaf_switches_) / dfly_->a();
      if (my_g_ == dstG){
        //go down to the eject stage
        int dstA = ej_addr % dfly_->a();
        hdr->edge_port = dstA;
        //don't change the vc
        //rter_debug("routing down to %d", int(hdr->edge_port));
      }else if (hdr->stage_number == valiant_stage11) {

         hdr->deadlock_vc = 1;

	int port;
        std::vector<std::pair<int,int>> groupConnections1;
		dfly_->groupWiring()->connectedToGroup(my_g_, hdr->randomG, groupConnections1);
		for(int i=0;i<groupConnections1.size();i++)
	    {
		  if(groupConnections1[i].first == my_a)
		  {
			  port = groupConnections1[i].second;
		  }
	    }
		hdr->stage_number = valiant_stage2;
		hdr->edge_port = (port%dfly_->h())+ dfly_->a();  
      }
      else if (hdr->stage_number == valiant_stage1) {
        
	    int port = rand()%dfly_->h();
            hdr->stage_number = valiant_stage2;
            hdr->edge_port = port+dfly_->a();   
      }
      else if (hdr->stage_number == valiant_stage2) {
         hdr->deadlock_vc = 1;
	 int port_no;
         std::vector<int> con; 
	    for(int i=0;i<groupConnections.size();i++)
	    {
		  if(groupConnections[i].first == my_a)
		  {
			  con.push_back(groupConnections[i].second);
		  }
	    }
		if(con.size()==0){
		  //down route packet
		  int dstA = rand() % dfly_->a();
		  hdr->stage_number = valiant_stage3;
          hdr->edge_port = dstA;
		}
		else{
		int con_index = rand()%con.size();
	        port_no = con[con_index];
	        int port = (port_no%dfly_->h())+ dfly_->a();
	        hdr->edge_port = port;
		hdr->stage_number = final_stage;
		}
      }
	  else if (hdr->stage_number == valiant_stage3) {

        int port_no;
	  std::vector<int> con; 
	  for(int i=0;i<groupConnections.size();i++)
	  {
		  if(groupConnections[i].first == my_a)
		  {
			  con.push_back(groupConnections[i].second);
		  }
	  }
	  int con_index = rand()%con.size();
	  port_no = con[con_index];
	    int port = (port_no%dfly_->h())+ dfly_->a();
	    hdr->edge_port = port;
		hdr->stage_number = final_stage;
      }
	  else {
        int port_no;
	  std::vector<int> con; 
	  for(int i=0;i<groupConnections.size();i++)
	  {
		  if(groupConnections[i].first == my_a)
		  {
			  con.push_back(groupConnections[i].second);
		  }
	  }
	  int con_index = rand()%con.size();
	  port_no = con[con_index];
	    int port = (port_no%dfly_->h())+ dfly_->a();
	    hdr->edge_port = port;
     }
    }
  }

  int numVC() const override {
    return 2;
  }

 private:
  DragonflyPlus* dfly_;
  int rotate_r;
  int my_row_;
  int my_a;
  int my_g_;
  int up_rotater_;
  int num_leaf_switches_;
  std::vector<int> grp_rotaters_;
  int covering_;

  mytype all_link_state;
  mytype2 all_link_state_timestamp;
  double PB_latency;


};

}
}
