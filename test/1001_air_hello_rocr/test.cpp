//===- test.cpp -------------------------------------------------*- C++ -*-===//
//
// Copyright (C) 2021-2022, Xilinx Inc.
// Copyright (C) 2022, Advanced Micro Devices, Inc.
// SPDX-License-Identifier: MIT
//
//===----------------------------------------------------------------------===//

#include <assert.h>
#include <cstdio>
#include <fcntl.h>
#include <iostream>
#include <stdlib.h>
#include <sys/mman.h>
#include <string.h>
#include <vector>
#include <unistd.h>
#include <cstring>
#include <string>

#include "air.hpp"

#include "hsa/hsa.h"
#include "hsa/hsa_ext_amd.h"

void hsa_check_status(const std::string func_name, hsa_status_t status)
{
  if (status != HSA_STATUS_SUCCESS) {
    const char *status_string(new char[1024]);
    hsa_status_string(status, &status_string);
    std::cout << func_name << " failed: " << status_string << std::endl;
    delete[] status_string;
  } else {
    std::cout << func_name << " success" << std::endl;
  }
}

int main(int argc, char *argv[]) {

  std::vector<hsa_agent_t> aie_agents;
  std::vector<hsa_queue_t *> queues;
  uint32_t aie_max_queue_size(0);

  hsa_check_status("init", hsa_init());
  hsa_check_status("iterate_agents", hsa_iterate_agents(find_aie, &aie_agents));

  std::cout << "Found " << aie_agents.size() << " agents" << std::endl;

  printf("EXITING EARLY\n");
  return -1;

  hsa_check_status("agent_get_info",
                   hsa_agent_get_info(aie_agents[0],
        	                      HSA_AGENT_INFO_QUEUE_MAX_SIZE,
        			      &aie_max_queue_size));

  std::cout << "Max AIE queue size: " << aie_max_queue_size << std::endl;

  // Creating all of the queues
  for(int i = 0; i < aie_max_queue_size; i++) {
    hsa_check_status("queue_create",
                     hsa_queue_create(aie_agents[0], aie_max_queue_size,
                                      HSA_QUEUE_TYPE_SINGLE, nullptr, nullptr, 0,
                                      0, &queues[i]));
  }

  // Sending a print packet to each queue
  uint64_t data = -1;
  char vend[8];
  for (auto q : queues) {

    std::cout << "Asking the agent to print hello world" << std::endl;

    // Getting our slot in the queue
    uint64_t wr_idx = hsa_queue_add_write_index_relaxed(q, 1);
    uint64_t packet_id = wr_idx % q->size;

    // Creating, initializing, and dispatching our packet
    hsa_agent_dispatch_packet_t aql_pkt;
    air_packet_hello(&aql_pkt, 0xCAFEED1E);
    air_queue_dispatch_and_wait(&aie_agents[0], q, wr_idx, &aql_pkt);
  
  }

  /*hsa_agent_dispatch_packet_t aql_pkt;
  std::memset(&aql_pkt, 0, sizeof(hsa_agent_dispatch_packet_t));
  aql_pkt.arg[0] = 0xbaadbeefdeadbeef;
  aql_pkt.header |= (HSA_PACKET_TYPE_AGENT_DISPATCH << HSA_PACKET_HEADER_TYPE);
  *reinterpret_cast<hsa_agent_dispatch_packet_t*>(aie_queue->base_address) = aql_pkt;

  std::cout << "writing doorbell" << std::endl;
  hsa_signal_store_relaxed(aie_queue->doorbell_signal, 0xacdc);

  auto rd_idx(hsa_queue_load_read_index_scacquire(aie_queue));
  std::cout << "Read index: " << rd_idx << std::endl;*/

  hsa_check_status("shut_duwn", hsa_shut_down());
  return 0;

/*
  hsa_status_t init_status = air_init();

  if (init_status != HSA_STATUS_SUCCESS) {
    std::cout << "air_init() failed. Exiting" << std::endl;
    return -1;
  }

  std::vector<air_agent_t> agents;
  auto ret = air_get_agents(agents);
  assert(ret == 0 && "failed to get agents!");

  if (agents.empty()) {
    std::cout << "No agents found. Exiting." << std::endl;
    return -1;
  }

  std::vector<queue_t *> queues;
  for (auto agent : agents) {
    // create the queue
    queue_t *q = nullptr;
    ret = air_queue_create(MB_QUEUE_SIZE, HSA_QUEUE_TYPE_SINGLE, &q,
                           agent.handle);
    if(ret) {
      printf("Failed to create queue. Not adding to list of queues\n");
    }
    else {
      queues.push_back(q);
    }
  }

  assert(queues.size() > 0 && "No queues were sucesfully created!");
 
  uint64_t data = -1;
  char vend[8];
  for (auto q : queues) {
    std::cout << std::endl << "Requesting attribute: AIR_AGENT_INFO_CONTROLLER_ID... ";
    air_get_agent_info(q, AIR_AGENT_INFO_CONTROLLER_ID, &data);
    std::cout << "Agent ID is: " << data << std::endl;

    std::cout << "Requesting attribute: AIR_AGENT_INFO_VENDOR_NAME... ";
    air_get_agent_info(q, AIR_AGENT_INFO_VENDOR_NAME, vend);
    std::cout << "Vendor is: " << vend << std::endl;

    std::cout << "Requesting attribute: AIR_AGENT_INFO_L2_MEM_SIZE... ";
    air_get_agent_info(q, AIR_AGENT_INFO_L2_MEM_SIZE, &data);
    std::cout << "L2 size is: " << std::dec << data << "B" << std::endl;
  }

  std::cout << std::endl << "PASS!" << std::endl;
  return 0;
*/
}
