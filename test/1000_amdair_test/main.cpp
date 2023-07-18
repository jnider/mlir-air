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

//#include "air.hpp"

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

hsa_status_t find_aie(hsa_agent_t agent, void *data)
{
  hsa_status_t status(HSA_STATUS_SUCCESS);
  hsa_device_type_t device_type;
  std::vector<hsa_agent_t> *aie_agents = nullptr;

  printf("[EDDIE DEBUG] in find_aie\n");

  if (!data) {
    status = HSA_STATUS_ERROR_INVALID_ARGUMENT;
    std::cout << "find_aie: INVALID ARGUMENT" << std::endl;
    return status;
  }

  aie_agents = static_cast<std::vector<hsa_agent_t>*>(data);
  printf("[EDDIE DEBUG] Calling get_info\n");
  status = hsa_agent_get_info(agent, HSA_AGENT_INFO_DEVICE, &device_type);
  printf("[EDDIE DEBUG] Done calling agent info. Device type is 0x%x\n", device_type);
  

  if (status != HSA_STATUS_SUCCESS) {
    return status;
  }

  if (device_type == HSA_DEVICE_TYPE_AIE) {
    aie_agents->push_back(agent);
  }

  return status;
}

int main(int argc, char *argv[]) {

  std::vector<hsa_agent_t> aie_agents;
  hsa_queue_t *aie_queue;
  uint32_t aie_max_queue_size(0);

  hsa_check_status("init", hsa_init());
  hsa_check_status("iterate_agents", hsa_iterate_agents(find_aie, &aie_agents));

  std::cout << "Found " << aie_agents.size() << " agents" << std::endl;

  if(aie_agents.size() == 0) {
    printf("[ERROR] Did not find any agents. Returning\n");
    return -1;
  }

  hsa_check_status("agent_get_info",
                   hsa_agent_get_info(aie_agents[0],
        	                      HSA_AGENT_INFO_QUEUE_MAX_SIZE,
        			      &aie_max_queue_size));

  std::cout << "Max AIE queue size: " << aie_max_queue_size << std::endl;
  hsa_check_status("queue_create",
                   hsa_queue_create(aie_agents[0], aie_max_queue_size,
                                    HSA_QUEUE_TYPE_SINGLE, nullptr, nullptr, 0,
                                    0, &aie_queue));

  hsa_agent_dispatch_packet_t aql_pkt;
  std::memset(&aql_pkt, 0, sizeof(hsa_agent_dispatch_packet_t));
  aql_pkt.arg[0] = 0xbaadbeefdeadbeef;
  aql_pkt.header |= (HSA_PACKET_TYPE_AGENT_DISPATCH << HSA_PACKET_HEADER_TYPE);
  *reinterpret_cast<hsa_agent_dispatch_packet_t*>(aie_queue->base_address) = aql_pkt;

  std::cout << "writing doorbell" << std::endl;
  hsa_signal_store_relaxed(aie_queue->doorbell_signal, 0xacdc);

  auto rd_idx(hsa_queue_load_read_index_scacquire(aie_queue));
  std::cout << "Read index: " << rd_idx << std::endl;

  hsa_check_status("shut_duwn", hsa_shut_down());
  return 0;

}
