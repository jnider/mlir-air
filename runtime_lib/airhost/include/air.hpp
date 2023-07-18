//===- air.hpp -------------------------------------------------*- C++ -*-===//
//
// Copyright (C) 2022, Advanced Micro Devices, Inc.
// SPDX-License-Identifier: MIT
//
//===----------------------------------------------------------------------===//

#ifndef AIR_HPP
#define AIR_HPP

#include "air_host.h"

#include <stdint.h>
#include <vector>

template<typename T>
inline void air_write_pkt(hsa_queue_t *q, uint32_t packet_id, T *pkt) {
  reinterpret_cast<T*>(q->base_address)[packet_id] = *pkt;
}

inline hsa_status_t air_get_agents(std::vector<hsa_agent_t> &agents) {
  return hsa_iterate_agents(find_aie, (void *)&agents);
}

inline hsa_status_t air_get_agents(std::vector<air_agent_t> &agents) {
  return air_iterate_agents(
      [](air_agent_t a, void *d) {
        auto *v = static_cast<std::vector<air_agent_t> *>(d);
        v->push_back(a);
        return HSA_STATUS_SUCCESS;
      },
      (void *)&agents);
}



uint64_t air_wait_all(std::vector<uint64_t> &signals);

#endif
