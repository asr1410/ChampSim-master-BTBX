#include "ooo_cpu.h"

#define BASIC_BTB_SETS 4096
#define BASIC_BTB_WAYS 4
#define BASIC_BTB_INDIRECT_SIZE 4096
#define BASIC_BTB_RAS_SIZE 64
#define BASIC_BTB_CALL_INSTR_SIZE_TRACKERS 1024

struct BTBEntry {
  uint64_t tag;
  uint64_t target_ip;
  uint8_t branch_type;
  uint64_t lru;
  uint8_t confidence;
};

struct BTB {
  std::vector<std::vector<BTBEntry>> theBTB;
  uint32_t numSets;
  uint32_t assoc;
  uint64_t indexMask;
  uint32_t numIndexBits;

  BTB() {}

  BTB(int32_t Sets, int32_t Assoc)
      : numSets(Sets), assoc(Assoc) {
    assert(((Sets - 1) & (Sets)) == 0);
    theBTB.resize(Sets);
    indexMask = Sets - 1;
    numIndexBits = (uint32_t)log2((double)Sets);
  }

  void init_btb(int32_t Sets, int32_t Assoc) {
    numSets = Sets;
    assoc = Assoc;
    assert(((Sets - 1) & (Sets)) == 0);
    theBTB.resize(Sets);
    indexMask = Sets - 1;
    numIndexBits = (uint32_t)log2((double)Sets);
  }

  int32_t index(uint64_t ip) {
    return ((ip >> 2) & indexMask);
  }

  uint64_t get_tag(uint64_t ip) {
    uint64_t addr = ip;
    addr = addr >> 2;
    addr = addr >> numIndexBits;
    uint64_t tag = addr & 0xFF;
    addr = addr >> 8;
    int tagMSBs = 0;
    for (int i = 0; i < 8; i++) {
      tagMSBs = tagMSBs ^ (addr & 0xFF);
      addr = addr >> 8;
    }
    tag = tag | (tagMSBs << 8);
    return tag;
  }

  BTBEntry *get_BTBentry(uint64_t ip) {
    BTBEntry *entry = NULL;
    int idx = index(ip);
    uint64_t tag = get_tag(ip);
    for (uint32_t i = 0; i < theBTB[idx].size(); i++) {
      if (theBTB[idx][i].tag == tag) {
        return &(theBTB[idx][i]);
      }
    }
    return entry;
  }

  void update_BTB(uint64_t ip, uint8_t b_type, uint64_t target, uint8_t taken, uint64_t lru_counter) {
    int idx = index(ip);
    uint64_t tag = get_tag(ip);
    int way = -1;
    for (uint32_t i = 0; i < theBTB[idx].size(); i++) {
      if (theBTB[idx][i].tag == tag) {
        way = i;
        break;
      }
    }
    if (way == -1) {
      if ((target != 0) && taken) {
        BTBEntry entry;
        entry.tag = tag;
        entry.branch_type = b_type;
        entry.target_ip = target;
        entry.lru = lru_counter;
        entry.confidence = 1;
        
        if (theBTB[idx].size() >= assoc) {
          theBTB[idx].erase(theBTB[idx].begin());
        }
        theBTB[idx].push_back(entry);
      }
    } else {
      BTBEntry entry = theBTB[idx][way];
      
      if (entry.target_ip == target) {
        if (entry.confidence < 3) entry.confidence++;
      } else {
        if (entry.confidence > 0) entry.confidence--;
      }

      if(entry.confidence < 2) {
        entry.branch_type = b_type;
        entry.target_ip = target;
        entry.lru = lru_counter;
      }
      theBTB[idx].erase(theBTB[idx].begin() + way);
      theBTB[idx].push_back(entry);
    }
  }

  uint64_t get_lru_value(uint64_t ip) {
    int idx = index(ip);
    uint64_t lru_value;
    if (theBTB[idx].size() < assoc) {
      lru_value = 0;
    } else {
      lru_value = theBTB[idx][0].lru;
      for (uint32_t i = 1; i < theBTB[idx].size(); i++) {
        if (theBTB[idx][i].lru < lru_value) {
          assert(0);
        }
      }
    }
    return lru_value;
  }
};

#define NUM_BTB_PARTITIONS 9
BTB btb_partition[NUM_BTB_PARTITIONS];

uint64_t basic_btb_lru_counter[NUM_CPUS];
uint64_t basic_btb_indirect[NUM_CPUS][BASIC_BTB_INDIRECT_SIZE];
uint64_t basic_btb_conditional_history[NUM_CPUS];
uint64_t basic_btb_ras[NUM_CPUS][BASIC_BTB_RAS_SIZE];
int basic_btb_ras_index[NUM_CPUS];
uint64_t basic_btb_call_instr_sizes[NUM_CPUS][BASIC_BTB_CALL_INSTR_SIZE_TRACKERS];

uint64_t basic_btb_abs_addr_dist(uint64_t addr1, uint64_t addr2) {
  if (addr1 > addr2) {
    return addr1 - addr2;
  }
  return addr2 - addr1;
}

void push_basic_btb_ras(uint8_t cpu, uint64_t ip) {
  basic_btb_ras_index[cpu]++;
  if (basic_btb_ras_index[cpu] == BASIC_BTB_RAS_SIZE) {
    basic_btb_ras_index[cpu] = 0;
  }
  basic_btb_ras[cpu][basic_btb_ras_index[cpu]] = ip;
}

uint64_t peek_basic_btb_ras(uint8_t cpu) {
  return basic_btb_ras[cpu][basic_btb_ras_index[cpu]];
}

uint64_t pop_basic_btb_ras(uint8_t cpu) {
  uint64_t target = basic_btb_ras[cpu][basic_btb_ras_index[cpu]];
  basic_btb_ras[cpu][basic_btb_ras_index[cpu]] = 0;
  basic_btb_ras_index[cpu]--;
  if (basic_btb_ras_index[cpu] == -1) {
    basic_btb_ras_index[cpu] += BASIC_BTB_RAS_SIZE;
  }
  return target;
}

uint64_t basic_btb_call_size_tracker_hash(uint64_t ip) {
  return (ip & (BASIC_BTB_CALL_INSTR_SIZE_TRACKERS - 1));
}

uint64_t basic_btb_get_call_size(uint8_t cpu, uint64_t ip) {
  uint64_t size = basic_btb_call_instr_sizes[cpu][basic_btb_call_size_tracker_hash(ip)];
  return size;
}

int convert_offsetBits_to_partitionID(int num_bits) {
  if (num_bits == 0) {
    return 0;
  } else if (num_bits <= 4) {
    return 1;
  } else if (num_bits <= 5) {
    return 2;
  } else if (num_bits <= 7) {
    return 3;
  } else if (num_bits <= 9) {
    return 4;
  } else if (num_bits <= 11) {
    return 5;
  } else if (num_bits <= 19) {
    return 6;
  } else if (num_bits <= 25) {
    return 7;
  } else {
    return 8;
  }
  assert(0);
}

int get_lru_partition(int start_partitionID, uint64_t ip) {
  int lru_partition = start_partitionID;
  uint64_t lru_value = btb_partition[start_partitionID].get_lru_value(ip);
  for (int i = start_partitionID + 1; i < NUM_BTB_PARTITIONS; i++) {
    uint64_t partition_lru_value = btb_partition[i].get_lru_value(ip);
    if (partition_lru_value < lru_value) {
      lru_partition = i;
      lru_value = partition_lru_value;
    }
  }
  return lru_partition;
}

void O3_CPU::initialize_btb() {
  std::cout << "Basic BTB sets: " << BASIC_BTB_SETS
            << " ways: " << BASIC_BTB_WAYS
            << " indirect buffer size: " << BASIC_BTB_INDIRECT_SIZE
            << " RAS size: " << BASIC_BTB_RAS_SIZE << std::endl;

  for (uint32_t i = 0; i < BASIC_BTB_RAS_SIZE; i++) {
    basic_btb_ras[cpu][i] = 0;
  }
  basic_btb_ras_index[cpu] = 0;
  for (uint32_t i = 0; i < BASIC_BTB_CALL_INSTR_SIZE_TRACKERS; i++) {
    basic_btb_call_instr_sizes[cpu][i] = 4;
  }

  basic_btb_lru_counter[cpu] = 0;

  btb_partition[0].init_btb(512, 1);
  btb_partition[1].init_btb(512, 1);
  btb_partition[2].init_btb(512, 1);
  btb_partition[3].init_btb(512, 1);
  btb_partition[4].init_btb(512, 1);
  btb_partition[5].init_btb(512, 1);
  btb_partition[6].init_btb(512, 1);
  btb_partition[7].init_btb(512, 1);
  btb_partition[8].init_btb(64, 1);
}

BTB_outcome O3_CPU::btb_prediction(uint64_t ip, uint8_t branch_type) {
  BTBEntry *btb_entry;
  for (int i = 0; i < NUM_BTB_PARTITIONS; i++) {
    btb_entry = btb_partition[i].get_BTBentry(ip);
    if (btb_entry) {
      break;
    }
  }
  if (btb_entry == NULL) {
    if (branch_type == BRANCH_DIRECT_CALL || branch_type == BRANCH_INDIRECT_CALL) {
      push_basic_btb_ras(cpu, ip);
    }
    BTB_outcome outcome = {0, BRANCH_CONDITIONAL, 2};
    return outcome;
  }

  branch_type = NOT_BRANCH;
  branch_type = btb_entry->branch_type;

  if ((branch_type == BRANCH_DIRECT_CALL) || (branch_type == BRANCH_INDIRECT_CALL)) {
    push_basic_btb_ras(cpu, ip);
  }

  if (branch_type == BRANCH_RETURN) {
    uint64_t target = peek_basic_btb_ras(cpu);
    target += basic_btb_get_call_size(cpu, target);
    BTB_outcome outcome = {target, BRANCH_RETURN, 0};
    return outcome;
  } else {
    BTB_outcome outcome = {btb_entry->target_ip, branch_type, 0};
    return outcome;
  }

  assert(0);
}

void O3_CPU::update_btb(uint64_t ip, uint64_t branch_target, uint8_t taken, uint8_t branch_type) {
  if (branch_type == BRANCH_RETURN) {
    uint64_t call_ip = pop_basic_btb_ras(cpu);
    uint64_t estimated_call_instr_size = basic_btb_abs_addr_dist(call_ip, branch_target);
    if (estimated_call_instr_size <= 10) {
      basic_btb_call_instr_sizes[cpu][basic_btb_call_size_tracker_hash(call_ip)] = estimated_call_instr_size;
    }
  }

  if (taken == false) return;

  BTBEntry *btb_entry;
  int partitionID = -1;
  for (int i = 0; i < NUM_BTB_PARTITIONS; i++) {
    btb_entry = btb_partition[i].get_BTBentry(ip);
    if (btb_entry) {
      partitionID = i;
      break;
    }
  }

  if (btb_entry == NULL) {
    BTB_writes++;
    int num_bits;
    if (branch_type == BRANCH_RETURN) {
      num_bits = 0;
    } else {
      uint64_t diff_bits = (branch_target >> 2) ^ (ip >> 2);
      num_bits = 0;
      while (diff_bits != 0) {
        diff_bits = diff_bits >> 1;
        num_bits++;
      }
    }
    assert(num_bits >= 0 && num_bits < 66);
    int smallest_offset_partition_id = convert_offsetBits_to_partitionID(num_bits);
    int partition = get_lru_partition(smallest_offset_partition_id, ip);
    assert(partition < NUM_BTB_PARTITIONS);
    btb_partition[partition].update_BTB(ip, branch_type, branch_target, taken, basic_btb_lru_counter[cpu]);
    basic_btb_lru_counter[cpu]++;
  } else {
    assert(partitionID != -1);
    btb_partition[partitionID].update_BTB(ip, branch_type, branch_target, taken, basic_btb_lru_counter[cpu]);
    basic_btb_lru_counter[cpu]++;
  }
}