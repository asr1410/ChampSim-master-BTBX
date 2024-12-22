#!/bin/bash
cd /ChampSim-master-BTBX

# run from 5 to 51 with step 5
for i in {5..51..5}
do
  ./run_champsim.sh hashed_perceptron-BTBX-fdip-next_line-spp_dev-no-lru-1core $i $i server_001.champsimtrace.xz
done