#!/bin/bash

declare -a bench
declare -a prefetch
declare -a btb

PATH_TO_CHAMPSIM=/ChampSim-master-BTBX

bench[0]=client_001
bench[1]=client_002
bench[2]=client_003
bench[3]=client_004
bench[4]=client_005
bench[5]=client_006
bench[6]=client_007
bench[7]=client_008
bench[8]=server_001
bench[9]=server_002
bench[10]=server_003
bench[11]=server_004
bench[12]=server_009
bench[13]=server_010
bench[14]=server_011
bench[15]=server_012
bench[16]=server_013
bench[17]=server_014
bench[18]=server_015
bench[19]=server_016
bench[20]=server_017
bench[21]=server_018
bench[22]=server_019
bench[23]=server_020
bench[24]=server_021
bench[25]=server_022
bench[26]=server_023
bench[27]=server_024
bench[28]=server_025
bench[29]=server_026
bench[30]=server_027
bench[31]=server_028
bench[32]=server_029
bench[33]=server_030
bench[34]=server_031
bench[35]=server_032
bench[36]=server_033
bench[37]=server_034
bench[38]=server_035
bench[39]=server_036
bench[40]=server_037
bench[41]=server_038
bench[42]=server_039

prefetch[0]=no
prefetch[1]=fdip

btb[0]=convBTB
btb[1]=pdede
btb[2]=BTBX


for ((j=1;j<2;j=j+1)); do
    for ((i=0;i<43;i=i+1)); do
	for ((k=2;k<3;k=k+1)); do
	    script_name="${bench[i]}_${prefetch[j]}_${btb[k]}.sh"
	    echo "#!/bin/bash" > $script_name
	    echo "cd ${PATH_TO_CHAMPSIM}" >> $script_name
	    echo "./run_champsim.sh hashed_perceptron-${btb[k]}-${prefetch[j]}-next_line-spp_dev-no-lru-1core 50 50 ${bench[i]}.champsimtrace.xz" >> $script_name
	done
    done
done 