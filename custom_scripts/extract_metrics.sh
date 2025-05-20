#!/bin/bash

directory="./results/fullnovel/"

# Arrays to store values
declare -a names=()
declare -a accuracies=()
declare -a mpkis=()
declare -a robs=()
declare -a ipcs=()
declare -a cycles=()
declare -a btb_reads=()  # Array for BTB reads
declare -a btb_writes=()  # Array for BTB writes

# Process each file
for file in "$directory"/*.txt; do
    if [ -f "$file" ]; then
        branch_line=$(grep "CPU 0 Branch Prediction Accuracy" "$file")
        ipc_line=$(grep "^XXX cumulative-IPC" "$file")
        cycle_line=$(grep "^XXX cycles" "$file")
        btb_read_line=$(grep "^XXX BTB_reads:" "$file")  # Line containing BTB reads
        btb_write_line=$(grep "^XXX BTB_writes:" "$file")  # Line containing BTB writes
        
        if [ ! -z "$branch_line" ] && [ ! -z "$ipc_line" ] && [ ! -z "$cycle_line" ]; then
            # Extract IPC value (third field)
            ipc=$(echo "$ipc_line" | awk '{print $3}')
            
            # Extract cycle count (third field)
            cycle=$(echo "$cycle_line" | awk '{print $3}')
            
            # Extract BTB read and write counts (second field - fix here)
            btb_read=$(echo "$btb_read_line" | awk '{print $3}')  # Changed from $2 to $3
            btb_write=$(echo "$btb_write_line" | awk '{print $3}')  # Changed from $2 to $3
            
            # Store other metrics as before
            accuracy=$(echo "$branch_line" | awk '{print $6}' | sed 's/%//')
            mpki=$(echo "$branch_line" | awk '{print $8}')
            rob=$(echo "$branch_line" | awk '{for(i=NF;i>=1;i--) if($i+0==$i) {print $i; exit}}')
            
            # Store values in arrays
            accuracies+=("$accuracy")
            mpkis+=("$mpki")
            robs+=("$rob")
            ipcs+=("$ipc")
            cycles+=("$cycle")
            btb_reads+=("$btb_read")  # Store BTB reads
            btb_writes+=("$btb_write")  # Store BTB writes
        fi
    fi
done

# Print each metric array on a single line
# Accuracies
echo -n "["
printf "%s" "${accuracies[0]}"
for i in "${!accuracies[@]}"; do
    [ $i -eq 0 ] && continue
    printf ", %s" "${accuracies[$i]}"
done
echo "]"

# MPKIs
echo -n "["
printf "%s" "${mpkis[0]}"
for i in "${!mpkis[@]}"; do
    [ $i -eq 0 ] && continue
    printf ", %s" "${mpkis[$i]}"
done
echo "]"

# ROB Occupancies
echo -n "["
printf "%s" "${robs[0]}"
for i in "${!robs[@]}"; do
    [ $i -eq 0 ] && continue
    printf ", %s" "${robs[$i]}"
done
echo "]"

# IPCs
echo -n "["
printf "%s" "${ipcs[0]}"
for i in "${!ipcs[@]}"; do
    [ $i -eq 0 ] && continue
    printf ", %s" "${ipcs[$i]}"
done
echo "]"

# Cycles
echo -n "["
printf "%s" "${cycles[0]}"
for i in "${!cycles[@]}"; do
    [ $i -eq 0 ] && continue
    printf ", %s" "${cycles[$i]}"
done
echo "]"

# BTB Reads
echo -n "["
printf "%s" "${btb_reads[0]}"
for i in "${!btb_reads[@]}"; do
    [ $i -eq 0 ] && continue
    printf ", %s" "${btb_reads[$i]}"
done
echo "]"

# BTB Writes
echo -n "["
printf "%s" "${btb_writes[0]}"
for i in "${!btb_writes[@]}"; do
    [ $i -eq 0 ] && continue
    printf ", %s" "${btb_writes[$i]}"
done
echo "]"