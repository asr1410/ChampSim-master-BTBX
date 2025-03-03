#!/bin/bash

directory="./results/"

# Arrays to store values
declare -a names=()
declare -a accuracies=()
declare -a mpkis=()
declare -a robs=()
declare -a ipcs=()

# Process each file
for file in "$directory"/*.txt; do
    if [ -f "$file" ]; then
        branch_line=$(grep "CPU 0 Branch Prediction Accuracy" "$file")
        ipc_line=$(grep "^XXX cumulative-IPC" "$file")
        
        if [ ! -z "$branch_line" ] && [ ! -z "$ipc_line" ]; then
            # Extract IPC value (third field)
            ipc=$(echo "$ipc_line" | awk '{print $3}')
            
            # Store other metrics as before
            accuracy=$(echo "$branch_line" | awk '{print $6}' | sed 's/%//')
            mpki=$(echo "$branch_line" | awk '{print $8}')
            rob=$(echo "$branch_line" | awk '{for(i=NF;i>=1;i--) if($i+0==$i) {print $i; exit}}')
            
            # Store values in arrays
            accuracies+=("$accuracy")
            mpkis+=("$mpki")
            robs+=("$rob")
            ipcs+=("$ipc")
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