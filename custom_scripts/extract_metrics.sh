#!/bin/bash

# Directory containing the trace files
# directory="./results/"
directory="./results_50M/"

# Arrays to store values
declare -a names=()
declare -a accuracies=()
declare -a mpkis=()
declare -a robs=()

# Process each file
for file in "$directory"/*.txt; do
    if [ -f "$file" ]; then
        line=$(grep "CPU 0 Branch Prediction Accuracy" "$file")
        
        if [ ! -z "$line" ]; then
            # Store filename
            filename=$(basename "$file" | cut -d'.' -f1)
            names+=("$filename")
            
            # Extract accuracy (remove % sign)
            accuracy=$(echo "$line" | awk '{print $6}' | sed 's/%//')
            
            # Extract MPKI
            mpki=$(echo "$line" | awk '{print $8}')
            
            # Extract ROB value (last number in the line)
            rob=$(echo "$line" | awk '{for(i=NF;i>=1;i--) if($i+0==$i) {print $i; exit}}')
            
            # Store values in arrays
            accuracies+=("$accuracy")
            mpkis+=("$mpki")
            robs+=("$rob")
        fi
    fi
done

# Print Accuracies
echo -n "["
for i in "${!accuracies[@]}"; do
    if [ $i -eq 0 ]; then
        printf "%s" "${accuracies[$i]}"
    else
        printf ", %s" "${accuracies[$i]}"
    fi
done
echo "]"

# Print MPKIs
echo -n "["
for i in "${!mpkis[@]}"; do
    if [ $i -eq 0 ]; then
        printf "%s" "${mpkis[$i]}"
    else
        printf ", %s" "${mpkis[$i]}"
    fi
done
echo "]"

# Print ROB Occupancies 
echo -n "["
for i in "${!robs[@]}"; do
    if [ $i -eq 0 ]; then
        printf "%s" "${robs[$i]}"
    else
        printf ", %s" "${robs[$i]}"
    fi
done
echo "]"