# #!/bin/bash

# folders=(
#     "results_5M"
#     "results_10M" 
#     "results_15M"
#     "results_20M"
#     "results_25M"
#     "results_30M"
#     "results_35M"
#     "results_40M"
#     "results_45M"
#     "results_50M"
# )

# for folder in "${folders[@]}"; do
#     if [ -d "$folder" ]; then
#         # Process each file in folder
#         for file in "$folder"/*; do
#             if [ -f "$file" ]; then
#                 grep "CPU 0 Branch Prediction Accuracy:" "$file" 2>/dev/null | \
#                 awk '{print $6}' | sed 's/%//'
#             fi
#         done
#     fi
# done


#!/bin/bash

# Directory containing the trace files
directory="./results_50M"

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
echo "Branch Prediction Accuracies:"
echo "----------------------------"
for i in "${!names[@]}"; do
    printf "${accuracies[$i]}\n"
done
echo -e "\n"

# Print MPKIs
echo "Branch MPKI Values:"
echo "------------------"
for i in "${!names[@]}"; do
    printf "${mpkis[$i]}\n"
done
echo -e "\n"

# Print ROB Occupancies
echo "Average ROB Occupancies at Mispredict:"
echo "-------------------------------------"
for i in "${!names[@]}"; do
    printf "${robs[$i]}\n"
done