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
directory="./old_results_50M"

# Process each file in the directory
for file in "$directory"/*.txt; do
    if [ -f "$file" ]; then
        # Extract the line containing Branch Prediction Accuracy
        line=$(grep "CPU 0 Branch Prediction Accuracy" "$file")
        
        if [ ! -z "$line" ]; then
            # Extract filename without extension and path
            filename=$(basename "$file" | cut -d'.' -f1)
            
            # Extract accuracy and MPKI using awk
            accuracy=$(echo "$line" | awk -F'[: %]+' '{print $6}')
            mpki=$(echo "$line" | awk '{print $8}')
            
            # Print in desired format
            echo "Name: $filename Accuracy: $accuracy MPKI: $mpki"
        fi
    fi
done