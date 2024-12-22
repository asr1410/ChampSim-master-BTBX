#!/bin/bash

# Input directory
input_dir="old_results_50M"

# Output directory
output_dir="results_csv"

# Navigate to the input directory
cd "$input_dir" || { echo "Input directory not found!"; exit 1; }

# Create the output directory if it doesn't exist
mkdir -p "../$output_dir"

# Loop through all .txt files in the directory
for file in *.txt; do
    # Extract the server name (e.g., "server_001") from the filename
    base_name=$(echo "$file" | cut -d'.' -f1)
    
    # Save the last 513 lines to the output directory with the shortened name
    tail -n 513 "$file" > "../$output_dir/${base_name}.csv"
    
    echo "Processed $file -> ../$output_dir/${base_name}.csv"
done