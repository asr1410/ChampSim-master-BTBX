#!/bin/bash

folders=(
    "results_5M"
    "results_10M" 
    "results_15M"
    "results_20M"
    "results_25M"
    "results_30M"
    "results_35M"
    "results_40M"
    "results_45M"
    "results_50M"
)

for folder in "${folders[@]}"; do
    if [ -d "$folder" ]; then
        # Process each file in folder
        for file in "$folder"/*; do
            if [ -f "$file" ]; then
                grep "CPU 0 Branch Prediction Accuracy:" "$file" 2>/dev/null | \
                awk '{print $6}' | sed 's/%//'
            fi
        done
    fi
done