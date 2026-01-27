#!/bin/bash

# SUMO Batch Runner Script
# This script runs SUMO simulations in batches of 30, processing different net files
# and their corresponding route files

# Configuration (override via environment variables)
BATCH_SIZE="${BATCH_SIZE:-1}"
SUMO_CMD="${SUMO_CMD:-sumo}"
TRIPS_OUTPUT_DIR="${TRIPS_OUTPUT_DIR:-outputs/trips_output}"
SSM_OUTPUT_DIR="${SSM_OUTPUT_DIR:-outputs/ssm_output}"
LOGS_DIR="${LOGS_DIR:-outputs/simulation_logs}"

# Base directories for scenario networks and routes
NET_BASE_DIR="${NET_BASE_DIR:-.}"
ROUTE_BASE_DIR="${ROUTE_BASE_DIR:-.}"

# Create output directories if they don't exist
mkdir -p "$TRIPS_OUTPUT_DIR"
mkdir -p "$SSM_OUTPUT_DIR"
mkdir -p "$LOGS_DIR"

# Define the net files and their corresponding route directories
declare -A NET_ROUTE_MAPPING=(
    ["$NET_BASE_DIR/merge/merge.net.xml"]="$ROUTE_BASE_DIR/merge"
    ["$NET_BASE_DIR/straight/straight.net.xml"]="$ROUTE_BASE_DIR/straight"
    ["$NET_BASE_DIR/secondary_merge/secondary_merge.net.xml"]="$ROUTE_BASE_DIR/secondary_merge"
    ["$NET_BASE_DIR/barrandov/reset.net.xml"]="$ROUTE_BASE_DIR/barrandov"
)

# Function to run a single SUMO simulation
run_simulation() {
    local net_file="$1"
    local route_file="$2"
    local trips_file="$3"
    local ssm_file="$4"
    local log_stdout="$5"
    local log_stderr="$6"
    
    echo "Running: $SUMO_CMD -n \"$net_file\" -r \"$route_file\" --tripinfo-output \"$trips_file\" --device.ssm.file \"$ssm_file\""
    $SUMO_CMD -n "$net_file" -r "$route_file" --tripinfo-output "$trips_file" --device.ssm.file "$ssm_file" --time-to-teleport 5 --lateral-resolution 0.8 --step-length 0.05 -e 7200 > "$log_stdout" 2> "$log_stderr"
    
    if [ $? -eq 0 ]; then
        echo "✓ Success: $(basename "$route_file")"
    else
        echo "✗ Failed: $(basename "$route_file")"
    fi
}

# Function to run a batch of simulations
run_batch() {
    local batch_jobs=("$@")
    local batch_size=${#batch_jobs[@]}
    
    echo "Starting batch of $batch_size simulations..."
    echo "=========================================="
    
    # Run all jobs in the batch in parallel
    for job in "${batch_jobs[@]}"; do
        eval "$job" &
    done
    
    # Wait for all jobs in this batch to complete
    wait
    
    echo "=========================================="
    echo "Batch completed!"
    echo ""
}

# Main execution
echo "SUMO Batch Runner"
echo "================="
echo "Batch size: $BATCH_SIZE"
echo "Trips output directory: $TRIPS_OUTPUT_DIR"
echo "SSM output directory: $SSM_OUTPUT_DIR"
echo "Logs directory: $LOGS_DIR"
echo ""

# Collect all simulation jobs
declare -a all_jobs=()
job_count=0

# Iterate through each net file and its corresponding route directory
for net_file in "${!NET_ROUTE_MAPPING[@]}"; do
    route_dir="${NET_ROUTE_MAPPING[$net_file]}"
    
    # Check if net file exists
    if [ ! -f "$net_file" ]; then
        echo "Warning: Net file not found: $net_file"
        continue
    fi
    
    # Check if route directory exists
    if [ ! -d "$route_dir" ]; then
        echo "Warning: Route directory not found: $route_dir"
        continue
    fi
    
    # Extract network name for organizing outputs
    net_name=$(basename "$net_file" .net.xml)
    
    # Create network-specific directories
    mkdir -p "$TRIPS_OUTPUT_DIR/$net_name"
    mkdir -p "$SSM_OUTPUT_DIR/$net_name"
    mkdir -p "$LOGS_DIR/$net_name"
    
    echo "Processing net: $net_file (network: $net_name)"
    echo "Route directory: $route_dir"
    
    # Find all route files in the directory (including base.rou.xml)
    while IFS= read -r -d '' route_file; do
        # Extract the base name without extension
        base_name=$(basename "$route_file" .rou.xml)
        
        # Create trips output file path (organized by network)
        trips_file="$TRIPS_OUTPUT_DIR/$net_name/${base_name}.trips.xml"
        
        # Create SSM output file path (organized by network)
        ssm_file="$SSM_OUTPUT_DIR/$net_name/${base_name}.ssm.xml"
        
        # Create log file paths (organized by network)
        log_stdout="$LOGS_DIR/$net_name/${base_name}_stdout.log"
        log_stderr="$LOGS_DIR/$net_name/${base_name}_stderr.log"
        
        # Create the simulation command
        job_cmd="run_simulation \"$net_file\" \"$route_file\" \"$trips_file\" \"$ssm_file\" \"$log_stdout\" \"$log_stderr\""
        all_jobs+=("$job_cmd")
        ((job_count++))
        
    done < <(find "$route_dir" -name "*.rou.xml" -print0)
    
    echo "Found $job_count total route files"
    echo ""
done

echo "Total simulations to run: $job_count"
echo ""

# Process jobs in batches
current_batch=()
batch_num=1
total_jobs=${#all_jobs[@]}
current_job_index=0

for job in "${all_jobs[@]}"; do
    current_batch+=("$job")
    ((current_job_index++))
    
    # When batch is full or this is the last job, run the batch
    if [ ${#current_batch[@]} -eq $BATCH_SIZE ] || [ $current_job_index -eq $total_jobs ]; then
        echo "Running Batch $batch_num (${#current_batch[@]} simulations)"
        run_batch "${current_batch[@]}"
        
        # Reset for next batch
        current_batch=()
        ((batch_num++))
    fi
done

echo "All simulations completed!"
echo "Check the '$TRIPS_OUTPUT_DIR' directory for trip output files (organized by network)."
echo "Check the '$SSM_OUTPUT_DIR' directory for SSM output files (organized by network)."
echo "Check the '$LOGS_DIR' directory for simulation logs (organized by network)."
