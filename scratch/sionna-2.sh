#!/bin/bash
# =============================================================================
# JOB SCRIPT TEMPLATE FOR SLURM - NSWT Cluster
# =============================================================================
# This script supports two modes:
# 1) BUILD_ONLY=1 : configure and build the ns-3 tree once.
# 2) default      : run an array task for one simulation using --no-build.
# Lines starting with #SBATCH are Slurm directives, NOT comments!
# =============================================================================

# --- JOB NAME (will appear in squeue) ---
#SBATCH --job-name=sionna_array

# --- PARTITION ---
#SBATCH --partition=ns3
#SBATCH --account=ns3

# --- OUTPUT AND ERRORS ---
# %j is automatically replaced with the Job ID
# %a is replaced with the task index (useful for array jobs)
#SBATCH --output=%j_%a.out
#SBATCH --error=%j_%a.err

# --- MAXIMUM EXECUTION TIME ---
# Format: days-hours:minutes:seconds or hours:minutes:seconds
# The job will be automatically killed if it exceeds this limit.
# Be reasonable: if you estimate 2 hours, set 4 hours as margin.
# Maximum allowed: 7 days (7-00:00:00)
#SBATCH --time=1-00:00:00

# --- CPU ---
# Use 1 CPU per simulation task if each ns-3 run is single-threaded.
#SBATCH --cpus-per-task=1

# --- RAM ---
# Default: 4GB per CPU (with 4 CPUs → 16GB automatically).
# Maximum requestable: 16GB per CPU (with 4 CPUs → max 64GB).
# Uncomment only if you need more RAM than the default.
##SBATCH --mem=32G

# --- GPU ---
# Maximum 2 GPUs per job.
# Maximum 2 GPUs simultaneously per user across the entire cluster.
##SBATCH --gpus=1

# --- ARRAY JOB ---
# 300 tasks, max 20 running concurrently
# Set the array range on submission so BUILD_ONLY can still use a single task.
# Example to run the simulation array:
#   sbatch --array=0-299%20 scratch/sionna.sh
# Example to build only once:
#   sbatch --array=0-0 --export=ALL,BUILD_ONLY=1 scratch/sionna.sh
##SBATCH --array=0-299%20

# =============================================================================
# ENVIRONMENT
# =============================================================================

# Reset limits inherited from the login node (DO NOT remove these lines!)
ulimit -v unlimited
ulimit -n 1024
ulimit -t unlimited
ulimit -f unlimited

# System and CUDA paths
export PATH=/usr/local/cuda/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:/usr/local/cuda/extras/CUPTI/lib64

set -euo pipefail

# =============================================================================
# DIAGNOSTICS
# =============================================================================
echo "Job ID: $SLURM_JOB_ID"
echo "Array Task ID: ${SLURM_ARRAY_TASK_ID:-none}"
echo "Node: $(hostname)"
echo "Start: $(date)"

# =============================================================================
# SETUP
# =============================================================================
source /home/$USER/venv/firez/bin/activate
cd /home/$USER/ns-3-dev/

PYBIND11_INCLUDE=$(python3 -c "import pybind11; print(pybind11.get_include())")
PYTHON_INCLUDE=$(python3 -c "import sysconfig; print(sysconfig.get_path('include'))")

echo "pybind11 include: $PYBIND11_INCLUDE"
echo "Python include: $PYTHON_INCLUDE"

if [ "${BUILD_ONLY:-0}" = "1" ]; then
    echo "Build mode enabled. Configure and build ns-3 only."
    ./ns3 clean
    ./ns3 configure --enable-python-bindings -- \
        -DPython3_EXECUTABLE=$(which python3) \
        -DCMAKE_CXX_FLAGS="-I${PYBIND11_INCLUDE} -I${PYTHON_INCLUDE}"
    ./ns3 build --jobs 24
    echo "Build completed. Submit the array job separately for simulations."
    exit 0
fi

if [ -z "${SLURM_ARRAY_TASK_ID:-}" ]; then
    echo "SLURM_ARRAY_TASK_ID is not set. Submit with sbatch --array or set the env variable."
    exit 1
fi

# =============================================================================
# SIMULATION SETUP
# =============================================================================
ROOTDIR=UrbanRaCompDir-sionna
RAs=(aodv olsr dsdv)
NODES=(10 20 30 40 50 60 70 80 90 100)
NUMSRCS=(6)
EPOCHS=(1 2 3 4 5 6 7 8 9 10)

num_ras=${#RAs[@]}
num_nodes=${#NODES[@]}
num_srcs=${#NUMSRCS[@]}
num_epochs=${#EPOCHS[@]}

tasks_per_epoch=$((num_ras * num_nodes * num_srcs))
TASK_ID=${SLURM_ARRAY_TASK_ID}
epoch_index=$((TASK_ID / tasks_per_epoch))
rem=$((TASK_ID % tasks_per_epoch))
ra_index=$((rem / (num_nodes * num_srcs)))
rem2=$((rem % (num_nodes * num_srcs)))
node_index=$((rem2 / num_srcs))
src_index=$((rem2 % num_srcs))

if [ "$epoch_index" -ge "$num_epochs" ]; then
    echo "Invalid SLURM_ARRAY_TASK_ID: $TASK_ID"
    exit 1
fi

RA=${RAs[$ra_index]}
NODES_VAL=${NODES[$node_index]}
NUMSRC=${NUMSRCS[$src_index]}
EPOCH=${EPOCHS[$epoch_index]}

RESULT_DIR="$ROOTDIR/Epoch_${EPOCH}/${RA}/numNodes/${NODES_VAL}"
mkdir -p "$RESULT_DIR"

LAYOUT_FILE="scratch/node2.csv"
SEED="$EPOCH"

CMD=(./ns3 run scratch/SionnaSub.cc --no-build -- --maxNodes=${NODES_VAL} --routing=${RA} --resultPath=${RESULT_DIR} --numSource=${NUMSRC} --layoutFile=${LAYOUT_FILE} --seed=${SEED})

echo "Running task $TASK_ID: ${CMD[*]}"
exec "${CMD[@]}"
