#!/bin/bash
#SBATCH --job-name=sionna_array
#SBATCH --output=logs/sionna_%A_%a.out
#SBATCH --error=logs/sionna_%A_%a.err
#SBATCH --time=02:00:00
#SBATCH --cpus-per-task=1
#SBATCH --mem=4G
#SBATCH --array=0-299%20

# NOTE: Pre-build once before submitting this array:
#   ./ns3 build scratch/SionnaSub.cc

ROOTDIR=UrbanRaCompDir
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
if [ -z "$TASK_ID" ]; then
  echo "SLURM_ARRAY_TASK_ID is not set."
  exit 1
fi

epoch_index=$((TASK_ID / tasks_per_epoch))
rem=$((TASK_ID % tasks_per_epoch))
ra_index=$((rem / (num_nodes * num_srcs)))
rem2=$((rem % (num_nodes * num_srcs)))
node_index=$((rem2 / num_srcs))
src_index=$((rem2 % num_srcs))

if [ "$epoch_index" -ge "$num_epochs" ]; then
  echo "Invalid array task id: $TASK_ID"
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

CMD="./ns3 run scratch/SionnaSub.cc --no-build -- --maxNodes=${NODES_VAL} --routing=${RA} --resultPath=${RESULT_DIR} --numSource=${NUMSRC} --layoutFile=${LAYOUT_FILE} --seed=${SEED}"

echo "Running task ${TASK_ID}: ${CMD}"
exec $CMD
