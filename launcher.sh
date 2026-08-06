#!/bin/bash

###############################################
# Master Experiment Launcher
###############################################

DATA_ROOT="$HOME/Desktop/DATA"

mkdir -p "$DATA_ROOT"

###############################################
# Generate Random Participant ID
###############################################

while true
do
    PARTICIPANT_ID="P$(printf "%06d" $((RANDOM * RANDOM % 1000000)))"

    # Make sure it doesn't already exist
    if [ ! -d "$DATA_ROOT/$PARTICIPANT_ID" ]; then
        break
    fi
done

export PARTICIPANT_ID
export PARTICIPANT_DIR="$DATA_ROOT/$PARTICIPANT_ID"

mkdir -p "$PARTICIPANT_DIR"

echo ""
echo "==============================================="
echo " Participant: $PARTICIPANT_ID"
echo " Saving data to:"
echo " $PARTICIPANT_DIR"
echo "==============================================="
echo ""

###############################################
# Helper Function
###############################################

run_experiment() {

    TITLE="$1"
    SCRIPT="$2"

    clear

    echo "==============================================="
    echo "$TITLE"
    echo "Participant: $PARTICIPANT_ID"
    echo "==============================================="
    echo ""
    read -p "Press ENTER when ready..."

    bash "$SCRIPT"
    
    
    if [ $? -ne 0 ]; then
        echo
        echo "ERROR: $TITLE failed."
        exit 1
    fi

    echo ""
    echo "==============================================="
    echo "$TITLE COMPLETE"
    echo "==============================================="
    echo ""
    read -p "Press ENTER for the next experiment..."
}

###############################################
# 5 Robot
###############################################

mkdir -p "$PARTICIPANT_DIR/5_robot"

export EXPERIMENT_FOLDER="$PARTICIPANT_DIR/5_robot"

run_experiment "5 Robot - Priority Single" "./experiment_launchers_5robot/priority_single.sh"
run_experiment "5 Robot - Priority Group"  "./experiment_launchers_5robot/priority_group.sh"
run_experiment "5 Robot - Urgency Single"  "./experiment_launchers_5robot/urgency_single.sh"
run_experiment "5 Robot - Urgency Group"   "./experiment_launchers_5robot/urgency_group.sh"

###############################################
# 4 Robot
###############################################

mkdir -p "$PARTICIPANT_DIR/4_robot"

export EXPERIMENT_FOLDER="$PARTICIPANT_DIR/4_robot"

run_experiment "4 Robot - Priority Single" "./experiment_launchers_4robot/priority_single.sh"
run_experiment "4 Robot - Priority Group"  "./experiment_launchers_4robot/priority_group.sh"
run_experiment "4 Robot - Urgency Single"  "./experiment_launchers_4robot/urgency_single.sh"
run_experiment "4 Robot - Urgency Group"   "./experiment_launchers_4robot/urgency_group.sh"

###############################################
# 3 Robot
###############################################

mkdir -p "$PARTICIPANT_DIR/3_robot"

export EXPERIMENT_FOLDER="$PARTICIPANT_DIR/3_robot"

run_experiment "3 Robot - Priority Single" "./experiment_launchers_3robot/priority_single.sh"
run_experiment "3 Robot - Priority Group"  "./experiment_launchers_3robot/priority_group.sh"
run_experiment "3 Robot - Urgency Single"  "./experiment_launchers_3robot/urgency_single.sh"
run_experiment "3 Robot - Urgency Group"   "./experiment_launchers_3robot/urgency_group.sh"

###############################################
# Finished
###############################################

clear

echo ""
echo "==============================================="
echo " ALL EXPERIMENTS COMPLETE"
echo ""
echo " Participant: $PARTICIPANT_ID"
echo ""
echo " Data saved in:"
echo " $PARTICIPANT_DIR"
echo "==============================================="
echo ""
