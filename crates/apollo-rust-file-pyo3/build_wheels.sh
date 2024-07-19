#!/bin/bash

# Define the Python versions and their corresponding virtual environment directories
declare -A PYTHON_ENVS=(
    ["3.9"]="myenv39"
    ["3.10"]="myenv310"
    ["3.11"]="myenv311"
)

# Loop through each Python version and build the wheels
for VERSION in "${!PYTHON_ENVS[@]}"; do
    PYTHON_BIN="python$VERSION"
    VENV_DIR="${PYTHON_ENVS[$VERSION]}"

    # Create the virtual environment if it doesn't exist
    if [ ! -d "$VENV_DIR" ]; then
        $PYTHON_BIN -m venv $VENV_DIR
    fi

    # Activate the virtual environment
    source $VENV_DIR/bin/activate

    # Build the wheels using maturin
    maturin build

    # Deactivate the virtual environment
    deactivate
done
