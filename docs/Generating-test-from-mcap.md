# MCAP → CSV Conversion

## Description

This document explains how to convert `.mcap` log files into CSV format for use in the test suite.  The Operation Squirrel test harness looks for data with monotonically increasing timestamps in each row.  The script automatically handles everything — including dependency installation and protobuf generation — with **no Linux-level dependencies** (no `apt install`).  Everything runs using **Python** inside a virtual environment - this has been tested in WSL2 Ubuntu-22.04 with python 3.10.12.

---

## How to use

1. **Create and activate a virtual environment**

    ```bash
    python3 -m venv os-venv
    source os-venv/bin/activate
    ```

2. **Navigate to the scripts directory**

    From the project root:
    ```bash
    cd OperationSquirrel/scripts
    ```

3. **Run the script**

    Run the following command, replacing the paths with your own `.mcap` files:

    ```bash
    ./createTestFromMCAP.sh --proto ../proto ../test_data/2025-10-31-test-flight/*.mcap
    ```

   The script will automatically:
    - Detect your active Python virtual environment  
    - Install the required dependencies (`protobuf`, `grpcio-tools`, `pandas`, `mcap`) if they are missing  
    - Generate Python protobuf bindings (`*_pb2.py`) from `.proto` files if they don’t exist  
    - Run the `mcap2CSV.py` conversion script to produce CSV files  
