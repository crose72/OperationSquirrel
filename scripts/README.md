## Overview

This folder contains scripts used to **build, run, and configure** the Operation Squirrel containers and Jetson environments.

- `helpers/` — contains platform-specific or supporting scripts called by the main ones  
- `archive/` — contains older or optional scripts for installation and development reference  

## 🧱 Primary Scripts

| Script | Purpose |
|--------|----------|
| **`run.sh`** | Runs either the **development** or **SquirrelDefender** container for a specified target (e.g. Orin, B01, Ubuntu). |
| **`build.sh`** | Builds the **SquirrelDefender** container for a specified target (e.g. Orin, B01). |
| **`setup.sh`** | Configures a Jetson to automatically start SquirrelDefender on power-up. Also configures OSRemote. |
| **`gen_test_csv_from_mcap.sh`** | Converts `.mcap` logs into test-harness compatible CSVs. |
| **`toggle_osremote.sh`** | Toggle jetson between hotspot mode and it's usual wifi connection. |

## 🚀 Example: `run.sh`

```bash
cd OperationSquirrel/scripts

# dev containers
# for the orin
./run.sh dev orin

# for the jetson nano b01
./run dev b01

# for ubuntu-22.02 with gpu sm 8.6
./run.sh dev ubuntu-22.04_sm86

# squirreldefender containers (deployment)
# for the orin
./run.sh squirreldefender orin

# for the jetson nano b01
./run.sh squirreldefender b01
```

## 🚀 Example: `build.sh`

```bash
cd OperationSquirrel/scripts

# squirreldefender containers (deployment)

# for the orin
./build.sh squirreldefender orin

# for the jetson nano b01
./build.sh squirreldefender b01
```

## ⚙️ Example: `setup.sh`

```bash
# After starting up the jetson
cd OperationSquirrel/scripts
# Don't run as sudo - will cause the squirreldefender.service to look for the run script in the /home/root/ path, you want /home/$USER/
./setup.sh squirreldefender --jetson=[orin|b01] # sets up the jetson to run the release container when the jetson powers on

./setup.sh osremote # sets up the jetson to be ready to be controlled from the osremote app

# This should setup the necessary components to enable the squirreldefender program to start as soon as the jetson is powered (useful when putting the jetson on a vehicle).
```

## 🚀 Example: `toggle_osremote.sh`

```bash
cd OperationSquirrel/scripts
sudo ./toggle_osremote.sh

# If connected to wifi, running this script will save your wifi credentials and put the jetson in access point mode (hotspot).  You can then connect your phone to this "hotspot" and use the OSRemote app to improve your dev experience in the field.  If the jetson is in hotspot mode and you are back in range of your wifi, running this script should connect the jetson back to the wifi network it was on before.
```

## 🧩 Example: `gen_test_csv_from_mcap.sh`

```bash
# Create and activate virtual environment
python3 -m venv os-venv
source os-venv/bin/activate

cd OperationSquirrel/scripts
./gen_test_csv_from_mcap.sh --proto ../proto ../test_data/2025-10-31-test-flight/*.mcap
```

The script will automatically:
- Detect your active Python virtual environment  
- Install the required dependencies (`protobuf`, `grpcio-tools`, `pandas`, `mcap`) if they are missing  
- Generate Python protobuf bindings (`*_pb2.py`) from `.proto` files if they don’t exist  
- Run the `mcap2CSV.py` conversion script to produce CSV files  

## 🗃️ Archived / Legacy Notes

The archive/ folder contains older scripts used for development setup, including:
- Installing specific versions of CMake or Jetson dependencies (SLAM, OpenCV, Jetson-Inference, etc.)
- Building and running Docker containers manually
- Experimental or early setup utilities for direct Jetson development (before containerization simplified this workflow)

#### ⚠️ These scripts may be outdated but remain useful for reference or manual builds.

If you wish to build your own Docker images from these scripts, update the Docker Hub username in the image tags to your own account before pushing.

Example: Copying the device model before building a dev container (only needed on the jetson when building from the opencv install script)

The dev container builds OpenCV from source and needs the correct Jetson model to target the right GPU architecture.
Before building for the first time, copy the device tree model:

The dev container builds opencv from a script, and that script needs to know which Jetson is being used so it can compile opencv for the correct GPU generation/architecture.  So before building dev for the first time you need to copy the device tree model to the `docker/fake-proc/device-tree/` folder before building the container, like so.

```
sudo cp /proc/device-tree/model <path-to-OperationSquirrel-repo>/docker/fake-proc/device-tree/model
```
