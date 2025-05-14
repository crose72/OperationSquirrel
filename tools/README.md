# Tools

This folder contains generic tools specific to Operation Squirrel, such as plotting the recorded data.

## data_log_plotter_v1.py

Written with Python 3.11.3.  Install the following dependencies:

```
pip install pandas plotly tk
```

Select a `data.csv` file from `SquirrelDefender/data`


## Creating a python .venv locally in this directory
1. Install 'uv' from astral (https://docs.astral.sh/uv/)
   1. windows: `winget install astral-sh.uv`
   2. linux: `curl -LsSf https://astral.sh/uv/install.sh | sh`

2. Create venv `uv venv`

3. Activate venv
   1. windows: `.venv\Scripts\activate`
   2. linux: `source .venv/bin/activate`

4. Add packages (unneeded for current functionality)
   1. `uv add my_sweet_package`
