# Dec-CEM Search and Track Minimal Working Example (MWE)

## Contact Details

 - **Author**: Rhett Hull 
 - **Email**: rhett.hull1@defence.gov.au

---

## Description

This code is to demonstrate how Dec-CEM works for a rudimentary search and track setup. These particular examples have not been optimised for speed. 

The main assumptions made are as follows:
 - Searchers
    - Fixed-speed bicycle kinematics
    - Perfect triangular sensor
- Targets
    - Follows bicycle kinematics
    - When not observed by the searchers, region the target could occupy in the future is defined by a holonomic robot bounded by the targets known maximimum speed.
    - Targets start moving on first observation by a searcher - this is an arbitrary constraint for demonstration purposes.
    - Target COP is centralised amongst searchers.
- Environment
    - Uniform prior over the NAI.

---

## Prerequisities

Requires ```python>=3.8```.

Create a virtual environment,
```bash
python -m venv ~/dstg-deccem
```

Then install the required dependencies,

```bash
pip install -r requirements.txt
```

Note, the [resources/dubins](resources/dubins) folder is a recompiled version of python ```dubins``` library updated to support ```python>=3.8```.

---

## Running **MWE**: $n$ agents and $m$ targets

To run the MWE, execute the `run.py` script,

```python
python run.py
```

The following options are available:

```python
Usage: run.py [OPTIONS]

Options:
  --config_fname TEXT  Path to the *.toml configuration.
  --output TEXT        Filename without ext. of the output video.
  --help               Show this message and exit.
```

By default, the output is saved under ```results/{output}.mp4```. To view it whilst running, uncomment the following in ```run.py```.

```
180: # plt.pause(0.1)
```

The simulation is controlled by the configuration defined in ```etc/config```. To add more targets or searchers, add more initial states under ```states```.

```bash
[sim]
    [sim.time]
    dt = 0.5
    max_T = 300

[nai]
    [nai.boundary]
    x = [0, 30]
    y = [0, 30]

[target]
    [target.kinematics]
    speed = 0.03

    # Controls the number of targets - defines the initial states
    states = [[10, 10, 0]] #, [15, 15, 0], [10, 20, 0]]
    max_turn_rate = 1.0

[searcher]
    [searcher.kinematics]
    speed = 1.0

    # Controls the number of searchers - defines the initial states
    states = [[0, 0, 0]] #, [0, 0, 0]]
    max_turn_rate = 2.5
    max_path_length = 20

    [searcher.cem]
    n_params = 5 # Number of sample configurations to take
    epsilon = 0.01 # Control variance to stop the optimisation
    alpha = 0.9 # Smooth update parameter
    n_elite = 3 # Number of elite samples to select
    n_samples = 10 # These samples are processed in parallel
    communication_period = 10 # number of CEM iterations to perform before communicating
    communication_cycles = 2 # total number of communication rounds

    [searcher.sensor]
    type = "triangle"

[sensor]
    [sensor.triangle]
    fov = 45 # degrees
    range = 4
    
    [sensor.circle]
    range = 4    
```