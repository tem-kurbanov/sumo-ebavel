# SUMO Scenario Kit: Driverless Vehicle Effects

This repository contains a small set of SUMO scenarios and analysis scripts for studying
how driverless (EGO) vehicle behavior affects surrounding traffic. It includes three
synthetic scenarios (straight, merge, secondary_merge) and one real-world, large-scale
urban scenario (Barrandov). The primary intent is to compare behavior configurations
against a baseline and report impacts on traffic efficiency and surrogate safety measures.

The scenarios are designed to run with 3D visualization using the SUMO fork
`sumo3DView` for richer visual inspection of behavior effects and interactions. See
the fork here: [`github.com/BalduwinIV/sumo3DView`](https://github.com/BalduwinIV/sumo3DView).

## Scenarios

- `straight/`: Straight highway segment for baseline behavior comparisons.
- `merge/`: Merge zone scenario for lane-changing and cooperation effects.
- `secondary_merge/`: More complex multi-stream merge/diverge behavior.
- `barrandov/`: Real-world urban scenario based on the Barrandov area with a larger
  network and realistic traffic characteristics.

Each scenario folder contains SUMO network and configuration files such as
`.net.xml`, `.sumocfg`, and route files (`.rou.xml`).

## Analysis Scripts

The `analysis/` folder provides scripts to compare EGO behavior configurations against
baseline traffic and to generate summary reports:

- `analysis/traffic_analysis.py`: Core analysis and comparison utilities.
- `analysis/batch_analysis.py`: Batch analysis across all configurations.
- `analysis/run_full_batch_analysis.py`: One-command batch runner with logging.
- `analysis/create_routes.py`: Generate EGO-configured route files.
- `analysis/batch_run.sh`: Run SUMO simulations in batches and collect outputs.

Most scripts expect precomputed SUMO outputs in `trips_output/` and `ssm_output/`.
The batch runner generates these outputs for each scenario and configuration.

## Quick usage

All Python scripts support `--help`:

```bash
python analysis/create_routes.py --help
python analysis/batch_analysis.py --help
python analysis/run_full_batch_analysis.py --help
```

### Generate route files for a scenario (optional)

`analysis/create_routes.py` can generate per-configuration `.rou.xml` files into an output directory.
It can also insert extra EGO vehicles (route + departure timing configurable via CLI).

```bash
python analysis/create_routes.py ^
  --scenario barrandov ^
  --initial-route-file barrandov/routes.rou.xml ^
  --output-dir experiments/barrandov ^
  --add-ego-count 10 ^
  --add-ego-start-time 1000 ^
  --add-ego-depart-interval 1 ^
  --add-ego-route-edges "<EDGE_ID_1 EDGE_ID_2 ...>" ^
  --add-ego-color "255,255,255"
```

### Run SUMO simulations in batches

`analysis/batch_run.sh` is a plain bash script (not Slurm). On Windows, run it via WSL or Git Bash.
It is configured via environment variables (defaults shown below).

Important: the analysis scripts look for `trips_output/` and `ssm_output/` by default. Either:
- Set `TRIPS_OUTPUT_DIR=trips_output` and `SSM_OUTPUT_DIR=ssm_output` when running the batch runner, or
- Move/copy outputs from `outputs/` into `trips_output/` and `ssm_output/` before analysis.

Example (using the analysis-friendly output locations):

```bash
TRIPS_OUTPUT_DIR=trips_output SSM_OUTPUT_DIR=ssm_output LOGS_DIR=simulation_logs \
  BATCH_SIZE=4 SUMO_CMD=sumo \
  bash analysis/batch_run.sh
```

### Run batch analysis

Once `trips_output/` and `ssm_output/` exist, you can run batch analysis over a directory of route configs:

```bash
python analysis/run_full_batch_analysis.py --configs-dir experiments/barrandov --yes
```

## Outputs

Analysis results are written to structured output directories (e.g.
`batch_analysis_results_*`) that include:

- Per-configuration EGO analysis tables and JSON data
- Summary tables for aggregate performance across scenarios
- Plots and reports for quick review

See the interpretation guides for detailed metric definitions and output formats:

- `analysis/TRAFFIC_ANALYSIS_DOCUMENTATION.md`

## Visualization

For interactive 3D visualization, run the scenarios with the `sumo3DView` fork of
SUMO: [`github.com/BalduwinIV/sumo3DView`](https://github.com/BalduwinIV/sumo3DView). This is especially useful for
qualitative inspection of lane-changing and merging behaviors.

