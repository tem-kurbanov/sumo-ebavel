## Driverless Vehicle Behaviour Impact Study (SUMO Scenarios + Analysis Toolkit)

This repository provides a **scenario set and analysis workflow** for studying how **driverless (“EGO”) vehicle behaviour** affects surrounding traffic in microscopic simulation. The primary goal is to support **controlled, repeatable experiments** that compare EGO behaviour configurations against a baseline and quantify impacts on **traffic efficiency** and **surrogate safety measures (SSM)**.

The repository also includes a SUMO-based simulator build with an **interactive 3D visualization view** to support qualitative inspection of interactions (lane changes, merges, conflicts) alongside quantitative analysis.

### Repository contents

- **`scenarios/`**: The experiment scenarios and the full analysis pipeline (batch simulation runner + batch analysis).
- **`sumo/`**: A SUMO source tree (based on **SUMO v1.23**) used for running the experiments. It includes 3D visualization functionality in `sumo-gui`.
- **`Developing a 3D Visualization Kit for.pdf`**: The bachelor thesis documenting the 3D visualization work, simulator requirements, and usage guidance.

### Experimental design (summary)

The experiments introduce **EGO vehicles** (vehicles running specific car-following and lane-changing behaviour configurations) into baseline traffic and evaluate:

- **Traffic efficiency metrics** (e.g., trip duration and speed impacts)
- **Safety proxies** via **SSM outputs** (surrogate safety measures)

The study design supports systematic variation of five behaviour dimensions (see the interpretation guide for definitions and encoding):

- Following distance preference (`tau`)
- Strategic planning horizon (`lcStrategic`)
- Cooperation level (`lcCooperative` / `lcCooperativeSpeed`)
- Speed optimization seeking (`lcSpeedGain` / `lcSpeedGainLookahead`)
- Gap acceptance behaviour (`lcPushy` / `lcPushyGap`)

The included tooling and documentation are designed to process **243 behaviour configurations** across multiple scenarios.

### Scenarios

The scenario suite is documented in [`scenarios/README.md`](scenarios/README.md) and includes:

- **`straight/`**: Straight highway segment
- **`merge/`**: Merge zone
- **`secondary_merge/`**: Multi-stream merge/diverge
- **`barrandov/`**: Large-scale urban scenario (Barrandov area)

### Recommended workflow

The workflow is designed around three stages: **(1) generate configurations (optional)** → **(2) run simulations in batches** → **(3) run batch analysis**.

#### 1) (Optional) Generate route/configuration sets

From `scenarios/`:

```bash
python analysis/create_routes.py --help
```

#### 2) Run simulations in batches (produce `tripinfo` + SSM outputs)

From `scenarios/`:

```bash
TRIPS_OUTPUT_DIR=trips_output SSM_OUTPUT_DIR=ssm_output LOGS_DIR=simulation_logs \
  BATCH_SIZE=4 SUMO_CMD=sumo \
  bash analysis/batch_run.sh
```

Notes:
- The batch runner writes `tripinfo-output` and SSM outputs (`--device.ssm.file`) and organizes them per scenario/network.
- The analysis scripts expect outputs under:
  - `trips_output/<scenario>/<config_id>.trips.xml`
  - `ssm_output/<scenario>/<config_id>.ssm.xml`

#### 3) Run batch analysis (produce tables, plots, and JSON summaries)

From `scenarios/` (after outputs exist):

```bash
python analysis/run_full_batch_analysis.py --configs-dir experiments/barrandov --yes
```

The analysis outputs a timestamped directory (e.g. `batch_analysis_results_YYYYMMDD_HHMMSS/`) containing per-configuration results and aggregated summaries.

### Interpretation guide (how to read results)

For detailed interpretation of metrics, output structure, colour coding, and configuration encoding, see:

- [`scenarios/analysis/TRAFFIC_ANALYSIS_DOCUMENTATION.md`](scenarios/analysis/TRAFFIC_ANALYSIS_DOCUMENTATION.md)

### Running scenarios interactively (visual inspection)

You can run individual scenarios in the GUI for inspection:

```bash
sumo-gui -c scenarios/merge/merge.sumocfg
sumo-gui -c scenarios/straight/straight.sumocfg
sumo-gui -c scenarios/secondary_merge/secondary_merge.sumocfg
sumo-gui -c scenarios/barrandov/osm.sumocfg
```

If you are using the `sumo/` source tree in this repository, build/install it according to the thesis and ensure `sumo` / `sumo-gui` are on your `PATH` (or invoke the built binaries directly).

#### 3D visualization

When built with OSG support, `sumo-gui` provides an additional 3D view for interactive inspection. In the GUI, open it via:

- **Window → “Open new 3D view”**

### Documentation and attribution

- **3D visualization work (SUMO-side changes)**: implemented by **Nikita Sazanov** ([GitHub profile](https://github.com/BalduwinIV)) as part of his bachelor project.
- **Primary documentation**: `Developing a 3D Visualization Kit for.pdf`.
- **Base simulator**: Eclipse SUMO (Simulation of Urban MObility).

### Licensing

This repository contains SUMO source code and related materials. See the licensing information in `sumo/LICENSE`, `sumo/NOTICE.md`, and the upstream SUMO project documentation.

<a href="https://sumo.dlr.de/docs"><p align="center"><img width=50% src="https://raw.githubusercontent.com/eclipse/sumo/main/docs/web/docs/images/sumo-logo.svg"></p></a>

## sumo3DView — SUMO 1.23 fork with 3D visualization (OpenSceneGraph)

This repository is a **fork of Eclipse SUMO (Simulation of Urban MObility), based on SUMO v1.23**, extended with an **interactive 3D visualization** inside `sumo-gui` (OpenSceneGraph / OSG).

It is used to run and visually inspect experiments on **how driverless (“EGO”) vehicle behavior affects surrounding traffic**, across multiple scenarios and behavior configurations.

The SUMO-side 3D changes were implemented by **Nikita Sazanov** (`https://github.com/BalduwinIV`) as part of his bachelor project. The accompanying thesis PDF in this repository is the main reference:

- `Developing a 3D Visualization Kit for.pdf` (extensive documentation of changes, requirements, build/setup, and usage guide)

### What’s in this repo

- **Full SUMO source tree (v1.23-based)**: this is not a small plugin; it’s a forked SUMO codebase.
- **3D view integration**: `sumo-gui` can open an additional **3D OSG view** of the current simulation.
- **Experiment material**: the `scenarios/` directory contains the experiment scenarios and analysis workflow.

### Key differences vs upstream SUMO

- **3D visualization inside `sumo-gui`**:
  - Adds a 3D view type backed by OpenSceneGraph (OSG), alongside the standard 2D OpenGL view.
  - In the GUI you can open it via **Window → “Open new 3D view”** (only available when built with OSG support).
- **Scenario + analysis kit for EGO impact experiments**: `scenarios/` contains runnable scenarios plus scripts to generate routes, batch-run simulations, and analyze outputs.

For general SUMO capabilities and documentation, see the upstream project:
- Eclipse SUMO docs: `https://sumo.dlr.de/docs`
- Upstream code: `https://github.com/eclipse-sumo/sumo`

### Building / requirements

This fork must be **built with OSG enabled** to use the 3D view in `sumo-gui`. Exact dependencies and build instructions are documented in:

- [`Developing a 3D Visualization Kit for.pdf`](Developing%20a%203D%20Visualization%20Kit%20for.pdf)

### Usage (3D visualization)

1. **Run a scenario in `sumo-gui`** (examples below).
2. In the GUI, open the 3D view via **Window → “Open new 3D view”** (or the corresponding toolbar button/icon if present).
   - If that menu item is missing/disabled, your build likely does **not** include OSG support.

### Scenarios and experiment workflow (driverless/EGO impact study)

All scenario + experiment guidance lives under `scenarios/`:

- Main entry point: [`scenarios/README.md`](scenarios/README.md)
- Metric interpretation guide: [`scenarios/analysis/TRAFFIC_ANALYSIS_DOCUMENTATION.md`](scenarios/analysis/TRAFFIC_ANALYSIS_DOCUMENTATION.md)

The study design introduces **EGO vehicles** with systematically varied car-following and lane-changing behavior parameters and compares them against a **baseline** across scenarios. The provided analysis workflow is designed to handle **243 behavior configurations** and reports both traffic efficiency metrics and **surrogate safety measures (SSM)**.

#### Included scenarios

- **`straight/`**: straight highway segment (baseline comparisons)
- **`merge/`**: merge zone (lane-changing and cooperation effects)
- **`secondary_merge/`**: more complex multi-stream merge/diverge
- **`barrandov/`**: large urban scenario (Barrandov area)

Each scenario directory contains SUMO inputs like `.net.xml`, `.sumocfg`, and route files (`.rou.xml`).

#### Quick run examples

From the repository root:

```bash
sumo-gui -c scenarios/merge/merge.sumocfg
sumo-gui -c scenarios/straight/straight.sumocfg
sumo-gui -c scenarios/secondary_merge/secondary_merge.sumocfg
sumo-gui -c scenarios/barrandov/osm.sumocfg
```

On Windows, the Barrandov scenario also includes a helper:

```bat
scenarios\barrandov\run.bat
```

#### Batch simulations (generate `tripinfo` + SSM outputs)

`scenarios/analysis/batch_run.sh` runs simulations in batches (Windows: use WSL or Git Bash). It produces:

- `tripinfo-output` files (travel time / speed / etc.)
- SSM output files (surrogate safety measures), via `--device.ssm.file`

Example that matches the analysis scripts’ default expectations:

```bash
TRIPS_OUTPUT_DIR=trips_output SSM_OUTPUT_DIR=ssm_output LOGS_DIR=simulation_logs \
  BATCH_SIZE=4 SUMO_CMD=sumo \
  bash scenarios/analysis/batch_run.sh
```

#### Batch analysis

Once `trips_output/` and `ssm_output/` exist:

```bash
python scenarios/analysis/run_full_batch_analysis.py --configs-dir experiments/barrandov --yes
```

#### (Optional) Generate per-configuration route files

If you want to generate route sets for a scenario and inject additional EGO vehicles, use:

```bash
python scenarios/analysis/create_routes.py --help
```

### Attribution / licensing

- **Upstream project**: Eclipse SUMO (Simulation of Urban MObility) — `https://sumo.dlr.de/`
- **3D visualization fork work (bachelor project)**: Nikita Sazanov — `https://github.com/BalduwinIV`
- **License**: This repository contains SUMO source code and remains under SUMO’s licensing terms (see `LICENSE`, `NOTICE.md`, and upstream documentation).
