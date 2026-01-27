# Traffic Simulation Analysis Results: Interpretation Guide

## Executive Summary

This document provides guidance for interpreting the results of a comprehensive traffic simulation study examining the impact of various lane-changing and following behaviors on traffic flow efficiency and safety. The study analyzed 243 different driving behavior configurations across multiple traffic scenarios, evaluating both traditional traffic efficiency metrics and advanced Surrogate Safety Measures (SSM).

The results are organized in color-coded summary tables and individual configuration analyses, designed to facilitate rapid identification of optimal driving behaviors for different traffic conditions. This guide explains how to interpret the numerical values, color coding, and statistical presentations to draw meaningful conclusions about traffic management strategies.

## Study Design and Scope

### Experimental Framework

The analysis employed a controlled experimental design with EGO vehicles (vehicles programmed with specific driving behaviors) introduced into baseline traffic scenarios. The study measured both direct effects on EGO vehicle performance and indirect effects on surrounding traffic through a performance-based proximity analysis methodology.

### Traffic Scenarios Evaluated

**Primary Scenarios:**
- **Straight Highway Segments**: Controlled environment for measuring fundamental driving behavior impacts
- **Merge Zones**: Critical areas where lane-changing behaviors significantly affect traffic flow
- **Secondary Merge Areas**: Complex multi-stream convergence points testing advanced coordination behaviors

**Urban Scenario:**
- **Barrandov**: Large-scale urban environment with complex intersection patterns, analyzed separately due to distinct traffic characteristics

### Driving Behavior Parameters

The study systematically varied five key driving behavior dimensions:

1. **Following Distance Preference (tau)**: Time headway maintained behind lead vehicles
2. **Strategic Planning Horizon (lcStrategic)**: Advance planning for lane changes
3. **Cooperative Behavior (lcCooperative/lcCooperativeSpeed)**: Willingness to facilitate other vehicles' maneuvers
4. **Speed Optimization Seeking (lcSpeedGain/lcSpeedGainLookahead)**: Aggressiveness in pursuing speed advantages
5. **Gap Acceptance Behavior (lcPushy/lcPushyGap)**: Minimum acceptable gaps for lane changes

## Understanding the Results Structure

### Summary Tables Overview

The results are presented in four primary summary tables, each serving a specific analytical purpose:

1. **Total Metrics Summary**: Average performance across the three primary scenarios
2. **Barrandov Metrics Summary**: Urban scenario results analyzed separately
3. **Configuration Parameters Summary**: Decoding of configuration IDs to specific parameter values
4. **Individual Configuration Results**: Detailed analysis for each driving behavior combination

### Color-Coded Performance Indicators

All results tables employ a standardized color-coding system for immediate visual assessment:

**Green Shading**: Indicates beneficial changes relative to baseline
- Darker green represents larger beneficial effects
- Light green indicates moderate improvements

**Red Shading**: Indicates detrimental changes relative to baseline
- Darker red represents larger negative effects
- Light red indicates moderate degradations

**White/Unshaded**: Neutral values, count data, or non-applicable measurements

## Batch Analysis Directory Structure

### Output Organization

The batch analysis system generates a structured directory hierarchy containing comprehensive results for all 243 configurations. Understanding this organization is essential for navigating the results effectively.

### Primary Directory Structure

```
batch_analysis_results_YYYYMMDD_HHMMSS/
├── configs/                    # Individual configuration results
│   ├── 11111/                 # Results for configuration 11111
│   │   ├── ego_analysis_table.png      # Visual EGO/proximity comparison table
│   │   ├── ego_analysis_table.txt      # Text version of the table
│   │   └── ego_analysis.json           # Complete numerical results (all scenarios)
│   ├── 11112/                 # Results for configuration 11112
│   │   └── [same file structure]
│   └── [... 241 more configuration directories]
└── summary/                   # Aggregated analysis results
    ├── tables/               # Summary tables for all configurations
    │   ├── total_metrics_summary.png/txt        # Average of three main scenarios
    │   ├── barrandov_metrics_summary.png/txt    # Urban scenario summary
    │   ├── config_parameters_summary.png/txt    # Parameter decoding table
    │   └── ego_analysis_summary.png/txt         # Comprehensive results overview
    ├── json/                 # Machine-readable summary data
    │   └── batch_analysis_summary.json         # Complete batch results
    └── analysis_report.txt   # Human-readable analysis summary
```

### Individual Configuration Files

Each configuration directory (`configs/{config_id}/`) contains:

**Visual Outputs:**
- `ego_analysis_table.png`: Color-coded performance comparison table showing percentage changes for EGO vehicles and the impacted “proximity” set across scenarios

**Text Outputs:**
- `ego_analysis_table.txt`: Plain text version of the performance comparison table

**Data Files:**
- `ego_analysis.json`: Complete numerical results including all calculated metrics, statistical measures, and configuration parameters

### Summary Files

**Summary Tables** (`summary/tables/`):
- `total_metrics_summary.png/txt`: Consolidated view of all configurations showing average performance across straight, merge, and secondary merge scenarios
- `barrandov_metrics_summary.png/txt`: Urban scenario results for all configurations, analyzed separately due to distinct traffic characteristics
- `config_parameters_summary.png/txt`: Parameter decoding table showing the specific driving behavior values for each configuration ID
- `ego_analysis_summary.png/txt`: Comprehensive overview table combining all scenarios and configurations

**Machine-Readable Data** (`summary/json/`):
- `batch_analysis_summary.json`: Complete batch analysis results in structured JSON format, including success rates, configuration parameters, and all calculated metrics

**Analysis Report** (`summary/analysis_report.txt`):
- Comprehensive text summary including processing statistics, file organization guide, and analysis methodology overview

### Navigation Strategy

**For Quick Overview**: Start with `summary/tables/total_metrics_summary.png` to identify top-performing configurations across scenarios.

**For Detailed Analysis**: Examine individual configuration directories for specific behaviors of interest, focusing on the `ego_analysis_table.png` for visual assessment and `ego_analysis.json` for numerical details.

**For Urban Applications**: Use `summary/tables/barrandov_metrics_summary.png` to identify configurations optimized for complex urban environments.

**For Parameter Understanding**: Reference `summary/tables/config_parameters_summary.png` to decode configuration IDs into specific driving behavior parameters.

**For Automated Processing**: Utilize `summary/json/batch_analysis_summary.json` for programmatic analysis and further statistical processing.

### File Format Specifications

**PNG Files**: High-resolution (300 DPI) images suitable for publications and presentations, with color-coded performance indicators and professional formatting.

**TXT Files**: Plain text tables with consistent formatting, suitable for import into spreadsheet applications or further text-based analysis.

**JSON Files**: Structured data format containing complete numerical results, metadata, and configuration parameters for programmatic analysis.

This organized structure enables efficient navigation from high-level summary insights to detailed configuration-specific analysis, supporting both rapid decision-making and comprehensive research investigations.

### Notes on simulation outputs (inputs to analysis)

The analysis scripts expect SUMO simulation outputs to exist under:

- `trips_output/<scenario>/<config_id>.trips.xml`
- `ssm_output/<scenario>/<config_id>.ssm.xml`

The batch simulation runner (`analysis/batch_run.sh`) can generate these, and its output directories are configurable via environment variables (see `README.md` for examples).

## Interpreting Performance Metrics

### Traffic Efficiency Indicators

**EGO Duration Change (%)**: Percentage change in trip completion time for vehicles with modified behaviors
- *Negative values (green)*: Faster trip completion, indicating improved individual efficiency
- *Positive values (red)*: Slower trip completion, suggesting behavior creates delays
- *Interpretation*: Values beyond ±10% represent substantial efficiency impacts

**EGO Speed Change (%)**: Percentage change in average travel speed for modified vehicles
- *Positive values (green)*: Higher average speeds, indicating improved flow
- *Negative values (red)*: Reduced speeds, suggesting congestion or inefficiency
- *Interpretation*: Speed changes often inversely correlate with duration changes

**Proximity Vehicle Counts**: Number of vehicles significantly affected by EGO behavior
- *Higher counts*: Broader influence of the driving behavior on traffic network
- *Lower counts*: More localized effects
- *Interpretation*: Indicates the spatial extent of behavior propagation through traffic

### Safety Performance Indicators

**Conflicts Change**: Difference in safety-critical vehicle interactions
- *Negative values (green)*: Fewer dangerous interactions, improved safety
- *Positive values (red)*: More safety-critical situations
- *Interpretation*: Each unit represents one additional/fewer critical interaction

**Time to Collision (TTC) Change (%)**: Modification in collision risk timing
- *Positive values (green)*: Longer time available to avoid collisions, safer conditions
- *Negative values (red)*: Shorter collision avoidance time, increased risk
- *Interpretation*: Changes >±20% represent significant safety impacts

**Deceleration Rate to Avoid Collision (DRAC) Change (%)**: Change in required emergency braking intensity
- *Negative values (green)*: Less severe braking required, smoother traffic flow
- *Positive values (red)*: More aggressive braking needed, increased crash risk
- *Interpretation*: Values >±30% indicate substantial changes in collision severity potential

## Configuration Parameter Interpretation

### Parameter Encoding System

Each configuration is identified by a 5-digit code where each digit represents specific parameter settings:

| Position | Parameter Group | Value 1 | Value 2 | Value 3 |
|----------|-----------------|---------|---------|---------|
| 1 | Following Distance | 1s (Short) | 5s (Medium) | 10s (Long) |
| 2 | Strategic Planning | None (0) | Basic (1) | Advanced (5) |
| 3 | Cooperation Level | None (0) | Moderate (0.5) | Full (1.0) |
| 4 | Speed Optimization | None (0/1) | Moderate (1/5) | Aggressive (5/10) |
| 5 | Gap Acceptance | Large gaps (0/0.6) | Medium gaps (0.5/0.3) | Small gaps (1.0/0.1) |

### Behavioral Interpretation Examples

**Configuration 11111 (Conservative Driving)**:
- Short following distances with no strategic planning
- No cooperative behaviors or speed optimization
- Accepts only large gaps for lane changes

**Configuration 33333 (Optimized Driving)**:
- Long following distances with advanced planning
- Full cooperation and aggressive speed optimization  
- Accepts small gaps for frequent lane changes