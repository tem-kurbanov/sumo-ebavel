#!/usr/bin/env python3
"""
Batch Analysis Script for Traffic Simulation Configurations

This script runs EGO vehicle analysis for all configurations
and organizes the outputs in a structured directory hierarchy.
"""

import os
import sys
import glob
import time
import shutil
from pathlib import Path
from typing import List, Dict
import subprocess
import json
from datetime import datetime

# Import the analysis module
from traffic_analysis import BenchmarkComparator


class BatchAnalyzer:
    """Batch analyzer for processing all configurations"""
    
    def __init__(
        self,
        output_base_dir: str = "batch_analysis_results",
        configs_dir: str = "experiments/barrandov"
    ):
        """
        Args:
            output_base_dir: Base directory for analysis outputs.
            configs_dir: Directory containing .rou.xml configurations.
        """
        self.output_base_dir = output_base_dir
        self.configs_dir = configs_dir
        self.results_summary = {}
        self.failed_configs = []
        self.successful_configs = []
        
        # Create output directory structure
        self._create_output_structure()
    
    def _create_output_structure(self):
        """Create the output directory structure"""
        # Main directories
        dirs = [
            self.output_base_dir,
            f"{self.output_base_dir}/configs",
            f"{self.output_base_dir}/summary",
            f"{self.output_base_dir}/summary/tables",
            f"{self.output_base_dir}/summary/plots",
            f"{self.output_base_dir}/summary/json"
        ]
        
        for dir_path in dirs:
            Path(dir_path).mkdir(parents=True, exist_ok=True)
    
    def get_all_configurations(self) -> List[str]:
        """Get all configuration IDs from the experiments directory"""
        config_files = glob.glob(f"{self.configs_dir}/*.rou.xml")
        config_ids = []
        
        for file_path in config_files:
            filename = os.path.basename(file_path)
            config_id = filename.replace('.rou.xml', '')
            if config_id != 'base':  # Skip base configuration
                config_ids.append(config_id)
        
        # Sort configurations for consistent processing order
        config_ids.sort()
        return config_ids
    
    def analyze_configuration(self, config_id: str) -> Dict:
        """Analyze a single configuration"""
        print(f"\n{'='*80}")
        print(f"ANALYZING CONFIGURATION: {config_id}")
        print(f"{'='*80}")
        
        start_time = time.time()
        
        try:
            # Create configuration-specific output directory
            config_output_dir = f"{self.output_base_dir}/configs/{config_id}"
            Path(config_output_dir).mkdir(parents=True, exist_ok=True)
            
            # Define parameter groups for configuration parsing
            groups = []
            
            tau_dict = {"tau": ["1", "5", "10"]}
            groups.append(tau_dict)
            
            strat_dict = {"lcStrategic": ["0", "1", "5"]}
            groups.append(strat_dict)
            
            coop_dict = {"lcCooperative": ["0", "0.5", "1"], "lcCooperativeSpeed": ["0", "0.5", "1"]}
            groups.append(coop_dict)
            
            speed_dict = {"lcSpeedGain": ["0", "1", "5"], "lcSpeedGainLookahead": ["1", "5", "10"]}
            groups.append(speed_dict)
            
            push_dict = {"lcPushy": ["0", "0.5", "1"], "lcPushyGap": ["0.6", "0.3", "0.1"]}
            groups.append(push_dict)
            
            # Run the analysis
            comparator = BenchmarkComparator(config_id, groups)
            
            # Run EGO analysis
            success = comparator.run_comparison(
                create_plots=True,
                save_results=True,
                include_ego_analysis=True
            )
            
            if success:
                # Move outputs to configuration-specific directory
                self._organize_config_outputs(config_id, config_output_dir)
                
                # Collect results for summary
                results = self._collect_config_results(config_id, comparator)
                
                elapsed_time = time.time() - start_time
                print(f"\n✓ Configuration {config_id} completed successfully in {elapsed_time:.2f}s")
                
                self.successful_configs.append(config_id)
                return results
            else:
                print(f"\n✗ Configuration {config_id} failed to complete")
                self.failed_configs.append(config_id)
                return None
                
        except Exception as e:
            elapsed_time = time.time() - start_time
            print(f"\n✗ Configuration {config_id} failed with error: {str(e)}")
            print(f"Error occurred after {elapsed_time:.2f}s")
            self.failed_configs.append(config_id)
            return None
    
    def _organize_config_outputs(self, config_id: str, config_output_dir: str):
        """Organize outputs for a specific configuration"""
        # Move EGO analysis outputs
        ego_sources = [
            f"comparison_output/ego_analysis_table.png",
            f"comparison_output/ego_analysis_table.txt"
        ]
        
        ego_destinations = [
            f"{config_output_dir}/ego_analysis_table.png",
            f"{config_output_dir}/ego_analysis_table.txt"
        ]
        
        # Move files
        for src, dst in zip(ego_sources, ego_destinations):
            if os.path.exists(src):
                shutil.move(src, dst)
        
        # Move JSON results
        json_sources = [
            f"ego_analysis_{config_id}.json"
        ]
        
        json_destinations = [
            f"{config_output_dir}/ego_analysis.json"
        ]
        
        for src, dst in zip(json_sources, json_destinations):
            if os.path.exists(src):
                shutil.move(src, dst)
    
    def _collect_config_results(self, config_id: str, comparator: BenchmarkComparator) -> Dict:
        """Collect key results from a configuration analysis"""
        try:
            # Load EGO analysis results
            ego_file = f"{self.output_base_dir}/configs/{config_id}/ego_analysis.json"
            
            results = {
                'config_id': config_id,
                'config_params': comparator.config_values,
                'timestamp': datetime.now().isoformat()
            }
            
            if os.path.exists(ego_file):
                with open(ego_file, 'r') as f:
                    ego_data = json.load(f)
                    results['ego_analysis'] = ego_data
            
            return results
            
        except Exception as e:
            print(f"Warning: Could not collect results for {config_id}: {str(e)}")
            return {'config_id': config_id, 'error': str(e)}
    
    def create_summary_tables(self):
        """Create summary tables for all configurations"""
        print(f"\n{'='*80}")
        print("CREATING SUMMARY TABLES")
        print(f"{'='*80}")
        
        # Create EGO analysis summary table
        self._create_ego_summary_table()
        
        # Create configuration parameters summary
        self._create_config_params_summary()
        
        # Create metric summary tables
        self._create_total_metrics_summary()
        self._create_barrandov_metrics_summary()
    

    
    def _create_ego_summary_table(self):
        """Create a summary table of EGO analysis results"""
        import pandas as pd
        
        # Collect data from all successful configurations
        summary_data = []
        
        for config_id in self.successful_configs:
            config_file = f"{self.output_base_dir}/configs/{config_id}/ego_analysis.json"
            if os.path.exists(config_file):
                try:
                    with open(config_file, 'r') as f:
                        data = json.load(f)
                    
                    # Extract EGO metrics from each scenario
                    scenarios = ['straight', 'merge', 'secondary_merge']
                    for scenario in scenarios:
                        if scenario in data:
                            ego_data = data[scenario].get('ego_comparison', {})
                            proximity_data = data[scenario].get('proximity_analysis', {})
                            
                            summary_data.append({
                                'Config_ID': config_id,
                                'Scenario': scenario.upper(),
                                'EGO_Count': ego_data.get('num_ego_vehicles', 0),
                                'EGO_Duration_Diff_%': ego_data.get('duration_diff_pct', float('nan')),
                                'EGO_Speed_Diff_%': ego_data.get('speed_diff_pct', float('nan')),
                                'Proximity_Vehicles': proximity_data.get('num_proximity_vehicles', 0),
                                'Proximity_Duration_Diff_%': proximity_data.get('duration_diff_pct', float('nan')),
                                'Proximity_Speed_Diff_%': proximity_data.get('speed_diff_pct', float('nan')),
                                'Proximity_BR_Violations_Diff': proximity_data.get('br_violations_diff', 0),
                                'Proximity_SGAP_Violations_Diff': proximity_data.get('sgap_violations_diff', 0)
                            })
                except Exception as e:
                    print(f"Warning: Could not process EGO data for {config_id}: {str(e)}")
        
        if summary_data:
            df = pd.DataFrame(summary_data)
            
            # Format numeric columns with appropriate decimal places
            for col in df.columns:
                if 'Diff_%' in col:
                    df[col] = df[col].apply(lambda x: f"{x:+.1f}" if pd.notna(x) and x != float('inf') and x != float('-inf') else "N/A")
                elif 'Violations_Diff' in col:
                    df[col] = df[col].apply(lambda x: f"{x:+.0f}" if pd.notna(x) else "0")
                elif col in ['EGO_Count', 'Proximity_Vehicles']:
                    df[col] = df[col].apply(lambda x: f"{x:.0f}" if pd.notna(x) else "0")
            
            # Save as text table
            txt_file = f"{self.output_base_dir}/summary/tables/ego_analysis_summary.txt"
            with open(txt_file, 'w') as f:
                f.write("EGO ANALYSIS SUMMARY TABLE\n")
                f.write("=" * 120 + "\n\n")
                f.write("Note: All differences are (Config - Benchmark)\n\n")
                f.write(df.to_string(index=False))
            
            # Create visual table
            self._create_visual_table(df, "EGO Analysis Summary - Batch Results", 
                                    f"{self.output_base_dir}/summary/tables/ego_analysis_summary.png")
    
    def _create_config_params_summary(self):
        """Create a summary table of configuration parameters"""
        import pandas as pd
        
        # Collect parameter data from all successful configurations
        summary_data = []
        
        for config_id in self.successful_configs:
            config_file = f"{self.output_base_dir}/configs/{config_id}/ego_analysis.json"
            if os.path.exists(config_file):
                try:
                    with open(config_file, 'r') as f:
                        data = json.load(f)
                    
                    config_params = data.get('config_values', {})
                    summary_data.append({
                        'Config_ID': config_id,
                        'tau': config_params.get('tau', 'N/A'),
                        'lcStrategic': config_params.get('lcStrategic', 'N/A'),
                        'lcCooperative': config_params.get('lcCooperative', 'N/A'),
                        'lcCooperativeSpeed': config_params.get('lcCooperativeSpeed', 'N/A'),
                        'lcSpeedGain': config_params.get('lcSpeedGain', 'N/A'),
                        'lcSpeedGainLookahead': config_params.get('lcSpeedGainLookahead', 'N/A'),
                        'lcPushy': config_params.get('lcPushy', 'N/A'),
                        'lcPushyGap': config_params.get('lcPushyGap', 'N/A')
                    })
                except Exception as e:
                    print(f"Warning: Could not process config params for {config_id}: {str(e)}")
        
        if summary_data:
            df = pd.DataFrame(summary_data)
            
            # Save as text table
            txt_file = f"{self.output_base_dir}/summary/tables/config_parameters_summary.txt"
            with open(txt_file, 'w') as f:
                f.write("CONFIGURATION PARAMETERS SUMMARY\n")
                f.write("=" * 80 + "\n\n")
                f.write(df.to_string(index=False))
            
            # Create visual table
            self._create_visual_table(df, "Configuration Parameters - Batch Results", 
                                    f"{self.output_base_dir}/summary/tables/config_parameters_summary.png")
    
    def _create_total_metrics_summary(self):
        """Create a summary table of TOTAL (average of three main scenarios) metrics"""
        import pandas as pd
        
        # Collect data from all successful configurations
        summary_data = []
        
        for config_id in self.successful_configs:
            config_file = f"{self.output_base_dir}/configs/{config_id}/ego_analysis.json"
            if os.path.exists(config_file):
                try:
                    with open(config_file, 'r') as f:
                        data = json.load(f)
                    
                    # Get TOTAL scenario data (if it exists)
                    scenarios = data.get('scenarios', {})
                    if 'TOTAL' in scenarios:
                        total_data = scenarios['TOTAL']
                        ego_data = total_data.get('ego_comparison', {})
                        proximity_data = total_data.get('proximity_analysis', {})
                        
                        summary_data.append({
                            'Config_ID': config_id,
                            'EGO_Duration_Change_%': ego_data.get('duration_diff_pct', float('nan')),
                            'EGO_Speed_Change_%': ego_data.get('speed_diff_pct', float('nan')),
                            'Proximity_Vehicles': proximity_data.get('num_proximity_vehicles', 0),
                            'Proximity_Duration_Change_%': proximity_data.get('duration_diff_pct', float('nan')),
                            'Proximity_Speed_Change_%': proximity_data.get('speed_diff_pct', float('nan')),
                            'Conflicts_Change': proximity_data.get('conflicts_diff', 0),
                            'TTC_Change_%': self._calculate_ttc_change_pct(proximity_data),
                            'DRAC_Change_%': self._calculate_drac_change_pct(proximity_data)
                        })
                except Exception as e:
                    print(f"Warning: Could not process TOTAL data for {config_id}: {str(e)}")
        
        if summary_data:
            df = pd.DataFrame(summary_data)
            
            # Format numeric columns with appropriate decimal places
            for col in df.columns:
                if 'Change_%' in col:
                    df[col] = df[col].apply(lambda x: f"{x:+.1f}" if pd.notna(x) and x != float('inf') and x != float('-inf') else "N/A")
                elif col == 'Conflicts_Change':
                    df[col] = df[col].apply(lambda x: f"{x:+.0f}" if pd.notna(x) else "0")
                elif col == 'Proximity_Vehicles':
                    df[col] = df[col].apply(lambda x: f"{x:.0f}" if pd.notna(x) else "0")
            
            # Save as text table
            txt_file = f"{self.output_base_dir}/summary/tables/total_metrics_summary.txt"
            with open(txt_file, 'w') as f:
                f.write("TOTAL METRICS SUMMARY TABLE (Average of Three Main Scenarios)\n")
                f.write("=" * 100 + "\n\n")
                f.write("Note: All changes are percentage differences (Config - Benchmark)\n\n")
                f.write(df.to_string(index=False))
            
            # Create color-coded visual table
            self._create_metrics_visual_table(df, "Total Metrics Summary - Average of Three Main Scenarios", 
                                            f"{self.output_base_dir}/summary/tables/total_metrics_summary.png")
    
    def _create_barrandov_metrics_summary(self):
        """Create a summary table of Barrandov scenario metrics"""
        import pandas as pd
        
        # Collect data from all successful configurations
        summary_data = []
        
        for config_id in self.successful_configs:
            config_file = f"{self.output_base_dir}/configs/{config_id}/ego_analysis.json"
            if os.path.exists(config_file):
                try:
                    with open(config_file, 'r') as f:
                        data = json.load(f)
                    
                    # Get BARRANDOV scenario data (if it exists)
                    scenarios = data.get('scenarios', {})
                    if 'BARRANDOV' in scenarios:
                        barrandov_data = scenarios['BARRANDOV']
                        ego_data = barrandov_data.get('ego_comparison', {})
                        proximity_data = barrandov_data.get('proximity_analysis', {})
                        
                        summary_data.append({
                            'Config_ID': config_id,
                            'EGO_Duration_Change_%': ego_data.get('duration_diff_pct', float('nan')),
                            'EGO_Speed_Change_%': ego_data.get('speed_diff_pct', float('nan')),
                            'Proximity_Vehicles': proximity_data.get('num_proximity_vehicles', 0),
                            'Proximity_Duration_Change_%': proximity_data.get('duration_diff_pct', float('nan')),
                            'Proximity_Speed_Change_%': proximity_data.get('speed_diff_pct', float('nan')),
                            'Conflicts_Change': proximity_data.get('conflicts_diff', 0),
                            'TTC_Change_%': self._calculate_ttc_change_pct(proximity_data),
                            'DRAC_Change_%': self._calculate_drac_change_pct(proximity_data)
                        })
                except Exception as e:
                    print(f"Warning: Could not process BARRANDOV data for {config_id}: {str(e)}")
        
        if summary_data:
            df = pd.DataFrame(summary_data)
            
            # Format numeric columns with appropriate decimal places
            for col in df.columns:
                if 'Change_%' in col:
                    df[col] = df[col].apply(lambda x: f"{x:+.1f}" if pd.notna(x) and x != float('inf') and x != float('-inf') else "N/A")
                elif col == 'Conflicts_Change':
                    df[col] = df[col].apply(lambda x: f"{x:+.0f}" if pd.notna(x) else "0")
                elif col == 'Proximity_Vehicles':
                    df[col] = df[col].apply(lambda x: f"{x:.0f}" if pd.notna(x) else "0")
            
            # Save as text table
            txt_file = f"{self.output_base_dir}/summary/tables/barrandov_metrics_summary.txt"
            with open(txt_file, 'w') as f:
                f.write("BARRANDOV METRICS SUMMARY TABLE\n")
                f.write("=" * 80 + "\n\n")
                f.write("Note: All changes are percentage differences (Config - Benchmark)\n\n")
                f.write(df.to_string(index=False))
            
            # Create color-coded visual table
            self._create_metrics_visual_table(df, "Barrandov Metrics Summary", 
                                            f"{self.output_base_dir}/summary/tables/barrandov_metrics_summary.png")
    
    def _calculate_ttc_change_pct(self, proximity_data):
        """Calculate TTC percentage change, handling infinite values"""
        benchmark_ttc = proximity_data.get('benchmark_min_ttc', float('inf'))
        config_ttc = proximity_data.get('config_min_ttc', float('inf'))
        
        if benchmark_ttc == float('inf') and config_ttc == float('inf'):
            return 0.0  # No change if both are infinite
        elif benchmark_ttc == float('inf'):
            return float('nan')  # Can't calculate percentage from infinite
        elif benchmark_ttc == 0:
            return float('nan')  # Can't divide by zero
        else:
            return ((config_ttc - benchmark_ttc) / benchmark_ttc) * 100
    
    def _calculate_drac_change_pct(self, proximity_data):
        """Calculate DRAC percentage change"""
        benchmark_drac = proximity_data.get('benchmark_max_drac', 0.0)
        config_drac = proximity_data.get('config_max_drac', 0.0)
        
        if benchmark_drac == 0:
            if config_drac == 0:
                return 0.0  # No change if both are zero
            else:
                return float('nan')  # Can't calculate percentage from zero
        else:
            return ((config_drac - benchmark_drac) / benchmark_drac) * 100
    
    def _create_visual_table(self, df, title: str, output_file: str):
        """Create a visual table using matplotlib"""
        import matplotlib.pyplot as plt
        import matplotlib.patches as patches
        
        # Create figure and axis
        fig, ax = plt.subplots(figsize=(16, max(8, len(df) * 0.4 + 2)))
        ax.axis('tight')
        ax.axis('off')
        
        # Create table
        table = ax.table(cellText=df.values, colLabels=df.columns, 
                        cellLoc='center', loc='center')
        
        # Style the table
        table.auto_set_font_size(False)
        table.set_fontsize(9)
        table.scale(1.2, 1.5)
        
        # Style header
        for i in range(len(df.columns)):
            table[(0, i)].set_facecolor('#4CAF50')
            table[(0, i)].set_text_props(weight='bold', color='white')
        
        # Helper function to get color based on value and metric type
        def get_color_for_change(value_str, column_name):
            """Get color based on value and whether decrease/increase is good"""
            if pd.isna(value_str) or str(value_str) == "N/A" or str(value_str) == "0":
                return '#ffffff'  # White for N/A or zero
            
            try:
                # Extract numeric value
                value_str = str(value_str)
                if '%' in value_str:
                    value = float(value_str.replace('%', '').replace('+', ''))
                else:
                    value = float(value_str.replace('+', ''))
                
                # Determine if change is good or bad based on column name
                is_good = False
                if 'Duration_Diff' in column_name:
                    # Duration: decrease is good (negative is good)
                    is_good = value < 0
                elif 'Speed_Diff' in column_name:
                    # Speed: increase is good (positive is good)
                    is_good = value > 0
                elif 'Violations_Diff' in column_name:
                    # Violations: decrease is good (negative is good)
                    is_good = value < 0
                
                # Calculate intensity based on absolute value
                abs_value = abs(value)
                if 'Duration_Diff' in column_name or 'Speed_Diff' in column_name:
                    # For percentage changes, cap at 50% for full saturation
                    intensity = min(abs_value / 50.0, 1.0)
                else:  # violations
                    # For violation changes, cap at 10 for full saturation
                    intensity = min(abs_value / 10.0, 1.0)
                
                # Generate color
                if is_good:
                    # Green with varying intensity (light green to dark green)
                    # Base green: #c8e6c9 (light) to #4caf50 (dark)
                    red_component = int(200 - 120 * intensity)  # 200 to 80
                    green_component = int(230 - 55 * intensity)  # 230 to 175
                    blue_component = int(201 - 121 * intensity)  # 201 to 80
                    return f'#{red_component:02x}{green_component:02x}{blue_component:02x}'
                else:
                    # Red with varying intensity (light red to dark red)
                    # Base red: #ffcdd2 (light) to #f44336 (dark)
                    red_component = int(255 - 11 * intensity)  # 255 to 244
                    green_component = int(205 - 138 * intensity)  # 205 to 67
                    blue_component = int(210 - 156 * intensity)  # 210 to 54
                    return f'#{red_component:02x}{green_component:02x}{blue_component:02x}'
                    
            except (ValueError, AttributeError):
                return '#ffffff'  # Default white
        
        # Color data cells based on values
        for i in range(1, len(df) + 1):
            for j in range(len(df.columns)):
                column_name = df.columns[j]
                if j <= 1:  # Config_ID and Scenario columns - no coloring
                    pass  # Keep default white background
                elif column_name in ['Proximity_Vehicles', 'EGO_Count']:  # Count columns (no coloring)
                    pass  # Keep default white background
                elif 'Diff' in column_name:  # Metric columns
                    cell_value = df.iloc[i-1, j]
                    color = get_color_for_change(cell_value, column_name)
                    table[(i, j)].set_facecolor(color)
                else:  # Other columns - no coloring
                    pass  # Keep default white background
        
        # Add title
        plt.title(title, fontsize=16, fontweight='bold', pad=20)
        
        # Save the table
        plt.tight_layout()
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        plt.close()
    
    def _create_metrics_visual_table(self, df, title: str, output_file: str):
        """Create a color-coded visual table for metrics using matplotlib"""
        import matplotlib.pyplot as plt
        import pandas as pd
        
        # Create figure and axis
        fig, ax = plt.subplots(figsize=(16, max(8, len(df) * 0.4 + 2)))
        ax.axis('tight')
        ax.axis('off')
        
        # Create table
        table = ax.table(cellText=df.values, colLabels=df.columns, 
                        cellLoc='center', loc='center')
        
        # Style the table
        table.auto_set_font_size(False)
        table.set_fontsize(9)
        table.scale(1.2, 1.5)
        
        # Style header
        for i in range(len(df.columns)):
            table[(0, i)].set_facecolor('#4CAF50')
            table[(0, i)].set_text_props(weight='bold', color='white')
        
        # Helper function to get color based on value and metric type
        def get_color_for_change(value_str, column_name):
            """Get color based on value and whether decrease/increase is good"""
            if pd.isna(value_str) or str(value_str) == "N/A" or str(value_str) == "0" or str(value_str) == "0.0":
                return '#ffffff'  # White for N/A or zero
            
            try:
                # Extract numeric value from formatted string
                value_str = str(value_str).strip()
                if '%' in value_str:
                    value = float(value_str.replace('%', '').replace('+', ''))
                else:
                    value = float(value_str.replace('+', ''))
                
                # Skip coloring if value is effectively zero
                if abs(value) < 0.01:
                    return '#ffffff'
                
                # Determine if change is good or bad based on column name
                is_good = False
                if 'Duration_Change' in column_name:
                    # Duration: decrease is good (negative is good)
                    is_good = value < 0
                elif 'Speed_Change' in column_name:
                    # Speed: increase is good (positive is good)
                    is_good = value > 0
                elif 'Conflicts_Change' in column_name:
                    # Conflicts: decrease is good (negative is good)
                    is_good = value < 0
                elif 'TTC_Change' in column_name:
                    # TTC: increase is good (positive is good) - more time to collision is safer
                    is_good = value > 0
                elif 'DRAC_Change' in column_name:
                    # DRAC: decrease is good (negative is good) - less severe braking needed
                    is_good = value < 0
                
                # Calculate intensity based on absolute value
                abs_value = abs(value)
                if 'Change_%' in column_name:
                    # For percentage changes, cap at 50% for full saturation
                    intensity = min(abs_value / 50.0, 1.0)
                else:  # conflicts
                    # For conflict changes, cap at 10 for full saturation
                    intensity = min(abs_value / 10.0, 1.0)
                
                # Generate color
                if is_good:
                    # Green with varying intensity (light green to dark green)
                    # Base green: #c8e6c9 (light) to #4caf50 (dark)
                    red_component = int(200 - 120 * intensity)  # 200 to 80
                    green_component = int(230 - 55 * intensity)  # 230 to 175
                    blue_component = int(201 - 121 * intensity)  # 201 to 80
                    return f'#{red_component:02x}{green_component:02x}{blue_component:02x}'
                else:
                    # Red with varying intensity (light red to dark red)
                    # Base red: #ffcdd2 (light) to #f44336 (dark)
                    red_component = int(255 - 11 * intensity)  # 255 to 244
                    green_component = int(205 - 138 * intensity)  # 205 to 67
                    blue_component = int(210 - 156 * intensity)  # 210 to 54
                    return f'#{red_component:02x}{green_component:02x}{blue_component:02x}'
                    
            except (ValueError, AttributeError):
                return '#ffffff'  # Default white
        
        # Color data cells based on values
        for i in range(1, len(df) + 1):
            for j in range(len(df.columns)):
                column_name = df.columns[j]
                if j == 0:  # Config_ID column - no coloring
                    pass  # Keep default white background
                elif column_name == 'Proximity_Vehicles':  # Proximity_Vehicles column (no coloring)
                    pass  # Keep default white background
                else:  # Metric columns
                    cell_value = df.iloc[i-1, j]
                    color = get_color_for_change(cell_value, column_name)
                    table[(i, j)].set_facecolor(color)
        
        # Add title
        plt.title(title, fontsize=16, fontweight='bold', pad=20)
        
        # Save the table
        plt.tight_layout()
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        plt.close()
    
    def create_summary_json(self):
        """Create a comprehensive JSON summary of all results"""
        summary = {
            'analysis_info': {
                'timestamp': datetime.now().isoformat(),
                'total_configurations': len(self.successful_configs) + len(self.failed_configs),
                'successful_configurations': len(self.successful_configs),
                'failed_configurations': len(self.failed_configs),
                'success_rate': f"{len(self.successful_configs) / (len(self.successful_configs) + len(self.failed_configs)) * 100:.1f}%"
            },
            'successful_configs': self.successful_configs,
            'failed_configs': self.failed_configs,
            'results': self.results_summary
        }
        
        # Save summary JSON
        summary_file = f"{self.output_base_dir}/summary/json/batch_analysis_summary.json"
        with open(summary_file, 'w') as f:
            json.dump(summary, f, indent=2)
        
        print(f"Summary JSON saved to: {summary_file}")
    
    def create_analysis_report(self):
        """Create a comprehensive analysis report"""
        report_file = f"{self.output_base_dir}/summary/analysis_report.txt"
        
        with open(report_file, 'w') as f:
            f.write("BATCH TRAFFIC ANALYSIS REPORT\n")
            f.write("=" * 80 + "\n\n")
            
            f.write(f"Analysis completed: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")
            
            f.write("SUMMARY STATISTICS:\n")
            f.write("-" * 20 + "\n")
            f.write(f"Total configurations processed: {len(self.successful_configs) + len(self.failed_configs)}\n")
            f.write(f"Successful analyses: {len(self.successful_configs)}\n")
            f.write(f"Failed analyses: {len(self.failed_configs)}\n")
            f.write(f"Success rate: {len(self.successful_configs) / (len(self.successful_configs) + len(self.failed_configs)) * 100:.1f}%\n\n")
            
            if self.failed_configs:
                f.write("FAILED CONFIGURATIONS:\n")
                f.write("-" * 25 + "\n")
                for config_id in self.failed_configs:
                    f.write(f"  {config_id}\n")
                f.write("\n")
            
            f.write("OUTPUT ORGANIZATION:\n")
            f.write("-" * 20 + "\n")
            f.write(f"Base directory: {self.output_base_dir}/\n")
            f.write("  ├── configs/           # Individual configuration results\n")
            f.write("  │   └── {config_id}/   # Results for each configuration\n")
            f.write("  │       ├── main_comparison_metrics.png/txt\n")
            f.write("  │       ├── ego_analysis_table.png/txt\n")
            f.write("  │       ├── barrandov_comparison.png\n")
            f.write("  │       ├── comparison_plots.png\n")
            f.write("  │       ├── comparison_results.json\n")
            f.write("  │       └── ego_analysis.json\n")
            f.write("  └── summary/           # Summary tables and reports\n")
            f.write("      ├── tables/        # Summary tables (PNG + TXT)\n")
            f.write("      ├── plots/         # Summary plots\n")
            f.write("      ├── json/          # Summary JSON files\n")
            f.write("      └── analysis_report.txt\n\n")
            
            f.write("ANALYSIS FEATURES:\n")
            f.write("-" * 18 + "\n")
            f.write("  - EGO vehicle analysis with proximity impact assessment\n")
            f.write("  - SSM (Surrogate Safety Measures) metrics analysis\n")
            f.write("  - Braking Rate, Space Gap, and Time Gap threshold violations\n")
            f.write("  - Performance-based proximity impact analysis\n")
            f.write("  - Comprehensive metrics across all scenarios (straight, merge, secondary_merge, Barrandov)\n")
            f.write("  - Visual outputs in both text and PNG formats\n\n")
            
            f.write("FILES GENERATED:\n")
            f.write("-" * 15 + "\n")
            f.write("Summary Tables:\n")
            f.write("  - ego_analysis_summary.txt/png\n")
            f.write("  - config_parameters_summary.txt/png\n")
            f.write("  - total_metrics_summary.txt/png (Average of three main scenarios)\n")
            f.write("  - barrandov_metrics_summary.txt/png (Barrandov scenario only)\n")
            f.write("Summary Files:\n")
            f.write("  - batch_analysis_summary.json\n")
            f.write("  - analysis_report.txt\n")
        
        print(f"Analysis report saved to: {report_file}")
    
    def run_batch_analysis(self, config_ids: List[str] = None, max_configs: int = None):
        """Run batch analysis for all configurations"""
        print(f"{'='*80}")
        print("BATCH TRAFFIC ANALYSIS")
        print(f"{'='*80}")
        print(f"Output directory: {self.output_base_dir}")
        
        # Get configurations to analyze
        if config_ids is None:
            all_configs = self.get_all_configurations()
        else:
            all_configs = config_ids
        
        if max_configs:
            all_configs = all_configs[:max_configs]
        
        print(f"Found {len(all_configs)} configurations to analyze")
        
        # Process each configuration
        start_time = time.time()
        
        for i, config_id in enumerate(all_configs, 1):
            print(f"\nProgress: {i}/{len(all_configs)} ({i/len(all_configs)*100:.1f}%)")
            
            results = self.analyze_configuration(config_id)
            if results:
                self.results_summary[config_id] = results
        
        total_time = time.time() - start_time
        
        # Create summaries
        print(f"\n{'='*80}")
        print("CREATING SUMMARY REPORTS")
        print(f"{'='*80}")
        
        self.create_summary_tables()
        self.create_summary_json()
        self.create_analysis_report()
        
        # Final summary
        print(f"\n{'='*80}")
        print("BATCH ANALYSIS COMPLETED")
        print(f"{'='*80}")
        print(f"Total time: {total_time:.2f}s ({total_time/60:.1f} minutes)")
        print(f"Average time per config: {total_time/len(all_configs):.2f}s")
        print(f"Successful: {len(self.successful_configs)}")
        print(f"Failed: {len(self.failed_configs)}")
        print(f"Success rate: {len(self.successful_configs)/len(all_configs)*100:.1f}%")
        print(f"\nResults saved to: {self.output_base_dir}/")
        
        if self.failed_configs:
            print(f"\nFailed configurations: {', '.join(self.failed_configs)}")


def main():
    """Main function to run batch analysis"""
    import argparse
    
    parser = argparse.ArgumentParser(description="Batch Traffic Analysis")
    parser.add_argument("--configs", nargs="+", help="Specific configuration IDs to analyze")
    parser.add_argument("--max", type=int, help="Maximum number of configurations to analyze")
    parser.add_argument("--output", default="batch_analysis_results", help="Output directory")
    parser.add_argument(
        "--configs-dir",
        default="experiments/barrandov",
        help="Directory containing .rou.xml configurations"
    )
    
    args = parser.parse_args()
    
    # Create batch analyzer
    analyzer = BatchAnalyzer(output_base_dir=args.output, configs_dir=args.configs_dir)
    
    # Run analysis
    analyzer.run_batch_analysis(
        config_ids=args.configs,
        max_configs=args.max
    )


if __name__ == "__main__":
    main() 