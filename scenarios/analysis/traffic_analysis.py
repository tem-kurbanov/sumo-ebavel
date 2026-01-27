#!/usr/bin/env python3
"""
Traffic Simulation Benchmark Comparison Script
Compares a given configuration against benchmark outputs for straight, merge, and secondary_merge scenarios.
Focuses on average trip durations, average speeds, and SSM (Surrogate Safety Measures) metrics.
"""

import xml.etree.ElementTree as ET
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import re
import os
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass
from pathlib import Path
import argparse
from scipy.stats import trim_mean

# Try to import seaborn for better plotting, but make it optional
try:
    import seaborn as sns
    sns.set_palette("husl")
    plt.style.use('seaborn-v0_8')
except ImportError:
    print("Warning: seaborn not available, using default matplotlib style")
    plt.style.use('default')

@dataclass
class SSMMetrics:
    """Data class to store SSM (Surrogate Safety Measures) metrics"""
    total_conflicts: int  # Total number of conflicts/interactions
    avg_min_ttc: float  # Average minimum Time to Collision
    avg_max_drac: float  # Average maximum Deceleration Rate to Avoid Collision
    avg_pet: float  # Average Post Encroachment Time (excluding NA values)

@dataclass
class BenchmarkMetrics:
    """Data class to store benchmark metrics"""
    avg_duration: float
    avg_speed: float
    ssm_metrics: SSMMetrics
    total_vehicles: int

@dataclass
class ConfigMetrics:
    """Data class to store configuration metrics"""
    avg_duration: float
    avg_speed: float
    ssm_metrics: SSMMetrics
    total_vehicles: int

class BenchmarkComparator:
    """Main class for comparing traffic simulation configurations against benchmarks"""
    
    def __init__(self, config_id: str, groups: List[Dict] = None):
        """
        Initialize the comparator
        
        Args:
            config_id: Configuration ID to compare against benchmarks (e.g., '33333')
            groups: List of parameter groups for parsing configuration values
        """
        self.config_id = config_id
        self.scenarios = ['straight', 'merge', 'secondary_merge']
        self.barrandov_scenario = 'reset'  # Barrandov scenario uses 'reset' folder
        self.groups = groups or []
        
        # Storage for benchmark and config data
        self.benchmarks: Dict[str, BenchmarkMetrics] = {}
        self.config_metrics: Dict[str, ConfigMetrics] = {}
        
        # Barrandov scenario data (separate from main scenarios)
        self.barrandov_benchmark: Optional[BenchmarkMetrics] = None
        self.barrandov_config: Optional[ConfigMetrics] = None
        
        # Comparison results
        self.comparison_results = {}
        
        # Parse configuration values
        self.config_values = self._parse_config_values()
        
    def _parse_config_values(self) -> Dict[str, str]:
        """Parse configuration ID to extract parameter values"""
        if len(self.config_id) != 5 or not self.groups:
            return {}
        
        # Define the parameter mapping based on the groups structure
        # Each digit corresponds to specific parameters
        param_mapping = [
            ("tau", ["1", "5", "10"]),
            ("lcStrategic", ["0", "1", "5"]),
            ("lcCooperative", ["0", "0.5", "1"]),
            ("lcCooperativeSpeed", ["0", "0.5", "1"]),
            ("lcSpeedGain", ["0", "1", "5"]),
            ("lcSpeedGainLookahead", ["1", "5", "10"]),
            ("lcPushy", ["0", "0.5", "1"]),
            ("lcPushyGap", ["0.6", "0.3", "0.1"])
        ]
        
        config_values = {}
        
        # Map each digit to its corresponding parameter(s)
        # Since we have 8 parameters but only 5 digits, we need to map them correctly
        digit_to_params = [
            [0],  # digit 1 -> tau
            [1],  # digit 2 -> lcStrategic  
            [2, 3],  # digit 3 -> lcCooperative, lcCooperativeSpeed
            [4, 5],  # digit 4 -> lcSpeedGain, lcSpeedGainLookahead
            [6, 7]   # digit 5 -> lcPushy, lcPushyGap
        ]
        
        for digit_idx, param_indices in enumerate(digit_to_params):
            if digit_idx < len(self.config_id):
                value_index = int(self.config_id[digit_idx]) - 1
                for param_idx in param_indices:
                    if param_idx < len(param_mapping):
                        param_name, param_values = param_mapping[param_idx]
                        if 0 <= value_index < len(param_values):
                            config_values[param_name] = param_values[value_index]
        
        return config_values
    
    def _load_ssm_data(self, file_path: str) -> Optional[SSMMetrics]:
        """Load and parse SSM data from XML file"""
        try:
            if not os.path.exists(file_path):
                print(f"    Warning: SSM file not found: {file_path}")
                return None
            
            tree = ET.parse(file_path)
            root = tree.getroot()
            
            # Initialize metrics
            min_ttc_values = []
            max_drac_values = []
            pet_values = []
            total_conflicts = 0
            
            # Parse conflict elements for each interaction
            for conflict in root.findall('.//conflict'):
                total_conflicts += 1
                
                # Extract minTTC (minimum Time to Collision)
                min_ttc_elem = conflict.find('minTTC')
                if min_ttc_elem is not None:
                    try:
                        min_ttc_value = min_ttc_elem.get('value', 'inf')
                        if min_ttc_value.upper() not in ['NA', 'INF']:
                            min_ttc = float(min_ttc_value)
                            if min_ttc != float('inf'):  # Only include valid TTC values
                                min_ttc_values.append(min_ttc)
                    except ValueError:
                        pass  # Skip invalid minTTC values
                
                # Extract maxDRAC (maximum Deceleration Rate to Avoid Collision)
                max_drac_elem = conflict.find('maxDRAC')
                if max_drac_elem is not None:
                    try:
                        max_drac_value = max_drac_elem.get('value', '0')
                        if max_drac_value.upper() != 'NA':
                            max_drac = float(max_drac_value)
                            if max_drac > 0:  # Only include positive DRAC values
                                # Cap unrealistic DRAC values (max ~15 m/s² is realistic for emergency braking)
                                max_drac = min(max_drac, 15.0)
                                max_drac_values.append(max_drac)
                    except ValueError:
                        pass  # Skip invalid maxDRAC values
                
                # Extract PET (Post Encroachment Time)
                pet_elem = conflict.find('PET')
                if pet_elem is not None:
                    pet_value = pet_elem.get('value', 'NA')
                    if pet_value != 'NA' and pet_value != '':
                        try:
                            pet = float(pet_value)
                            pet_values.append(pet)
                        except ValueError:
                            pass  # Skip invalid PET values
            
            # Calculate average metrics
            avg_min_ttc = np.mean(min_ttc_values) if min_ttc_values else float('inf')
            avg_max_drac = np.mean(max_drac_values) if max_drac_values else 0.0
            avg_pet = np.mean(pet_values) if pet_values else float('nan')
            
            return SSMMetrics(
                total_conflicts=total_conflicts,
                avg_min_ttc=avg_min_ttc,
                avg_max_drac=avg_max_drac,
                avg_pet=avg_pet
            )
            
        except Exception as e:
            print(f"    Error parsing SSM file {file_path}: {str(e)}")
            return None
    
    def load_benchmarks(self) -> bool:
        """Load benchmark data for all scenarios"""
        print("Loading benchmark data...")
        
        for scenario in self.scenarios:
            print(f"  Loading benchmark for {scenario} scenario...")
            
            # Load benchmark tripinfo
            tripinfo_file = f"trips_output/{scenario}/base.trips.xml"
            if not os.path.exists(tripinfo_file):
                print(f"    Error: Benchmark tripinfo file not found: {tripinfo_file}")
                return False
            
            # Load benchmark SSM data
            ssm_file = f"ssm_output/{scenario}/base.ssm.xml"
            ssm_metrics = self._load_ssm_data(ssm_file)
            
            # Parse benchmark data
            trip_data = self._load_tripinfo(tripinfo_file)
            
            if trip_data is None:
                print(f"    Error: Failed to parse benchmark data for {scenario}")
                return False
            
            # Calculate benchmark metrics using trimmed mean (remove 10% outliers)
            durations = [trip['duration'] for trip in trip_data]
            speeds = [trip['route_length'] / trip['duration'] for trip in trip_data]
            
            avg_duration = trim_mean(durations, 0.1)  # Remove 10% outliers
            avg_speed = trim_mean(speeds, 0.1)  # Remove 10% outliers
            
            # Use default SSM metrics if file not found
            if ssm_metrics is None:
                ssm_metrics = SSMMetrics(
                    max_br=0.0, min_sgap=float('inf'), min_tgap=float('inf'),
                    avg_br=0.0, avg_sgap=float('inf'), avg_tgap=float('inf'),
                    br_threshold_violations=0, sgap_threshold_violations=0, tgap_threshold_violations=0,
                    total_interactions=0
                )
            
            self.benchmarks[scenario] = BenchmarkMetrics(
                avg_duration=avg_duration,
                avg_speed=avg_speed,
                ssm_metrics=ssm_metrics,
                total_vehicles=len(trip_data)
            )
            
            print(f"    Benchmark {scenario}: {len(trip_data)} vehicles, "
                  f"avg duration: {avg_duration:.2f}s, avg speed: {avg_speed:.2f}m/s, "
                  f"SSM conflicts: {ssm_metrics.total_conflicts}")
        
        print("Benchmark data loaded successfully!")
        
        # Load Barrandov benchmark data
        print(f"\n  Loading benchmark for {self.barrandov_scenario} (Barrandov) scenario...")
        
        # Load Barrandov benchmark tripinfo
        tripinfo_file = f"trips_output/{self.barrandov_scenario}/base.trips.xml"
        barrandov_benchmark_trip_data = None
        if not os.path.exists(tripinfo_file):
            print(f"    Warning: Barrandov benchmark tripinfo file not found: {tripinfo_file}")
        else:
            # Load Barrandov benchmark SSM data
            ssm_file = f"ssm_output/{self.barrandov_scenario}/base.ssm.xml"
            ssm_metrics = self._load_ssm_data(ssm_file)
            
            # Parse Barrandov benchmark data
            barrandov_benchmark_trip_data = self._load_tripinfo(tripinfo_file)
            self._barrandov_benchmark_trip_data = barrandov_benchmark_trip_data  # Save for config filtering
            if barrandov_benchmark_trip_data is not None:
                # Filtering will be done after config is loaded
                if ssm_metrics is None:
                    ssm_metrics = SSMMetrics(
                        total_conflicts=0,
                        avg_min_ttc=float('inf'),
                        avg_max_drac=0.0,
                        avg_pet=float('nan')
                    )
                self._barrandov_benchmark_ssm = ssm_metrics
        
        return True
    
    def load_config_data(self) -> bool:
        """Load configuration data for all scenarios"""
        print(f"\nLoading configuration {self.config_id} data...")
        
        for scenario in self.scenarios:
            print(f"  Loading {self.config_id} for {scenario} scenario...")
            
            # Load config tripinfo
            tripinfo_file = f"trips_output/{scenario}/{self.config_id}.trips.xml"
            if not os.path.exists(tripinfo_file):
                print(f"    Error: Config tripinfo file not found: {tripinfo_file}")
                return False
            
            # Load config SSM data
            ssm_file = f"ssm_output/{scenario}/{self.config_id}.ssm.xml"
            ssm_metrics = self._load_ssm_data(ssm_file)
            
            # Parse config data
            trip_data = self._load_tripinfo(tripinfo_file)
            
            if trip_data is None:
                print(f"    Error: Failed to parse config data for {scenario}")
                return False
            
            # Calculate config metrics using trimmed mean (remove 10% outliers)
            durations = [trip['duration'] for trip in trip_data]
            speeds = [trip['route_length'] / trip['duration'] for trip in trip_data]
            
            avg_duration = trim_mean(durations, 0.1)  # Remove 10% outliers
            avg_speed = trim_mean(speeds, 0.1)  # Remove 10% outliers
            
            # Use default SSM metrics if file not found
            if ssm_metrics is None:
                ssm_metrics = SSMMetrics(
                    max_br=0.0, min_sgap=float('inf'), min_tgap=float('inf'),
                    avg_br=0.0, avg_sgap=float('inf'), avg_tgap=float('inf'),
                    br_threshold_violations=0, sgap_threshold_violations=0, tgap_threshold_violations=0,
                    total_interactions=0
                )
            
            self.config_metrics[scenario] = ConfigMetrics(
                avg_duration=avg_duration,
                avg_speed=avg_speed,
                ssm_metrics=ssm_metrics,
                total_vehicles=len(trip_data)
            )
            
            print(f"    Config {scenario}: {len(trip_data)} vehicles, "
                  f"avg duration: {avg_duration:.2f}s, avg speed: {avg_speed:.2f}m/s, "
                  f"SSM conflicts: {ssm_metrics.total_conflicts}")
        
        print("Configuration data loaded successfully!")
        
        # Load Barrandov configuration data
        print(f"\n  Loading {self.config_id} for {self.barrandov_scenario} (Barrandov) scenario...")
        
        # Load Barrandov config tripinfo
        tripinfo_file = f"trips_output/{self.barrandov_scenario}/{self.config_id}.trips.xml"
        barrandov_config_trip_data = None
        if not os.path.exists(tripinfo_file):
            print(f"    Warning: Barrandov config tripinfo file not found: {tripinfo_file}")
        else:
            # Load Barrandov config SSM data
            ssm_file = f"ssm_output/{self.barrandov_scenario}/{self.config_id}.ssm.xml"
            ssm_metrics = self._load_ssm_data(ssm_file)
            
            # Parse Barrandov config data
            barrandov_config_trip_data = self._load_tripinfo(tripinfo_file)
            # Now filter both benchmark and config trip_data to only vehicles present in both
            if hasattr(self, '_barrandov_benchmark_trip_data') and barrandov_config_trip_data is not None:
                bench_ids = set(trip['id'] for trip in self._barrandov_benchmark_trip_data)
                config_ids = set(trip['id'] for trip in barrandov_config_trip_data)
                common_ids = bench_ids & config_ids
                
                filtered_bench = [trip for trip in self._barrandov_benchmark_trip_data if trip['id'] in common_ids]
                filtered_config = [trip for trip in barrandov_config_trip_data if trip['id'] in common_ids]
                
                # Calculate Barrandov benchmark metrics using trimmed mean (remove 10% outliers)
                durations = [trip['duration'] for trip in filtered_bench]
                speeds = [trip['route_length'] / trip['duration'] for trip in filtered_bench]
                avg_duration = trim_mean(durations, 0.1) if durations else float('nan')
                avg_speed = trim_mean(speeds, 0.1) if speeds else float('nan')
                
                # Use benchmark SSM data if available
                benchmark_ssm = getattr(self, '_barrandov_benchmark_ssm', None)
                if benchmark_ssm is None:
                    benchmark_ssm = SSMMetrics(
                        total_conflicts=0,
                        avg_min_ttc=float('inf'),
                        avg_max_drac=0.0,
                        avg_pet=float('nan')
                    )
                
                self.barrandov_benchmark = BenchmarkMetrics(
                    avg_duration=avg_duration,
                    avg_speed=avg_speed,
                    ssm_metrics=benchmark_ssm,
                    total_vehicles=len(filtered_bench)
                )
                
                # Calculate Barrandov config metrics using trimmed mean (remove 10% outliers)
                durations = [trip['duration'] for trip in filtered_config]
                speeds = [trip['route_length'] / trip['duration'] for trip in filtered_config]
                avg_duration = trim_mean(durations, 0.1) if durations else float('nan')
                avg_speed = trim_mean(speeds, 0.1) if speeds else float('nan')
                
                # Use config SSM data if available
                if ssm_metrics is None:
                    ssm_metrics = SSMMetrics(
                        total_conflicts=0,
                        avg_min_ttc=float('inf'),
                        avg_max_drac=0.0,
                        avg_pet=float('nan')
                    )
                
                self.barrandov_config = ConfigMetrics(
                    avg_duration=avg_duration,
                    avg_speed=avg_speed,
                    ssm_metrics=ssm_metrics,
                    total_vehicles=len(filtered_config)
                )
                
                print(f"    Barrandov (filtered) benchmark: {len(filtered_bench)} vehicles, "
                      f"avg duration: {self.barrandov_benchmark.avg_duration:.2f}s, avg speed: {self.barrandov_benchmark.avg_speed:.2f}m/s, "
                      f"SSM conflicts: {self.barrandov_benchmark.ssm_metrics.total_conflicts}")
                print(f"    Barrandov (filtered) config: {len(filtered_config)} vehicles, "
                      f"avg duration: {self.barrandov_config.avg_duration:.2f}s, avg speed: {self.barrandov_config.avg_speed:.2f}m/s, "
                      f"SSM conflicts: {self.barrandov_config.ssm_metrics.total_conflicts}")
            else:
                print(f"    Error: Failed to parse Barrandov config data or missing benchmark data")
        
        return True
    
    def _load_tripinfo(self, file_path: str) -> Optional[List[Dict]]:
        """Load tripinfo XML data"""
        try:
            tree = ET.parse(file_path)
            root = tree.getroot()
            
            trip_data = []
            for tripinfo in root.findall('tripinfo'):
                trip = {
                    'id': tripinfo.get('id', ''),
                    'vtype': tripinfo.get('vType', ''),
                    'depart_time': float(tripinfo.get('depart', 0)),
                    'arrival_time': float(tripinfo.get('arrival', 0)),
                    'duration': float(tripinfo.get('duration', 0)),
                    'route_length': float(tripinfo.get('routeLength', 0)),
                    'time_loss': float(tripinfo.get('timeLoss', 0)),
                    'waiting_time': float(tripinfo.get('waitingTime', 0)),
                    'waiting_count': int(tripinfo.get('waitingCount', 0)),
                    'stop_time': float(tripinfo.get('stopTime', 0)),
                    'depart_lane': tripinfo.get('departLane', ''),
                    'arrival_lane': tripinfo.get('arrivalLane', ''),
                    'depart_speed': float(tripinfo.get('departSpeed', 0)),
                    'arrival_speed': float(tripinfo.get('arrivalSpeed', 0)),
                    'speed_factor': float(tripinfo.get('speedFactor', 1.0)),
                    'depart_pos_lat': float(tripinfo.get('departPosLat', 0)),
                    'arrival_pos_lat': float(tripinfo.get('arrivalPosLat', 0))
                }
                trip_data.append(trip)
                
            return trip_data
            
        except Exception as e:
            print(f"Error loading tripinfo from {file_path}: {e}")
            return None
    
    def _load_error_logs(self, file_path: str) -> Optional[List[Dict]]:
        """Load error log data"""
        try:
            error_events = []
            
            with open(file_path, 'r') as f:
                for line in f:
                    event = self._parse_error_line(line.strip())
                    if event:
                        error_events.append(event)
                        
            return error_events
            
        except Exception as e:
            print(f"Error loading error logs from {file_path}: {e}")
            return None
    
    def _parse_error_line(self, line: str) -> Optional[Dict]:
        """Parse a single error log line"""
        if not line.startswith('Warning:'):
            return None
            
        # Teleportation events - waited too long
        teleport_wait_match = re.search(r"Teleporting vehicle '(\d+)'; waited too long \(([^)]+)\), lane='([^']+)', time=([\d.]+)", line)
        if teleport_wait_match:
            return {
                'vehicle_id': teleport_wait_match.group(1),
                'event_type': 'teleportation',
                'time': float(teleport_wait_match.group(4).rstrip('.')),
                'lane': teleport_wait_match.group(3),
                'teleport_reason': teleport_wait_match.group(2),
                'details': {'reason': teleport_wait_match.group(2)}
            }
        
        # Teleportation events - collision
        teleport_collision_match = re.search(r"Teleporting vehicle '(\d+)'; collision with vehicle '(\d+)', lane='([^']+)', gap=([^,]+), latGap=([^,]+), time=([\d.]+)", line)
        if teleport_collision_match:
            return {
                'vehicle_id': teleport_collision_match.group(1),
                'event_type': 'teleportation',
                'time': float(teleport_collision_match.group(6).rstrip('.')),
                'lane': teleport_collision_match.group(3),
                'teleport_reason': 'collision',
                'details': {
                    'collision_with': teleport_collision_match.group(2),
                    'gap': teleport_collision_match.group(4),
                    'latGap': teleport_collision_match.group(5)
                }
            }
        
        # Emergency braking events
        braking_match = re.search(r"Vehicle '(\d+)' performs emergency braking on lane '([^']+)' with decel=([\d.]+), wished=([\d.]+), severity=([\d.]+), time=([\d.]+)", line)
        if braking_match:
            return {
                'vehicle_id': braking_match.group(1),
                'event_type': 'emergency_braking',
                'time': float(braking_match.group(6).rstrip('.')),
                'lane': braking_match.group(2),
                'details': {
                    'decel': float(braking_match.group(3)),
                    'wished': float(braking_match.group(4)),
                    'severity': float(braking_match.group(5))
                }
            }
        
        return None
    
    def compare_metrics(self):
        """Compare configuration metrics against benchmarks"""
        print("\n=== COMPARISON ANALYSIS ===")
        
        # Print configuration values if available
        if self.config_values:
            print(f"Configuration {self.config_id} parameters:")
            for param, value in self.config_values.items():
                print(f"  {param}: {value}")
            print()
        
        comparison_data = {}
        
        for scenario in self.scenarios:
            benchmark = self.benchmarks[scenario]
            config = self.config_metrics[scenario]
            
            # Calculate relative differences (percentage change from benchmark)
            duration_diff = ((config.avg_duration - benchmark.avg_duration) / benchmark.avg_duration) * 100
            speed_diff = ((config.avg_speed - benchmark.avg_speed) / benchmark.avg_speed) * 100
            
            # SSM metric differences
            conflicts_diff = config.ssm_metrics.total_conflicts - benchmark.ssm_metrics.total_conflicts
            min_ttc_diff = config.ssm_metrics.avg_min_ttc - benchmark.ssm_metrics.avg_min_ttc
            max_drac_diff = config.ssm_metrics.avg_max_drac - benchmark.ssm_metrics.avg_max_drac
            pet_diff = config.ssm_metrics.avg_pet - benchmark.ssm_metrics.avg_pet
            
            comparison_data[scenario] = {
                'duration_diff_pct': duration_diff,
                'speed_diff_pct': speed_diff,
                'conflicts_diff': conflicts_diff,
                'min_ttc_diff': min_ttc_diff,
                'max_drac_diff': max_drac_diff,
                'pet_diff': pet_diff,
                'benchmark': benchmark,
                'config': config
            }
            
            print(f"\n{scenario.upper()} SCENARIO:")
            print(f"  Average Duration:")
            print(f"    Benchmark: {benchmark.avg_duration:.2f}s")
            print(f"    Config {self.config_id}: {config.avg_duration:.2f}s")
            print(f"    Difference: {duration_diff:+.2f}%")
            
            print(f"  Average Speed:")
            print(f"    Benchmark: {benchmark.avg_speed:.2f}m/s")
            print(f"    Config {self.config_id}: {config.avg_speed:.2f}m/s")
            print(f"    Difference: {speed_diff:+.2f}%")
            
            print(f"  SSM Safety Metrics:")
            print(f"    Total Conflicts:")
            print(f"      Benchmark: {benchmark.ssm_metrics.total_conflicts}")
            print(f"      Config {self.config_id}: {config.ssm_metrics.total_conflicts}")
            print(f"      Difference: {conflicts_diff:+d}")
            
            print(f"    Average Minimum TTC (s):")
            print(f"      Benchmark: {benchmark.ssm_metrics.avg_min_ttc:.2f}")
            print(f"      Config {self.config_id}: {config.ssm_metrics.avg_min_ttc:.2f}")
            print(f"      Difference: {min_ttc_diff:+.2f}")
            
            print(f"    Average Maximum DRAC (m/s²):")
            print(f"      Benchmark: {benchmark.ssm_metrics.avg_max_drac:.2f}")
            print(f"      Config {self.config_id}: {config.ssm_metrics.avg_max_drac:.2f}")
            print(f"      Difference: {max_drac_diff:+.2f}")
            
            print(f"    Average PET (s):")
            if not np.isnan(benchmark.ssm_metrics.avg_pet) and not np.isnan(config.ssm_metrics.avg_pet):
                print(f"      Benchmark: {benchmark.ssm_metrics.avg_pet:.2f}")
                print(f"      Config {self.config_id}: {config.ssm_metrics.avg_pet:.2f}")
                print(f"      Difference: {pet_diff:+.2f}")
            else:
                print(f"      Benchmark: {'N/A' if np.isnan(benchmark.ssm_metrics.avg_pet) else f'{benchmark.ssm_metrics.avg_pet:.2f}'}")
                print(f"      Config {self.config_id}: {'N/A' if np.isnan(config.ssm_metrics.avg_pet) else f'{config.ssm_metrics.avg_pet:.2f}'}")
                print(f"      Difference: {'N/A' if np.isnan(pet_diff) else f'{pet_diff:+.2f}'}")
        
        # Calculate total/average across all scenarios
        total_duration_diff = np.mean([comparison_data[scenario]['duration_diff_pct'] for scenario in self.scenarios])
        total_speed_diff = np.mean([comparison_data[scenario]['speed_diff_pct'] for scenario in self.scenarios])
        total_conflicts_diff = np.mean([comparison_data[scenario]['conflicts_diff'] for scenario in self.scenarios])
        
        # Handle TTC diff carefully - only calculate for scenarios with conflicts
        ttc_diff_values = [comparison_data[scenario]['min_ttc_diff'] for scenario in self.scenarios 
                          if not np.isinf(comparison_data[scenario]['benchmark'].ssm_metrics.avg_min_ttc) and 
                             not np.isinf(comparison_data[scenario]['config'].ssm_metrics.avg_min_ttc)]
        total_min_ttc_diff = np.mean(ttc_diff_values) if ttc_diff_values else float('nan')
        
        total_max_drac_diff = np.mean([comparison_data[scenario]['max_drac_diff'] for scenario in self.scenarios])
        
        # Handle PET diff carefully - only calculate if there are valid values
        pet_diff_values = [comparison_data[scenario]['pet_diff'] for scenario in self.scenarios if not np.isnan(comparison_data[scenario]['pet_diff'])]
        total_pet_diff = np.mean(pet_diff_values) if pet_diff_values else float('nan')
        
        # Calculate average benchmark and config values
        avg_benchmark_duration = np.mean([comparison_data[scenario]['benchmark'].avg_duration for scenario in self.scenarios])
        avg_benchmark_speed = np.mean([comparison_data[scenario]['benchmark'].avg_speed for scenario in self.scenarios])
        avg_benchmark_conflicts = np.mean([comparison_data[scenario]['benchmark'].ssm_metrics.total_conflicts for scenario in self.scenarios])
        
        # Handle TTC values carefully - only calculate for scenarios with conflicts
        benchmark_ttc_values = [comparison_data[scenario]['benchmark'].ssm_metrics.avg_min_ttc for scenario in self.scenarios 
                               if not np.isinf(comparison_data[scenario]['benchmark'].ssm_metrics.avg_min_ttc)]
        avg_benchmark_min_ttc = np.mean(benchmark_ttc_values) if benchmark_ttc_values else float('inf')
        
        avg_benchmark_max_drac = np.mean([comparison_data[scenario]['benchmark'].ssm_metrics.avg_max_drac for scenario in self.scenarios])
        # Handle PET values carefully - only calculate if there are valid values
        benchmark_pet_values = [comparison_data[scenario]['benchmark'].ssm_metrics.avg_pet for scenario in self.scenarios if not np.isnan(comparison_data[scenario]['benchmark'].ssm_metrics.avg_pet)]
        avg_benchmark_pet = np.mean(benchmark_pet_values) if benchmark_pet_values else float('nan')
        
        avg_config_duration = np.mean([comparison_data[scenario]['config'].avg_duration for scenario in self.scenarios])
        avg_config_speed = np.mean([comparison_data[scenario]['config'].avg_speed for scenario in self.scenarios])
        avg_config_conflicts = np.mean([comparison_data[scenario]['config'].ssm_metrics.total_conflicts for scenario in self.scenarios])
        
        # Handle TTC values carefully - only calculate for scenarios with conflicts
        config_ttc_values = [comparison_data[scenario]['config'].ssm_metrics.avg_min_ttc for scenario in self.scenarios 
                            if not np.isinf(comparison_data[scenario]['config'].ssm_metrics.avg_min_ttc)]
        avg_config_min_ttc = np.mean(config_ttc_values) if config_ttc_values else float('inf')
        
        avg_config_max_drac = np.mean([comparison_data[scenario]['config'].ssm_metrics.avg_max_drac for scenario in self.scenarios])
        
        # Handle PET values carefully - only calculate if there are valid values
        config_pet_values = [comparison_data[scenario]['config'].ssm_metrics.avg_pet for scenario in self.scenarios if not np.isnan(comparison_data[scenario]['config'].ssm_metrics.avg_pet)]
        avg_config_pet = np.mean(config_pet_values) if config_pet_values else float('nan')
        
        comparison_data['TOTAL'] = {
            'duration_diff_pct': total_duration_diff,
            'speed_diff_pct': total_speed_diff,
            'conflicts_diff': total_conflicts_diff,
            'min_ttc_diff': total_min_ttc_diff,
            'max_drac_diff': total_max_drac_diff,
            'pet_diff': total_pet_diff,
            'benchmark': BenchmarkMetrics(
                avg_duration=avg_benchmark_duration,
                avg_speed=avg_benchmark_speed,
                ssm_metrics=SSMMetrics(
                    total_conflicts=avg_benchmark_conflicts,
                    avg_min_ttc=avg_benchmark_min_ttc,
                    avg_max_drac=avg_benchmark_max_drac,
                    avg_pet=avg_benchmark_pet
                ),
                total_vehicles=0  # Not meaningful for total
            ),
            'config': ConfigMetrics(
                avg_duration=avg_config_duration,
                avg_speed=avg_config_speed,
                ssm_metrics=SSMMetrics(
                    total_conflicts=avg_config_conflicts,
                    avg_min_ttc=avg_config_min_ttc,
                    avg_max_drac=avg_config_max_drac,
                    avg_pet=avg_config_pet
                ),
                total_vehicles=0  # Not meaningful for total
            )
        }
        
        print(f"\nTOTAL (AVERAGE ACROSS ALL SCENARIOS):")
        print(f"  Average Duration:")
        print(f"    Benchmark: {avg_benchmark_duration:.2f}s")
        print(f"    Config {self.config_id}: {avg_config_duration:.2f}s")
        print(f"    Difference: {total_duration_diff:+.2f}%")
        
        print(f"  Average Speed:")
        print(f"    Benchmark: {avg_benchmark_speed:.2f}m/s")
        print(f"    Config {self.config_id}: {avg_config_speed:.2f}m/s")
        print(f"    Difference: {total_speed_diff:+.2f}%")
        
        print(f"  SSM Safety Metrics:")
        print(f"    Total Conflicts:")
        print(f"      Benchmark: {avg_benchmark_conflicts:.1f}")
        print(f"      Config {self.config_id}: {avg_config_conflicts:.1f}")
        print(f"      Difference: {total_conflicts_diff:+.1f}")
        
        print(f"    Average Minimum TTC (s):")
        if not np.isinf(avg_benchmark_min_ttc) and not np.isinf(avg_config_min_ttc):
            print(f"      Benchmark: {avg_benchmark_min_ttc:.2f}")
            print(f"      Config {self.config_id}: {avg_config_min_ttc:.2f}")
            print(f"      Difference: {total_min_ttc_diff:+.2f}")
        else:
            print(f"      Benchmark: {'inf' if np.isinf(avg_benchmark_min_ttc) else f'{avg_benchmark_min_ttc:.2f}'}")
            print(f"      Config {self.config_id}: {'inf' if np.isinf(avg_config_min_ttc) else f'{avg_config_min_ttc:.2f}'}")
            print(f"      Difference: {'N/A (no conflicts in scenarios with finite TTC)' if np.isnan(total_min_ttc_diff) else f'{total_min_ttc_diff:+.2f}'}")
        
        print(f"    Average Maximum DRAC (m/s²):")
        print(f"      Benchmark: {avg_benchmark_max_drac:.2f}")
        print(f"      Config {self.config_id}: {avg_config_max_drac:.2f}")
        print(f"      Difference: {total_max_drac_diff:+.2f}")
        
        print(f"    Average PET (s):")
        if not np.isnan(avg_benchmark_pet) and not np.isnan(avg_config_pet):
            print(f"      Benchmark: {avg_benchmark_pet:.2f}")
            print(f"      Config {self.config_id}: {avg_config_pet:.2f}")
            print(f"      Difference: {total_pet_diff:+.2f}")
        else:
            print(f"      Benchmark: {'N/A' if np.isnan(avg_benchmark_pet) else f'{avg_benchmark_pet:.2f}'}")
            print(f"      Config {self.config_id}: {'N/A' if np.isnan(avg_config_pet) else f'{avg_config_pet:.2f}'}")
            print(f"      Difference: {'N/A' if np.isnan(total_pet_diff) else f'{total_pet_diff:+.2f}'}")
        
        # Add Barrandov scenario comparison (separate from TOTAL)
        if self.barrandov_benchmark is not None and self.barrandov_config is not None:
            print(f"\nBARRANDOV SCENARIO:")
            
            # Calculate Barrandov differences
            barrandov_duration_diff = ((self.barrandov_config.avg_duration - self.barrandov_benchmark.avg_duration) / self.barrandov_benchmark.avg_duration) * 100
            barrandov_speed_diff = ((self.barrandov_config.avg_speed - self.barrandov_benchmark.avg_speed) / self.barrandov_benchmark.avg_speed) * 100
            barrandov_conflicts_diff = self.barrandov_config.ssm_metrics.total_conflicts - self.barrandov_benchmark.ssm_metrics.total_conflicts
            barrandov_min_ttc_diff = self.barrandov_config.ssm_metrics.avg_min_ttc - self.barrandov_benchmark.ssm_metrics.avg_min_ttc
            barrandov_max_drac_diff = self.barrandov_config.ssm_metrics.avg_max_drac - self.barrandov_benchmark.ssm_metrics.avg_max_drac
            barrandov_pet_diff = self.barrandov_config.ssm_metrics.avg_pet - self.barrandov_benchmark.ssm_metrics.avg_pet
            
            comparison_data['BARRANDOV'] = {
                'duration_diff_pct': barrandov_duration_diff,
                'speed_diff_pct': barrandov_speed_diff,
                'conflicts_diff': barrandov_conflicts_diff,
                'min_ttc_diff': barrandov_min_ttc_diff,
                'max_drac_diff': barrandov_max_drac_diff,
                'pet_diff': barrandov_pet_diff,
                'benchmark': self.barrandov_benchmark,
                'config': self.barrandov_config
            }
            
            print(f"  Average Duration:")
            print(f"    Benchmark: {self.barrandov_benchmark.avg_duration:.2f}s")
            print(f"    Config {self.config_id}: {self.barrandov_config.avg_duration:.2f}s")
            print(f"    Difference: {barrandov_duration_diff:+.2f}%")
            
            print(f"  Average Speed:")
            print(f"    Benchmark: {self.barrandov_benchmark.avg_speed:.2f}m/s")
            print(f"    Config {self.config_id}: {self.barrandov_config.avg_speed:.2f}m/s")
            print(f"    Difference: {barrandov_speed_diff:+.2f}%")
            
            print(f"  SSM Safety Metrics:")
            print(f"    Total Conflicts:")
            print(f"      Benchmark: {self.barrandov_benchmark.ssm_metrics.total_conflicts}")
            print(f"      Config {self.config_id}: {self.barrandov_config.ssm_metrics.total_conflicts}")
            print(f"      Difference: {barrandov_conflicts_diff:+d}")
            
            print(f"    Average Minimum TTC (s):")
            print(f"      Benchmark: {self.barrandov_benchmark.ssm_metrics.avg_min_ttc:.2f}")
            print(f"      Config {self.config_id}: {self.barrandov_config.ssm_metrics.avg_min_ttc:.2f}")
            print(f"      Difference: {barrandov_min_ttc_diff:+.2f}")
            
            print(f"    Average Maximum DRAC (m/s²):")
            print(f"      Benchmark: {self.barrandov_benchmark.ssm_metrics.avg_max_drac:.2f}")
            print(f"      Config {self.config_id}: {self.barrandov_config.ssm_metrics.avg_max_drac:.2f}")
            print(f"      Difference: {barrandov_max_drac_diff:+.2f}")
            
            print(f"    Average PET (s):")
            if not np.isnan(self.barrandov_benchmark.ssm_metrics.avg_pet) and not np.isnan(self.barrandov_config.ssm_metrics.avg_pet):
                print(f"      Benchmark: {self.barrandov_benchmark.ssm_metrics.avg_pet:.2f}")
                print(f"      Config {self.config_id}: {self.barrandov_config.ssm_metrics.avg_pet:.2f}")
                print(f"      Difference: {barrandov_pet_diff:+.2f}")
            else:
                print(f"      Benchmark: {'N/A' if np.isnan(self.barrandov_benchmark.ssm_metrics.avg_pet) else f'{self.barrandov_benchmark.ssm_metrics.avg_pet:.2f}'}")
                print(f"      Config {self.config_id}: {'N/A' if np.isnan(self.barrandov_config.ssm_metrics.avg_pet) else f'{self.barrandov_config.ssm_metrics.avg_pet:.2f}'}")
                print(f"      Difference: {'N/A' if np.isnan(barrandov_pet_diff) else f'{barrandov_pet_diff:+.2f}'}")
        else:
            print(f"\nBARRANDOV SCENARIO: Data not available")
        
        self.comparison_results = comparison_data
        return comparison_data
    
    def _get_color_for_metric(self, metric: str, value: float) -> str:
        """Get color based on metric and value (green for good, red for bad)"""
        if metric == 'duration_diff_pct':
            # For duration: negative (decrease) is good (green), positive (increase) is bad (red)
            return 'green' if value < 0 else 'red' if value > 0 else 'gray'
        elif metric == 'speed_diff_pct':
            # For speed: positive (increase) is good (green), negative (decrease) is bad (red)
            return 'green' if value > 0 else 'red' if value < 0 else 'gray'
        elif metric in ['br_violations_diff', 'sgap_violations_diff', 'tgap_violations_diff']:
            # For SSM violations: negative (decrease) is good (green), positive (increase) is bad (red)
            return 'green' if value < 0 else 'red' if value > 0 else 'gray'
        else:
            return 'gray'
    
    def create_comparison_visualizations(self, output_dir: str = "comparison_output"):
        """Create visualizations comparing configuration against benchmarks"""
        print(f"\n=== CREATING COMPARISON VISUALIZATIONS ===")
        
        # Create output directory
        os.makedirs(output_dir, exist_ok=True)
        
        # Prepare data for plotting
        scenarios = list(self.comparison_results.keys())
        # Filter out BARRANDOV from main plots to keep it separate
        plot_scenarios = [s for s in scenarios if s != 'BARRANDOV']
        metrics = ['duration_diff_pct', 'speed_diff_pct', 'br_violations_diff', 'sgap_violations_diff']
        metric_names = ['Duration Difference (%)', 'Speed Difference (%)', 
                       'Braking Rate Violations Diff', 'Space Gap Violations Diff']
        
        # Create comparison plots
        fig, axes = plt.subplots(2, 2, figsize=(15, 12))
        fig.suptitle(f'Configuration {self.config_id} vs Benchmark Comparison', fontsize=16)
        
        for i, (metric, metric_name) in enumerate(zip(metrics, metric_names)):
            row, col = i // 2, i % 2
            ax = axes[row, col]
            
            values = [self.comparison_results[scenario][metric] for scenario in plot_scenarios]
            colors = [self._get_color_for_metric(metric, value) for value in values]
            
            bars = ax.bar(plot_scenarios, values, color=colors, alpha=0.7, edgecolor='black')
            ax.set_title(metric_name)
            ax.set_ylabel(metric_name)
            ax.axhline(y=0, color='black', linestyle='-', alpha=0.3)
            
            # Add value labels on bars
            for bar, value, metric_name in zip(bars, values, metrics):
                height = bar.get_height()
                if metric_name in ['br_violations_diff', 'sgap_violations_diff', 'tgap_violations_diff']:
                    label = f'{value:+.1f}'
                else:
                    label = f'{value:+.1f}' if metric_name.endswith('_pct') else f'{value:+d}'
                ax.text(bar.get_x() + bar.get_width()/2., height,
                       label,
                       ha='center', va='bottom' if height >= 0 else 'top')
            
            ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(f"{output_dir}/comparison_{self.config_id}.png", dpi=300, bbox_inches='tight')
        plt.close()
        
        # Create detailed metrics table visualization
        self._create_metrics_table(output_dir)
        
        # Create separate Barrandov visualization if data is available
        if 'BARRANDOV' in self.comparison_results:
            self._create_barrandov_visualization(output_dir)
        
        print(f"Comparison visualizations saved to {output_dir}/")
    
    def _create_metrics_table(self, output_dir: str):
        """Create a detailed metrics table visualization"""
        fig, ax = plt.subplots(figsize=(12, 8))
        ax.axis('tight')
        ax.axis('off')
        
        # Prepare table data
        table_data = []
        headers = ['Scenario', 'Metric', 'Benchmark', f'Config {self.config_id}', 'Difference']
        
        for scenario in self.scenarios + ['TOTAL']:
            benchmark = self.comparison_results[scenario]['benchmark']
            config = self.comparison_results[scenario]['config']
            
            # Duration
            duration_diff = self.comparison_results[scenario]['duration_diff_pct']
            table_data.append([scenario, 'Avg Duration (s)', 
                             f"{benchmark.avg_duration:.2f}", 
                             f"{config.avg_duration:.2f}", 
                             f"{duration_diff:+.2f}%"])
            
            # Speed
            speed_diff = self.comparison_results[scenario]['speed_diff_pct']
            table_data.append(['', 'Avg Speed (m/s)', 
                             f"{benchmark.avg_speed:.2f}", 
                             f"{config.avg_speed:.2f}", 
                             f"{speed_diff:+.2f}%"])
            
            # SSM Braking Rate Violations
            br_violations_diff = self.comparison_results[scenario]['br_violations_diff']
            table_data.append(['', 'BR Violations (>3.0 m/s²)', 
                             f"{benchmark.ssm_metrics.br_threshold_violations:.1f}", 
                             f"{config.ssm_metrics.br_threshold_violations:.1f}", 
                             f"{br_violations_diff:+.1f}"])
            
            # SSM Space Gap Violations
            sgap_violations_diff = self.comparison_results[scenario]['sgap_violations_diff']
            table_data.append(['', 'SGAP Violations (<2.0s)', 
                             f"{benchmark.ssm_metrics.sgap_threshold_violations:.1f}", 
                             f"{config.ssm_metrics.sgap_threshold_violations:.1f}", 
                             f"{sgap_violations_diff:+.1f}"])
            
            # Add empty row between scenarios
            if scenario != 'TOTAL':
                table_data.append(['', '', '', '', ''])
        
        # Add Barrandov scenario if present
        if 'BARRANDOV' in self.comparison_results:
            # Add empty row before Barrandov
            table_data.append(['', '', '', '', ''])
            scenario = 'BARRANDOV'
            benchmark = self.comparison_results[scenario]['benchmark']
            config = self.comparison_results[scenario]['config']
            duration_diff = self.comparison_results[scenario]['duration_diff_pct']
            speed_diff = self.comparison_results[scenario]['speed_diff_pct']
            br_violations_diff = self.comparison_results[scenario]['br_violations_diff']
            sgap_violations_diff = self.comparison_results[scenario]['sgap_violations_diff']
            table_data.append([scenario, 'Avg Duration (s)', 
                                f"{benchmark.avg_duration:.2f}", 
                                f"{config.avg_duration:.2f}", 
                                f"{duration_diff:+.2f}%"])
            table_data.append(['', 'Avg Speed (m/s)', 
                                f"{benchmark.avg_speed:.2f}", 
                                f"{config.avg_speed:.2f}", 
                                f"{speed_diff:+.2f}%"])
            table_data.append(['', 'BR Violations (>3.0 m/s²)', 
                                f"{benchmark.ssm_metrics.br_threshold_violations:.1f}", 
                                f"{config.ssm_metrics.br_threshold_violations:.1f}", 
                                f"{br_violations_diff:+.1f}"])
            table_data.append(['', 'SGAP Violations (<2.0s)', 
                                f"{benchmark.ssm_metrics.sgap_threshold_violations:.1f}", 
                                f"{config.ssm_metrics.sgap_threshold_violations:.1f}", 
                                f"{sgap_violations_diff:+.1f}"])
        
        # Create table
        table = ax.table(cellText=table_data, colLabels=headers, 
                        cellLoc='center', loc='center')
        table.auto_set_font_size(False)
        table.set_fontsize(10)
        table.scale(1.2, 1.5)
        
        # Style the table
        for i in range(len(headers)):
            table[(0, i)].set_facecolor('#4CAF50')
            table[(0, i)].set_text_props(weight='bold', color='white')
        
        # Color code differences based on metric type
        for i, row in enumerate(table_data):
            if len(row) > 4 and row[4] != '':
                diff_val = row[4]
                metric_type = row[1] if len(row) > 1 else ''
                
                # Determine if the difference is good or bad
                is_good = False
                if 'Duration' in metric_type:
                    # For duration: negative (decrease) is good
                    is_good = diff_val.startswith('-') and not diff_val.startswith('-0')
                elif 'Speed' in metric_type:
                    # For speed: positive (increase) is good
                    is_good = diff_val.startswith('+') and not diff_val.startswith('+0')
                elif 'Teleportations' in metric_type or 'Brakings' in metric_type:
                    # For teleportations and brakings: negative (decrease) is good
                    is_good = diff_val.startswith('-') and not diff_val.startswith('-0')
                
                if is_good:
                    table[(i+1, 4)].set_facecolor('#C8E6C9')  # Light green for better
                elif diff_val.startswith('+') and not diff_val.startswith('+0') or diff_val.startswith('-') and not diff_val.startswith('-0'):
                    table[(i+1, 4)].set_facecolor('#FFCDD2')  # Light red for worse
        
        # Create title with configuration parameters
        if self.config_values:
            param_str = ", ".join([f"{k}={v}" for k, v in self.config_values.items()])
            title = f'Detailed Metrics Comparison: Config {self.config_id} vs Benchmark\nParameters: {param_str}'
        else:
            title = f'Detailed Metrics Comparison: Config {self.config_id} vs Benchmark'
        
        plt.title(title, fontsize=14, pad=20)
        plt.savefig(f"{output_dir}/metrics_table_{self.config_id}.png", 
                   dpi=300, bbox_inches='tight')
        plt.close()
    
    def _create_barrandov_visualization(self, output_dir: str):
        """Create a separate visualization for the Barrandov scenario"""
        if 'BARRANDOV' not in self.comparison_results:
            return
        
        fig, ax = plt.subplots(figsize=(10, 6))
        
        # Get Barrandov data
        barrandov_data = self.comparison_results['BARRANDOV']
        metrics = ['duration_diff_pct', 'speed_diff_pct', 'br_violations_diff', 'sgap_violations_diff']
        metric_names = ['Duration\nDifference (%)', 'Speed\nDifference (%)', 
                       'BR Violations\nDifference', 'SGAP Violations\nDifference']
        
        values = [barrandov_data[metric] for metric in metrics]
        colors = [self._get_color_for_metric(metric, value) for metric, value in zip(metrics, values)]
        
        bars = ax.bar(metric_names, values, color=colors, alpha=0.7, edgecolor='black')
        ax.set_title(f'Barrandov Scenario: Config {self.config_id} vs Benchmark', fontsize=14)
        ax.set_ylabel('Difference from Benchmark')
        ax.axhline(y=0, color='black', linestyle='-', alpha=0.3)
        
        # Add value labels on bars
        for bar, value, metric_name in zip(bars, values, metrics):
            height = bar.get_height()
            if metric_name in ['br_violations_diff', 'sgap_violations_diff']:
                label = f'{value:+.1f}'
            else:
                label = f'{value:+.1f}' if metric_name.endswith('_pct') else f'{value:+d}'
            ax.text(bar.get_x() + bar.get_width()/2., height,
                   label,
                   ha='center', va='bottom' if height >= 0 else 'top')
        
        ax.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig(f"{output_dir}/barrandov_comparison_{self.config_id}.png", dpi=300, bbox_inches='tight')
        plt.close()
    
    def save_comparison_results(self, output_file: str = None):
        """Save comparison results to JSON file"""
        import json
        
        if output_file is None:
            output_file = f"comparison_results_{self.config_id}.json"
        
        # Convert results to JSON-serializable format
        results = {
            'config_id': self.config_id,
            'config_values': self.config_values,
            'comparison_timestamp': pd.Timestamp.now().isoformat(),
            'scenarios': {}
        }
        
        for scenario, data in self.comparison_results.items():
            results['scenarios'][scenario] = {
                'duration_diff_pct': data['duration_diff_pct'],
                'speed_diff_pct': data['speed_diff_pct'],
                'br_violations_diff': data['br_violations_diff'],
                'sgap_violations_diff': data['sgap_violations_diff'],
                'benchmark': {
                    'avg_duration': data['benchmark'].avg_duration,
                    'avg_speed': data['benchmark'].avg_speed,
                    'ssm_metrics': data['benchmark'].ssm_metrics,
                    'total_vehicles': data['benchmark'].total_vehicles
                },
                'config': {
                    'avg_duration': data['config'].avg_duration,
                    'avg_speed': data['config'].avg_speed,
                    'ssm_metrics': data['config'].ssm_metrics,
                    'total_vehicles': data['config'].total_vehicles
                }
            }
        
        with open(output_file, 'w') as f:
            json.dump(results, f, indent=2, default=str)
        
        print(f"Comparison results saved to {output_file}")
    
    def save_ego_results(self, output_file: str = None):
        """Save EGO analysis results to JSON file"""
        import json
        
        if not hasattr(self, 'ego_results'):
            print("No EGO results to save")
            return
        
        if output_file is None:
            output_file = f"ego_analysis_{self.config_id}.json"
        
        # Convert results to JSON-serializable format
        results = {
            'config_id': self.config_id,
            'config_values': self.config_values,
            'analysis_timestamp': pd.Timestamp.now().isoformat(),
            'scenarios': {}
        }
        
        for scenario, data in self.ego_results.items():
            results['scenarios'][scenario] = {
                'ego_comparison': data['ego_comparison'],
                'proximity_analysis': data['proximity_analysis']
            }
        
        with open(output_file, 'w') as f:
            json.dump(results, f, indent=2, default=str)
        
        print(f"EGO analysis results saved to {output_file}")
        
        # Also save EGO analysis table
        output_dir = "comparison_output"
        os.makedirs(output_dir, exist_ok=True)
        save_ego_analysis_table(self.ego_results, output_dir, self.config_id)
    
    def run_comparison(self, create_plots: bool = True, save_results: bool = True, include_ego_analysis: bool = True):
        """Run EGO vehicle analysis and proximity impact assessment"""
        print(f"Starting EGO vehicle analysis for configuration {self.config_id}")
        print("=" * 60)
        ego_results = None
        if include_ego_analysis:
            ego_results = self.analyze_ego_vehicles()
            if save_results:
                self.save_ego_results()
        print("\nEGO vehicle analysis complete!")
        return ego_results

    def analyze_ego_vehicles(self):
        """Second stage analysis: Focus on EGO vehicles and their impact on nearby vehicles"""
        print("\n=== EGO VEHICLE ANALYSIS ===")
        
        ego_results = {}
        
        for scenario in self.scenarios:
            print(f"\nAnalyzing EGO vehicles for {scenario} scenario...")
            
            # Load tripinfo files for this scenario
            benchmark_tripinfo = f"trips_output/{scenario}/base.trips.xml"
            config_tripinfo = f"trips_output/{scenario}/{self.config_id}.trips.xml"
            
            if not os.path.exists(benchmark_tripinfo) or not os.path.exists(config_tripinfo):
                print(f"  Warning: Tripinfo files not found for {scenario}")
                continue
            
            # Load and parse trip data
            benchmark_trips = self._load_tripinfo(benchmark_tripinfo)
            config_trips = self._load_tripinfo(config_tripinfo)
            
            if benchmark_trips is None or config_trips is None:
                print(f"  Error: Failed to parse trip data for {scenario}")
                continue
            
            # Find EGO vehicles in config file
            config_ego = [trip for trip in config_trips if trip['vtype'] == 'EGO']
            
            if len(config_ego) == 0:
                print(f"  Warning: No EGO vehicles found in config for {scenario}")
                continue
            
            # Find the same vehicles in benchmark file by ID (regardless of their type there)
            config_ego_ids = set(trip['id'] for trip in config_ego)
            benchmark_ego = [trip for trip in benchmark_trips if trip['id'] in config_ego_ids]
            
            print(f"  Found {len(config_ego)} EGO vehicles in config, {len(benchmark_ego)} matching vehicles in benchmark")
            
            if len(benchmark_ego) == 0:
                print(f"  Warning: No matching vehicles found in benchmark for {scenario}")
                continue
            
            # Compare EGO vehicle metrics
            ego_comparison = self._compare_ego_vehicles(benchmark_ego, config_ego, scenario)
            
            # Analyze impact on nearby vehicles
            proximity_analysis = self._analyze_ego_proximity_impact(
                benchmark_trips, config_trips, benchmark_ego, config_ego, scenario
            )
            
            ego_results[scenario] = {
                'ego_comparison': ego_comparison,
                'proximity_analysis': proximity_analysis
            }
        
        # Add Barrandov EGO analysis
        print(f"\nAnalyzing EGO vehicles for {self.barrandov_scenario} (Barrandov) scenario...")
        
        # Load Barrandov tripinfo files
        benchmark_tripinfo = f"trips_output/{self.barrandov_scenario}/base.trips.xml"
        config_tripinfo = f"trips_output/{self.barrandov_scenario}/{self.config_id}.trips.xml"
        
        if os.path.exists(benchmark_tripinfo) and os.path.exists(config_tripinfo):
            benchmark_trips = self._load_tripinfo(benchmark_tripinfo)
            config_trips = self._load_tripinfo(config_tripinfo)
            
            if benchmark_trips is not None and config_trips is not None:
                # Filter to common vehicles (as in main analysis)
                bench_ids = set(trip['id'] for trip in benchmark_trips)
                config_ids = set(trip['id'] for trip in config_trips)
                common_ids = bench_ids & config_ids
                filtered_bench = [trip for trip in benchmark_trips if trip['id'] in common_ids]
                filtered_config = [trip for trip in config_trips if trip['id'] in common_ids]
                
                # Find EGO vehicles in config file
                config_ego = [trip for trip in filtered_config if trip['vtype'] == 'EGO']
                
                if len(config_ego) == 0:
                    print(f"  Warning: No EGO vehicles found in config for {self.barrandov_scenario}")
                else:
                    # Find the same vehicles in benchmark file by ID (regardless of their type there)
                    config_ego_ids = set(trip['id'] for trip in config_ego)
                    benchmark_ego = [trip for trip in filtered_bench if trip['id'] in config_ego_ids]
                    
                    print(f"  Found {len(config_ego)} EGO vehicles in config, {len(benchmark_ego)} matching vehicles in benchmark")
                    
                    if len(benchmark_ego) > 0:
                        ego_comparison = self._compare_ego_vehicles(benchmark_ego, config_ego, 'BARRANDOV')
                        proximity_analysis = self._analyze_ego_proximity_impact(
                            filtered_bench, filtered_config, benchmark_ego, config_ego, 'BARRANDOV'
                        )
                        
                        ego_results['BARRANDOV'] = {
                            'ego_comparison': ego_comparison,
                            'proximity_analysis': proximity_analysis
                        }
        else:
            print(f"  Warning: Barrandov tripinfo files not found")
            if not os.path.exists(benchmark_tripinfo):
                print(f"    Missing: {benchmark_tripinfo}")
            if not os.path.exists(config_tripinfo):
                print(f"    Missing: {config_tripinfo}")
        
        # Calculate totals (average across main scenarios)
        if len([s for s in ego_results.keys() if s != 'BARRANDOV']) > 0:
            main_scenarios = [s for s in ego_results.keys() if s != 'BARRANDOV']
            total_ego_comparison = self._calculate_total_ego_comparison(
                [ego_results[s]['ego_comparison'] for s in main_scenarios]
            )
            total_proximity_analysis = self._calculate_total_proximity_analysis(
                [ego_results[s]['proximity_analysis'] for s in main_scenarios]
            )
            
            ego_results['TOTAL'] = {
                'ego_comparison': total_ego_comparison,
                'proximity_analysis': total_proximity_analysis
            }
        
        self.ego_results = ego_results
        return ego_results
    
    def _compare_ego_vehicles(self, benchmark_ego, config_ego, scenario):
        """Compare EGO vehicle metrics between benchmark and config"""
        # Calculate metrics for EGO vehicles
        bench_durations = [trip['duration'] for trip in benchmark_ego]
        bench_speeds = [trip['route_length'] / trip['duration'] for trip in benchmark_ego]
        config_durations = [trip['duration'] for trip in config_ego]
        config_speeds = [trip['route_length'] / trip['duration'] for trip in config_ego]
        
        # Use trimmed means to remove outliers
        bench_avg_duration = trim_mean(bench_durations, 0.1) if bench_durations else float('nan')
        bench_avg_speed = trim_mean(bench_speeds, 0.1) if bench_speeds else float('nan')
        config_avg_duration = trim_mean(config_durations, 0.1) if config_durations else float('nan')
        config_avg_speed = trim_mean(config_speeds, 0.1) if config_speeds else float('nan')
        
        # Calculate differences
        duration_diff = ((config_avg_duration - bench_avg_duration) / bench_avg_duration) * 100 if bench_avg_duration != 0 else float('nan')
        speed_diff = ((config_avg_speed - bench_avg_speed) / bench_avg_speed) * 100 if bench_avg_speed != 0 else float('nan')
        
        print(f"  EGO vehicles - Duration: {bench_avg_duration:.2f}s → {config_avg_duration:.2f}s ({duration_diff:+.2f}%)")
        print(f"  EGO vehicles - Speed: {bench_avg_speed:.2f}m/s → {config_avg_speed:.2f}m/s ({speed_diff:+.2f}%)")
        
        return {
            'benchmark_duration': bench_avg_duration,
            'benchmark_speed': bench_avg_speed,
            'config_duration': config_avg_duration,
            'config_speed': config_avg_speed,
            'duration_diff_pct': duration_diff,
            'speed_diff_pct': speed_diff,
            'num_ego_vehicles': len(benchmark_ego)
        }
    
    def _analyze_ego_proximity_impact(self, benchmark_trips, config_trips, benchmark_ego, config_ego, scenario):
        """Analyze how EGO vehicles affect nearby vehicles based on performance impact"""
        print(f"  Analyzing EGO proximity impact...")
        
        # Find the earliest EGO vehicle departure time
        ego_departure_times = [ego_trip['depart_time'] for ego_trip in benchmark_ego]
        if not ego_departure_times:
            print(f"  No EGO vehicles found")
            return self._empty_proximity_result()
        
        first_ego_departure = min(ego_departure_times)
        print(f"  First EGO vehicle departs at {first_ego_departure:.2f}s")
        
        # Load SSM data for proximity impact analysis
        # For Barrandov, use 'reset' folder instead of scenario name
        if scenario == 'BARRANDOV':
            benchmark_ssm_file = f"ssm_output/reset/base.ssm.xml"
            config_ssm_file = f"ssm_output/reset/{self.config_id}.ssm.xml"
        else:
            benchmark_ssm_file = f"ssm_output/{scenario}/base.ssm.xml"
            config_ssm_file = f"ssm_output/{scenario}/{self.config_id}.ssm.xml"
        
        benchmark_ssm = self._load_ssm_data(benchmark_ssm_file)
        config_ssm = self._load_ssm_data(config_ssm_file)
        
        # Use default SSM metrics if files not found
        if benchmark_ssm is None:
            benchmark_ssm = SSMMetrics(
                total_conflicts=0,
                avg_min_ttc=float('inf'),
                avg_max_drac=0.0,
                avg_pet=float('nan')
            )
        
        if config_ssm is None:
            config_ssm = SSMMetrics(
                total_conflicts=0,
                avg_min_ttc=float('inf'),
                avg_max_drac=0.0,
                avg_pet=float('nan')
            )
        
        print(f"  Benchmark SSM: {benchmark_ssm.total_conflicts} conflicts, avg minTTC: {benchmark_ssm.avg_min_ttc:.2f}s, avg maxDRAC: {benchmark_ssm.avg_max_drac:.2f}m/s²")
        print(f"  Config SSM: {config_ssm.total_conflicts} conflicts, avg minTTC: {config_ssm.avg_min_ttc:.2f}s, avg maxDRAC: {config_ssm.avg_max_drac:.2f}m/s²")
        
        # Sort all vehicles by departure time
        benchmark_trips_sorted = sorted(benchmark_trips, key=lambda x: x['depart_time'])
        config_trips_sorted = sorted(config_trips, key=lambda x: x['depart_time'])
        
        # Create lookup dictionaries for quick access
        benchmark_lookup = {trip['id']: trip for trip in benchmark_trips}
        config_lookup = {trip['id']: trip for trip in config_trips}
        
        # Find vehicles that depart after the first EGO vehicle
        post_ego_vehicles = []
        for trip in benchmark_trips_sorted:
            if trip['depart_time'] >= first_ego_departure and trip['vtype'] != 'EGO':
                vehicle_id = trip['id']
                if vehicle_id in config_lookup:
                    post_ego_vehicles.append(vehicle_id)
        
        print(f"  Found {len(post_ego_vehicles)} vehicles departing after first EGO vehicle")
        
        # Analyze performance changes for each vehicle
        proximity_vehicles = []
        consecutive_normal = 0
        max_consecutive_normal = 5  # Stop when 5 consecutive vehicles show no significant changes
        consecutive_normal_indices = []  # Track indices of consecutive normal vehicles in departure order
        
        for idx, vehicle_id in enumerate(post_ego_vehicles):
            bench_trip = benchmark_lookup[vehicle_id]
            config_trip = config_lookup[vehicle_id]
            
            # Calculate performance metrics
            bench_duration = bench_trip['duration']
            bench_speed = bench_trip['route_length'] / bench_trip['duration']
            config_duration = config_trip['duration']
            config_speed = config_trip['route_length'] / config_trip['duration']
            
            # Calculate percentage changes
            duration_change = abs((config_duration - bench_duration) / bench_duration * 100) if bench_duration > 0 else 0
            speed_change = abs((config_speed - bench_speed) / bench_speed * 100) if bench_speed > 0 else 0
            
            # Check if vehicle is significantly affected
            # Criteria: any metric off by >= 5% (using performance-based approach)
            is_affected = (
                duration_change >= 5.0 or 
                speed_change >= 5.0
            )
            
            if is_affected:
                proximity_vehicles.append(vehicle_id)
                consecutive_normal = 0  # Reset counter
                consecutive_normal_indices = []  # Reset consecutive indices
            else:
                consecutive_normal += 1
                consecutive_normal_indices.append(idx)
                
                # Check if we have 5 consecutive vehicles in departure order
                if consecutive_normal >= max_consecutive_normal:
                    # Check if the last 5 indices are consecutive (representing consecutive vehicles in departure order)
                    last_5_indices = consecutive_normal_indices[-max_consecutive_normal:]
                    is_consecutive_sequence = True
                    
                    for i in range(1, len(last_5_indices)):
                        if last_5_indices[i] != last_5_indices[i-1] + 1:
                            is_consecutive_sequence = False
                            break
                    
                    if is_consecutive_sequence:
                        # Get the vehicle IDs for the consecutive sequence
                        consecutive_vehicle_ids = [post_ego_vehicles[idx] for idx in last_5_indices]
                        print(f"  Proximity interval ended after {len(proximity_vehicles)} affected vehicles")
                        print(f"  (5 consecutive vehicles in departure order showed normal performance)")
                        print(f"  Consecutive vehicle IDs (in departure order): {consecutive_vehicle_ids}")
                        break
        
        print(f"  Found {len(proximity_vehicles)} vehicles in performance-based proximity")
        
        # Calculate metrics for proximity vehicles
        if proximity_vehicles:
            bench_proximity_trips = [benchmark_lookup[vid] for vid in proximity_vehicles]
            config_proximity_trips = [config_lookup[vid] for vid in proximity_vehicles]
            
            # Calculate average metrics using trimmed means
            bench_durations = [trip['duration'] for trip in bench_proximity_trips]
            bench_speeds = [trip['route_length'] / trip['duration'] for trip in bench_proximity_trips]
            config_durations = [trip['duration'] for trip in config_proximity_trips]
            config_speeds = [trip['route_length'] / trip['duration'] for trip in config_proximity_trips]
            
            bench_avg_duration = trim_mean(bench_durations, 0.1) if bench_durations else float('nan')
            bench_avg_speed = trim_mean(bench_speeds, 0.1) if bench_speeds else float('nan')
            config_avg_duration = trim_mean(config_durations, 0.1) if config_durations else float('nan')
            config_avg_speed = trim_mean(config_speeds, 0.1) if config_speeds else float('nan')
            
            duration_diff = ((config_avg_duration - bench_avg_duration) / bench_avg_duration) * 100 if bench_avg_duration != 0 else float('nan')
            speed_diff = ((config_avg_speed - bench_avg_speed) / bench_avg_speed) * 100 if bench_avg_speed != 0 else float('nan')
            
            # Calculate SSM metrics differences for proximity impact
            conflicts_diff = config_ssm.total_conflicts - benchmark_ssm.total_conflicts
            min_ttc_diff = config_ssm.avg_min_ttc - benchmark_ssm.avg_min_ttc
            max_drac_diff = config_ssm.avg_max_drac - benchmark_ssm.avg_max_drac
            pet_diff = config_ssm.avg_pet - benchmark_ssm.avg_pet
            
            print(f"  Proximity vehicles - Duration: {bench_avg_duration:.2f}s → {config_avg_duration:.2f}s ({duration_diff:+.2f}%)")
            print(f"  Proximity vehicles - Speed: {bench_avg_speed:.2f}m/s → {config_avg_speed:.2f}m/s ({speed_diff:+.2f}%)")
            print(f"  SSM - Total Conflicts: {benchmark_ssm.total_conflicts} → {config_ssm.total_conflicts} ({conflicts_diff:+d})")
            
            # Handle TTC display - show N/A if no conflicts
            if benchmark_ssm.total_conflicts == 0 and config_ssm.total_conflicts == 0:
                print(f"  SSM - Avg Min TTC: N/A (no conflicts)")
            elif np.isinf(benchmark_ssm.avg_min_ttc) and np.isinf(config_ssm.avg_min_ttc):
                print(f"  SSM - Avg Min TTC: N/A (no conflicts)")
            else:
                bench_ttc_str = "inf" if np.isinf(benchmark_ssm.avg_min_ttc) else f"{benchmark_ssm.avg_min_ttc:.2f}"
                config_ttc_str = "inf" if np.isinf(config_ssm.avg_min_ttc) else f"{config_ssm.avg_min_ttc:.2f}"
                diff_str = "N/A" if np.isnan(min_ttc_diff) else f"{min_ttc_diff:+.2f}"
                print(f"  SSM - Avg Min TTC: {bench_ttc_str}s → {config_ttc_str}s ({diff_str}s)")
            
            print(f"  SSM - Avg Max DRAC: {benchmark_ssm.avg_max_drac:.2f}m/s² → {config_ssm.avg_max_drac:.2f}m/s² ({max_drac_diff:+.2f}m/s²)")
            if not np.isnan(pet_diff):
                print(f"  SSM - Avg PET: {benchmark_ssm.avg_pet:.2f}s → {config_ssm.avg_pet:.2f}s ({pet_diff:+.2f}s)")
            else:
                print(f"  SSM - Avg PET: N/A")
            
            return {
                'benchmark_duration': bench_avg_duration,
                'benchmark_speed': bench_avg_speed,
                'config_duration': config_avg_duration,
                'config_speed': config_avg_speed,
                'duration_diff_pct': duration_diff,
                'speed_diff_pct': speed_diff,
                'benchmark_conflicts': benchmark_ssm.total_conflicts,
                'config_conflicts': config_ssm.total_conflicts,
                'conflicts_diff': conflicts_diff,
                'benchmark_min_ttc': benchmark_ssm.avg_min_ttc,
                'config_min_ttc': config_ssm.avg_min_ttc,
                'min_ttc_diff': min_ttc_diff,
                'benchmark_max_drac': benchmark_ssm.avg_max_drac,
                'config_max_drac': config_ssm.avg_max_drac,
                'max_drac_diff': max_drac_diff,
                'benchmark_pet': benchmark_ssm.avg_pet,
                'config_pet': config_ssm.avg_pet,
                'pet_diff': pet_diff,
                'num_proximity_vehicles': len(proximity_vehicles),
                'proximity_definition': 'performance_based',
                'first_ego_departure': first_ego_departure,
                'consecutive_normal_threshold': max_consecutive_normal
            }
        else:
            print(f"  No vehicles significantly affected by EGO behavior")
            return self._empty_proximity_result()
    
    def _empty_proximity_result(self):
        """Return empty proximity analysis result"""
        return {
            'benchmark_duration': float('nan'),
            'benchmark_speed': float('nan'),
            'config_duration': float('nan'),
            'config_speed': float('nan'),
            'duration_diff_pct': float('nan'),
            'speed_diff_pct': float('nan'),
            'benchmark_conflicts': 0,
            'config_conflicts': 0,
            'conflicts_diff': 0,
            'benchmark_min_ttc': float('inf'),
            'config_min_ttc': float('inf'),
            'min_ttc_diff': 0.0,
            'benchmark_max_drac': 0.0,
            'config_max_drac': 0.0,
            'max_drac_diff': 0.0,
            'benchmark_pet': float('nan'),
            'config_pet': float('nan'),
            'pet_diff': float('nan'),
            'num_proximity_vehicles': 0,
            'proximity_definition': 'performance_based',
            'first_ego_departure': float('nan'),
            'consecutive_normal_threshold': 5
        }
    
    def _calculate_total_ego_comparison(self, ego_comparisons):
        """Calculate total/average EGO comparison across scenarios"""
        valid_comparisons = [comp for comp in ego_comparisons if not (np.isnan(comp['duration_diff_pct']) or np.isnan(comp['speed_diff_pct']))]
        
        if not valid_comparisons:
            return {
                'benchmark_duration': float('nan'),
                'benchmark_speed': float('nan'),
                'config_duration': float('nan'),
                'config_speed': float('nan'),
                'duration_diff_pct': float('nan'),
                'speed_diff_pct': float('nan'),
                'num_ego_vehicles': 0
            }
        
        avg_bench_duration = np.mean([comp['benchmark_duration'] for comp in valid_comparisons])
        avg_bench_speed = np.mean([comp['benchmark_speed'] for comp in valid_comparisons])
        avg_config_duration = np.mean([comp['config_duration'] for comp in valid_comparisons])
        avg_config_speed = np.mean([comp['config_speed'] for comp in valid_comparisons])
        avg_duration_diff = np.mean([comp['duration_diff_pct'] for comp in valid_comparisons])
        avg_speed_diff = np.mean([comp['speed_diff_pct'] for comp in valid_comparisons])
        total_ego_vehicles = sum([comp['num_ego_vehicles'] for comp in valid_comparisons])
        
        return {
            'benchmark_duration': avg_bench_duration,
            'benchmark_speed': avg_bench_speed,
            'config_duration': avg_config_duration,
            'config_speed': avg_config_speed,
            'duration_diff_pct': avg_duration_diff,
            'speed_diff_pct': avg_speed_diff,
            'num_ego_vehicles': total_ego_vehicles
        }
    
    def _calculate_total_proximity_analysis(self, proximity_analyses):
        """Calculate total/average proximity analysis across scenarios"""
        valid_analyses = [analysis for analysis in proximity_analyses if not (np.isnan(analysis['duration_diff_pct']) or np.isnan(analysis['speed_diff_pct']))]
        
        if not valid_analyses:
            return {
                'benchmark_duration': float('nan'),
                'benchmark_speed': float('nan'),
                'config_duration': float('nan'),
                'config_speed': float('nan'),
                'duration_diff_pct': float('nan'),
                'speed_diff_pct': float('nan'),
                'benchmark_teleportations': 0,
                'config_teleportations': 0,
                'teleportation_diff': 0,
                'benchmark_brakings': 0,
                'config_brakings': 0,
                'braking_diff': 0,
                'num_proximity_vehicles': 0,
                'proximity_definition': 'performance_based',
                'first_ego_departure': float('nan'),
                'consecutive_normal_threshold': 5
            }
        
        avg_bench_duration = np.mean([analysis['benchmark_duration'] for analysis in valid_analyses])
        avg_bench_speed = np.mean([analysis['benchmark_speed'] for analysis in valid_analyses])
        avg_config_duration = np.mean([analysis['config_duration'] for analysis in valid_analyses])
        avg_config_speed = np.mean([analysis['config_speed'] for analysis in valid_analyses])
        avg_duration_diff = np.mean([analysis['duration_diff_pct'] for analysis in valid_analyses])
        avg_speed_diff = np.mean([analysis['speed_diff_pct'] for analysis in valid_analyses])
        
        # Sum/average SSM metrics
        total_bench_conflicts = sum([analysis['benchmark_conflicts'] for analysis in valid_analyses])
        total_config_conflicts = sum([analysis['config_conflicts'] for analysis in valid_analyses])
        total_conflicts_diff = total_config_conflicts - total_bench_conflicts
        
        # Handle TTC values carefully - only calculate if there are valid values
        bench_min_ttc_values = [analysis['benchmark_min_ttc'] for analysis in valid_analyses if not np.isinf(analysis['benchmark_min_ttc'])]
        config_min_ttc_values = [analysis['config_min_ttc'] for analysis in valid_analyses if not np.isinf(analysis['config_min_ttc'])]
        avg_bench_min_ttc = np.mean(bench_min_ttc_values) if bench_min_ttc_values else float('inf')
        avg_config_min_ttc = np.mean(config_min_ttc_values) if config_min_ttc_values else float('inf')
        avg_min_ttc_diff = avg_config_min_ttc - avg_bench_min_ttc if not np.isinf(avg_bench_min_ttc) and not np.isinf(avg_config_min_ttc) else float('nan')
        
        avg_bench_max_drac = np.mean([analysis['benchmark_max_drac'] for analysis in valid_analyses])
        avg_config_max_drac = np.mean([analysis['config_max_drac'] for analysis in valid_analyses])
        avg_max_drac_diff = avg_config_max_drac - avg_bench_max_drac
        
        # Handle PET values carefully - only calculate if there are valid values
        bench_pet_values = [analysis['benchmark_pet'] for analysis in valid_analyses if not np.isnan(analysis['benchmark_pet'])]
        config_pet_values = [analysis['config_pet'] for analysis in valid_analyses if not np.isnan(analysis['config_pet'])]
        avg_bench_pet = np.mean(bench_pet_values) if bench_pet_values else float('nan')
        avg_config_pet = np.mean(config_pet_values) if config_pet_values else float('nan')
        avg_pet_diff = avg_config_pet - avg_bench_pet if not np.isnan(avg_bench_pet) and not np.isnan(avg_config_pet) else float('nan')
        
        total_proximity_vehicles = sum([analysis['num_proximity_vehicles'] for analysis in valid_analyses])
        
        return {
            'benchmark_duration': avg_bench_duration,
            'benchmark_speed': avg_bench_speed,
            'config_duration': avg_config_duration,
            'config_speed': avg_config_speed,
            'duration_diff_pct': avg_duration_diff,
            'speed_diff_pct': avg_speed_diff,
            'benchmark_conflicts': total_bench_conflicts,
            'config_conflicts': total_config_conflicts,
            'conflicts_diff': total_conflicts_diff,
            'benchmark_min_ttc': avg_bench_min_ttc,
            'config_min_ttc': avg_config_min_ttc,
            'min_ttc_diff': avg_min_ttc_diff,
            'benchmark_max_drac': avg_bench_max_drac,
            'config_max_drac': avg_config_max_drac,
            'max_drac_diff': avg_max_drac_diff,
            'benchmark_pet': avg_bench_pet,
            'config_pet': avg_config_pet,
            'pet_diff': avg_pet_diff,
            'num_proximity_vehicles': total_proximity_vehicles,
            'proximity_definition': 'performance_based',
            'first_ego_departure': float('nan'),
            'consecutive_normal_threshold': 5
        }

def save_ego_analysis_table(ego_results, output_dir, config_id=None):
    """Save EGO analysis results in a structured table format"""
    
    # Create table data
    table_data = []
    
    # Add header
    table_data.append([
        "Scenario", "EGO Duration Change (%)", "EGO Speed Change (%)", 
        "Proximity Vehicles", "Duration Change (%)", "Speed Change (%)",
        "Conflicts Change", "TTC Change (%)", "DRAC Change (%)"
    ])
    
    # Add data for each scenario
    for scenario in ['straight', 'merge', 'secondary_merge']:
        if scenario in ego_results:
            ego_data = ego_results[scenario]
            ego_comparison = ego_data['ego_comparison']
            proximity_analysis = ego_data['proximity_analysis']
            
            # EGO vehicle metrics
            ego_duration_change = ego_comparison['duration_diff_pct']
            ego_speed_change = ego_comparison['speed_diff_pct']
            
            # Proximity metrics
            proximity_count = proximity_analysis['num_proximity_vehicles']
            proximity_duration_change = proximity_analysis['duration_diff_pct']
            proximity_speed_change = proximity_analysis['speed_diff_pct']
            proximity_conflicts_change = proximity_analysis['conflicts_diff']
            proximity_min_ttc_change = proximity_analysis['min_ttc_diff']
            proximity_max_drac_change = proximity_analysis['max_drac_diff']
            
            # Calculate percentage changes for TTC and DRAC
            proximity_min_ttc_bench = proximity_analysis['benchmark_min_ttc']
            proximity_min_ttc_config = proximity_analysis['config_min_ttc']
            proximity_max_drac_bench = proximity_analysis['benchmark_max_drac']
            proximity_max_drac_config = proximity_analysis['config_max_drac']
            
            # Calculate TTC percentage change (handle infinite values)
            if np.isinf(proximity_min_ttc_bench) and np.isinf(proximity_min_ttc_config):
                ttc_change_pct = 0.0
                ttc_change_str = "N/A"
            elif np.isinf(proximity_min_ttc_bench):
                ttc_change_str = "N/A"
            elif proximity_min_ttc_bench != 0:
                ttc_change_pct = (proximity_min_ttc_config - proximity_min_ttc_bench) / proximity_min_ttc_bench * 100
                ttc_change_str = f"{ttc_change_pct:+.1f}%"
            else:
                ttc_change_str = "N/A"
            
            # Calculate DRAC percentage change
            if proximity_max_drac_bench != 0:
                drac_change_pct = (proximity_max_drac_config - proximity_max_drac_bench) / proximity_max_drac_bench * 100
                drac_change_str = f"{drac_change_pct:+.1f}%"
            else:
                drac_change_str = "N/A"
            
            table_data.append([
                scenario.upper(),
                f"{ego_duration_change:+.1f}%" if not np.isnan(ego_duration_change) else "N/A",
                f"{ego_speed_change:+.1f}%" if not np.isnan(ego_speed_change) else "N/A",
                f"{proximity_count}",
                f"{proximity_duration_change:+.1f}%" if not np.isnan(proximity_duration_change) and proximity_count > 0 else "N/A",
                f"{proximity_speed_change:+.1f}%" if not np.isnan(proximity_speed_change) and proximity_count > 0 else "N/A",
                f"{proximity_conflicts_change:+d}",
                ttc_change_str,
                drac_change_str
            ])
    
    # Calculate totals (average across main scenarios, excluding Barrandov)
    main_scenarios = ['straight', 'merge', 'secondary_merge']
    if all(scenario in ego_results for scenario in main_scenarios):
        # EGO metrics (always available)
        total_ego_duration_change = np.mean([ego_results[scenario]['ego_comparison']['duration_diff_pct'] for scenario in main_scenarios])
        total_ego_speed_change = np.mean([ego_results[scenario]['ego_comparison']['speed_diff_pct'] for scenario in main_scenarios])
        
        # Proximity metrics (include all scenarios, use 0% for scenarios with no proximity vehicles)
        total_proximity_count = np.mean([ego_results[scenario]['proximity_analysis']['num_proximity_vehicles'] for scenario in main_scenarios])
        
        # Include all scenarios, use 0% for scenarios with no proximity vehicles
        proximity_duration_changes = []
        proximity_speed_changes = []
        proximity_conflicts_changes = []
        
        for scenario in main_scenarios:
            if ego_results[scenario]['proximity_analysis']['num_proximity_vehicles'] > 0:
                proximity_duration_changes.append(ego_results[scenario]['proximity_analysis']['duration_diff_pct'])
                proximity_speed_changes.append(ego_results[scenario]['proximity_analysis']['speed_diff_pct'])
                proximity_conflicts_changes.append(ego_results[scenario]['proximity_analysis']['conflicts_diff'])
            else:
                # No proximity vehicles = no change = 0%
                proximity_duration_changes.append(0.0)
                proximity_speed_changes.append(0.0)
                proximity_conflicts_changes.append(0)
        
        total_proximity_duration_change = np.mean(proximity_duration_changes)
        total_proximity_speed_change = np.mean(proximity_speed_changes)
        total_proximity_conflicts_change = np.mean(proximity_conflicts_changes)
        
        # TTC and DRAC percentage changes (include all scenarios, use 0% for N/A cases)
        ttc_changes = []
        drac_changes = []
        
        for scenario in main_scenarios:
            bench_ttc = ego_results[scenario]['proximity_analysis']['benchmark_min_ttc']
            config_ttc = ego_results[scenario]['proximity_analysis']['config_min_ttc']
            if not np.isinf(bench_ttc) and bench_ttc != 0:
                ttc_change_pct = (config_ttc - bench_ttc) / bench_ttc * 100
                ttc_changes.append(ttc_change_pct)
            else:
                # No conflicts = no change = 0%
                ttc_changes.append(0.0)
            
            bench_drac = ego_results[scenario]['proximity_analysis']['benchmark_max_drac']
            config_drac = ego_results[scenario]['proximity_analysis']['config_max_drac']
            if bench_drac != 0:
                drac_change_pct = (config_drac - bench_drac) / bench_drac * 100
                drac_changes.append(drac_change_pct)
            else:
                # No DRAC = no change = 0%
                drac_changes.append(0.0)
        
        total_ttc_change_str = f"{np.mean(ttc_changes):+.1f}%"
        total_drac_change_str = f"{np.mean(drac_changes):+.1f}%"
        total_duration_change_str = f"{total_proximity_duration_change:+.1f}%"
        total_speed_change_str = f"{total_proximity_speed_change:+.1f}%"
        
        table_data.append([
            "TOTAL (AVERAGE)",
            f"{total_ego_duration_change:+.1f}%",
            f"{total_ego_speed_change:+.1f}%",
            f"{total_proximity_count:.0f}",
            total_duration_change_str,
            total_speed_change_str,
            f"{total_proximity_conflicts_change:+.1f}",
            total_ttc_change_str,
            total_drac_change_str
        ])
    
    # Add Barrandov separately (after TOTAL since it's not included in total)
    if 'BARRANDOV' in ego_results:
        ego_data = ego_results['BARRANDOV']
        ego_comparison = ego_data['ego_comparison']
        proximity_analysis = ego_data['proximity_analysis']
        
        # EGO vehicle metrics
        ego_duration_change = ego_comparison['duration_diff_pct']
        ego_speed_change = ego_comparison['speed_diff_pct']
        
        # Proximity metrics
        proximity_count = proximity_analysis['num_proximity_vehicles']
        proximity_duration_change = proximity_analysis['duration_diff_pct']
        proximity_speed_change = proximity_analysis['speed_diff_pct']
        proximity_conflicts_change = proximity_analysis['conflicts_diff']
        
        # Calculate percentage changes for TTC and DRAC
        proximity_min_ttc_bench = proximity_analysis['benchmark_min_ttc']
        proximity_min_ttc_config = proximity_analysis['config_min_ttc']
        proximity_max_drac_bench = proximity_analysis['benchmark_max_drac']
        proximity_max_drac_config = proximity_analysis['config_max_drac']
        
        # Calculate TTC percentage change (handle infinite values)
        if np.isinf(proximity_min_ttc_bench) and np.isinf(proximity_min_ttc_config):
            ttc_change_str = "N/A"
        elif np.isinf(proximity_min_ttc_bench):
            ttc_change_str = "N/A"
        elif proximity_min_ttc_bench != 0:
            ttc_change_pct = (proximity_min_ttc_config - proximity_min_ttc_bench) / proximity_min_ttc_bench * 100
            ttc_change_str = f"{ttc_change_pct:+.1f}%"
        else:
            ttc_change_str = "N/A"
        
        # Calculate DRAC percentage change
        if proximity_max_drac_bench != 0:
            drac_change_pct = (proximity_max_drac_config - proximity_max_drac_bench) / proximity_max_drac_bench * 100
            drac_change_str = f"{drac_change_pct:+.1f}%"
        else:
            drac_change_str = "N/A"
        
        table_data.append([
            "BARRANDOV",
            f"{ego_duration_change:+.1f}%" if not np.isnan(ego_duration_change) else "N/A",
            f"{ego_speed_change:+.1f}%" if not np.isnan(ego_speed_change) else "N/A",
            f"{proximity_count}",
            f"{proximity_duration_change:+.1f}%" if not np.isnan(proximity_duration_change) and proximity_count > 0 else "N/A",
            f"{proximity_speed_change:+.1f}%" if not np.isnan(proximity_speed_change) and proximity_count > 0 else "N/A",
            f"{proximity_conflicts_change:+d}",
            ttc_change_str,
            drac_change_str
        ])
    
    # Save table to file
    table_file = os.path.join(output_dir, "ego_analysis_table.txt")
    with open(table_file, 'w') as f:
        f.write(f"EGO VEHICLE PROXIMITY ANALYSIS TABLE - Configuration {config_id or 'Unknown'}\n")
        f.write("=" * 80 + "\n\n")
        f.write("Note: SSM metrics columns show safety measures for the entire scenario.\n")
        f.write("Conflicts = total interactions, TTC = Time to Collision, DRAC = Deceleration Rate to Avoid Collision.\n\n")
        
        # Calculate column widths
        col_widths = []
        for col in range(len(table_data[0])):
            max_width = max(len(str(row[col])) for row in table_data)
            col_widths.append(max_width + 2)
        
        # Write table
        for i, row in enumerate(table_data):
            if i == 0:  # Header
                f.write(" | ".join(f"{cell:<{col_widths[j]}}" for j, cell in enumerate(row)) + "\n")
                f.write("-" * sum(col_widths) + "-" * (len(col_widths) * 3) + "\n")
            else:
                f.write(" | ".join(f"{cell:<{col_widths[j]}}" for j, cell in enumerate(row)) + "\n")
    
    print(f"EGO analysis table saved to {table_file}")
    
    # Also create  version
    try:
        import matplotlib.pyplot as plt
        import matplotlib.patches as patches
        
        # Create figure and axis
        fig, ax = plt.subplots(figsize=(16, 8))
        ax.axis('tight')
        ax.axis('off')
        
        # Create table
        table = ax.table(cellText=table_data[1:], colLabels=table_data[0], 
                        cellLoc='center', loc='center', 
                        colWidths=[0.12, 0.12, 0.12, 0.12, 0.12, 0.12, 0.10, 0.10, 0.10])
        
        # Style the table
        table.auto_set_font_size(False)
        table.set_fontsize(9)
        table.scale(1, 2)
        
        # Color header row
        for i in range(len(table_data[0])):
            table[(0, i)].set_facecolor('#4CAF50')
            table[(0, i)].set_text_props(weight='bold', color='white')
        
        # Helper function to get color based on value and metric type
        def get_color_for_change(value_str, metric_type):
            """Get color based on value and whether decrease/increase is good"""
            if value_str == "N/A" or value_str == "0":
                return '#ffffff'  # White for N/A or zero
            
            try:
                # Extract numeric value
                if '%' in value_str:
                    value = float(value_str.replace('%', '').replace('+', ''))
                else:
                    value = float(value_str.replace('+', ''))
                
                # Determine if change is good or bad based on metric type
                is_good = False
                if metric_type in ['ego_duration', 'proximity_duration']:
                    # Duration: decrease is good (negative is good)
                    is_good = value < 0
                elif metric_type in ['ego_speed', 'proximity_speed']:
                    # Speed: increase is good (positive is good)
                    is_good = value > 0
                elif metric_type == 'conflicts':
                    # Conflicts: decrease is good (negative is good)
                    is_good = value < 0
                elif metric_type == 'ttc':
                    # TTC: increase is good (positive is good) - more time to collision is safer
                    is_good = value > 0
                elif metric_type == 'drac':
                    # DRAC: decrease is good (negative is good) - less severe braking needed
                    is_good = value < 0
                
                # Calculate intensity based on absolute value (cap at reasonable values)
                abs_value = abs(value)
                if metric_type in ['ego_duration', 'proximity_duration', 'ego_speed', 'proximity_speed', 'ttc', 'drac']:
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
        metric_types = ['scenario', 'ego_duration', 'ego_speed', 'proximity_vehicles', 
                       'proximity_duration', 'proximity_speed', 'conflicts', 'ttc', 'drac']
        
        for i in range(1, len(table_data)):
            for j in range(len(table_data[0])):
                if j == 0:  # Scenario name column - no coloring
                    pass  # Keep default white background
                elif j == 3:  # Proximity vehicles count column (no coloring)
                    pass  # Keep default white background
                else:  # Metric columns
                    cell_value = table_data[i][j]
                    metric_type = metric_types[j] if j < len(metric_types) else 'other'
                    color = get_color_for_change(cell_value, metric_type)
                    table[(i, j)].set_facecolor(color)
                    
                # Make TOTAL row bold but don't override colors
                if table_data[i][0] == "TOTAL (AVERAGE)":
                    table[(i, j)].set_text_props(weight='bold')
        
        # Set title and note
        plt.title(f'EGO VEHICLE ANALYSIS TABLE - Configuration {config_id or "Unknown"}', fontsize=16, fontweight='bold', pad=20)
        
        # Save PNG
        png_file = os.path.join(output_dir, "ego_analysis_table.png")
        plt.savefig(png_file, dpi=300, bbox_inches='tight', facecolor='white')
        plt.close()
        
        print(f"EGO analysis table PNG saved to {png_file}")
        
    except ImportError:
        print("matplotlib not available, skipping PNG generation")
    except Exception as e:
        print(f"Error generating PNG: {e}")
    
    return table_file

def main():
    """Main function to run the comparison analysis"""
    parser = argparse.ArgumentParser(description='Compare SUMO traffic simulation configuration against benchmarks')
    parser.add_argument('config_id', help='Configuration ID to compare against benchmarks (e.g., 33333)')
    parser.add_argument('--no-plots', action='store_true', help='Skip creating visualizations')
    parser.add_argument('--no-save', action='store_true', help='Skip saving results')
    parser.add_argument('--no-ego', action='store_true', help='Skip EGO vehicle analysis')
    
    args = parser.parse_args()
    
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
    
    # Create comparator and run analysis
    comparator = BenchmarkComparator(config_id=args.config_id, groups=groups)
    results = comparator.run_comparison(
        create_plots=not args.no_plots,
        save_results=not args.no_save,
        include_ego_analysis=not args.no_ego
    )
    
    return results

if __name__ == "__main__":
    main() 