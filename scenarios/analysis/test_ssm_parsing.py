#!/usr/bin/env python3
"""
Test script for SSM parsing functionality
"""

import os
import sys
from traffic_analysis import BenchmarkComparator

def test_ssm_parsing():
    """Test SSM parsing with a sample configuration"""
    print("Testing SSM parsing functionality...")
    
    # Create a test comparator
    comparator = BenchmarkComparator("11111")
    
    # Test SSM file parsing
    test_ssm_file = "ssm_output/straight/11111.ssm.xml"
    
    if os.path.exists(test_ssm_file):
        print(f"Found SSM file: {test_ssm_file}")
        
        # Test the SSM parsing method
        ssm_metrics = comparator._load_ssm_data(test_ssm_file)
        
        if ssm_metrics:
            print("SSM parsing successful!")
            print(f"  Total interactions: {ssm_metrics.total_interactions}")
            print(f"  Max Braking Rate: {ssm_metrics.max_br:.2f} m/s²")
            print(f"  Min Space Gap: {ssm_metrics.min_sgap:.2f} s")
            print(f"  Min Time Gap: {ssm_metrics.min_tgap:.2f} s")
            print(f"  BR threshold violations: {ssm_metrics.br_threshold_violations}")
            print(f"  SGAP threshold violations: {ssm_metrics.sgap_threshold_violations}")
            print(f"  TGAP threshold violations: {ssm_metrics.tgap_threshold_violations}")
        else:
            print("SSM parsing failed!")
    else:
        print(f"SSM file not found: {test_ssm_file}")
        print("Please ensure the SSM output files are available for testing.")
    
    # Test benchmark loading
    print("\nTesting benchmark loading...")
    if comparator.load_benchmarks():
        print("Benchmark loading successful!")
        for scenario, benchmark in comparator.benchmarks.items():
            print(f"  {scenario}: {benchmark.ssm_metrics.total_interactions} interactions")
    else:
        print("Benchmark loading failed!")
    
    # Test config loading
    print("\nTesting config loading...")
    if comparator.load_config_data():
        print("Config loading successful!")
        for scenario, config in comparator.config_metrics.items():
            print(f"  {scenario}: {config.ssm_metrics.total_interactions} interactions")
    else:
        print("Config loading failed!")

if __name__ == "__main__":
    test_ssm_parsing() 