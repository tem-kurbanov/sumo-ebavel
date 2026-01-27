#!/usr/bin/env python3
"""
Full Batch Analysis Runner

This script runs the complete batch analysis for all configurations.
It includes proper logging, error handling, and progress tracking.
"""

import os
import sys
import time
import logging
import argparse
from datetime import datetime
from batch_analysis import BatchAnalyzer

def setup_logging():
    """Setup logging configuration"""
    # Create logs directory
    os.makedirs("logs", exist_ok=True)
    
    # Setup logging
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_file = f"logs/batch_analysis_{timestamp}.log"
    
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler(log_file),
            logging.StreamHandler(sys.stdout)
        ]
    )
    
    return log_file

def parse_args():
    """Parse CLI arguments for full batch analysis."""
    parser = argparse.ArgumentParser(description="Run full batch EGO analysis")
    parser.add_argument(
        "--configs-dir",
        default="experiments/barrandov",
        help="Directory containing .rou.xml configurations"
    )
    parser.add_argument(
        "--output-dir",
        default=None,
        help="Output directory (defaults to timestamped name)"
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="Run without interactive confirmation"
    )
    return parser.parse_args()


def main():
    """Run the full batch analysis"""
    print("="*80)
    print("FULL BATCH EGO VEHICLE ANALYSIS")
    print("="*80)
    
    args = parse_args()
    # Setup logging
    log_file = setup_logging()
    logging.info("Starting full batch EGO vehicle analysis")
    
    # Create batch analyzer
    output_dir = args.output_dir or f"batch_analysis_results_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    analyzer = BatchAnalyzer(output_base_dir=output_dir, configs_dir=args.configs_dir)
    
    # Get all configurations
    all_configs = analyzer.get_all_configurations()
    logging.info(f"Found {len(all_configs)} configurations to analyze")
    
    # Estimate time
    estimated_time_per_config = 4  # seconds (based on test run)
    total_estimated_time = len(all_configs) * estimated_time_per_config
    estimated_hours = total_estimated_time // 3600
    estimated_minutes = (total_estimated_time % 3600) // 60
    
    print(f"\nEstimated time: {estimated_hours}h {estimated_minutes}m")
    print(f"Log file: {log_file}")
    print(f"Output directory: {output_dir}")
    
    # Ask for confirmation
    if not args.yes:
        response = input("\nProceed with full analysis? (y/N): ")
        if response.lower() != 'y':
            print("Analysis cancelled.")
            return
    
    # Start the analysis
    start_time = time.time()
    
    try:
        analyzer.run_batch_analysis(config_ids=all_configs)
        
        total_time = time.time() - start_time
        hours = int(total_time // 3600)
        minutes = int((total_time % 3600) // 60)
        seconds = int(total_time % 60)
        
        logging.info(f"Full batch EGO vehicle analysis completed successfully!")
        logging.info(f"Total time: {hours}h {minutes}m {seconds}s")
        logging.info(f"Results saved to: {output_dir}/")
        
        print(f"\n{'='*80}")
        print("FULL BATCH EGO VEHICLE ANALYSIS COMPLETED!")
        print(f"{'='*80}")
        print(f"Total time: {hours}h {minutes}m {seconds}s")
        print(f"Results saved to: {output_dir}/")
        print(f"Log file: {log_file}")
        
    except KeyboardInterrupt:
        logging.warning("Analysis interrupted by user")
        print("\n\nAnalysis interrupted by user.")
        print(f"Partial results saved to: {output_dir}/")
        
    except Exception as e:
        logging.error(f"Analysis failed with error: {str(e)}")
        print(f"\n\nAnalysis failed with error: {str(e)}")
        print(f"Check log file: {log_file}")

if __name__ == "__main__":
    main() 