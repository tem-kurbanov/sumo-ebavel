#!/usr/bin/env python3
"""Debug script to test SSM parsing"""

import xml.etree.ElementTree as ET
import numpy as np

def debug_ssm_parsing(file_path):
    """Debug SSM parsing for a specific file"""
    print(f"Debugging SSM file: {file_path}")
    
    try:
        tree = ET.parse(file_path)
        root = tree.getroot()
        
        # Initialize metrics
        min_ttc_values = []
        max_drac_values = []
        pet_values = []
        total_conflicts = 0
        
        print(f"Root element: {root.tag}")
        
        # Parse globalMeasures for each ego vehicle
        global_measures = root.findall('.//globalMeasures')
        print(f"Found {len(global_measures)} globalMeasures elements")
        
        for i, global_measure in enumerate(global_measures):
            total_conflicts += 1
            print(f"\nProcessing globalMeasures {i+1}:")
            
            # Extract minTTC
            min_ttc_elem = global_measure.find('.//minTTC')
            if min_ttc_elem is not None:
                min_ttc = float(min_ttc_elem.get('value', float('inf')))
                print(f"  minTTC: {min_ttc}")
                if min_ttc != float('inf'):
                    min_ttc_values.append(min_ttc)
            else:
                print("  minTTC: Not found")
            
            # Extract maxDRAC
            max_drac_elem = global_measure.find('.//maxDRAC')
            if max_drac_elem is not None:
                max_drac = float(max_drac_elem.get('value', 0))
                print(f"  maxDRAC: {max_drac}")
                if max_drac > 0:
                    max_drac_values.append(max_drac)
            else:
                print("  maxDRAC: Not found")
            
            # Extract PET
            pet_elem = global_measure.find('.//PET')
            if pet_elem is not None:
                pet_value = pet_elem.get('value', 'NA')
                print(f"  PET: {pet_value}")
                if pet_value != 'NA' and pet_value != '':
                    try:
                        pet = float(pet_value)
                        pet_values.append(pet)
                    except ValueError:
                        print(f"    Invalid PET value: {pet_value}")
            else:
                print("  PET: Not found")
        
        # Calculate averages
        avg_min_ttc = np.mean(min_ttc_values) if min_ttc_values else float('inf')
        avg_max_drac = np.mean(max_drac_values) if max_drac_values else 0.0
        avg_pet = np.mean(pet_values) if pet_values else float('nan')
        
        print(f"\nSUMMARY:")
        print(f"Total conflicts: {total_conflicts}")
        print(f"Min TTC values: {len(min_ttc_values)} valid values")
        print(f"Max DRAC values: {len(max_drac_values)} valid values")
        print(f"PET values: {len(pet_values)} valid values")
        print(f"Average Min TTC: {avg_min_ttc}")
        print(f"Average Max DRAC: {avg_max_drac}")
        print(f"Average PET: {avg_pet}")
        
        if max_drac_values:
            print(f"DRAC values found: {max_drac_values[:10]}...")  # Show first 10
        
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    # Test both benchmark and config files
    print("="*80)
    print("TESTING BENCHMARK FILE")
    print("="*80)
    debug_ssm_parsing("ssm_output/reset/base.ssm.xml")
    
    print("\n" + "="*80)
    print("TESTING CONFIG FILE")
    print("="*80)
    debug_ssm_parsing("ssm_output/reset/11111.ssm.xml") 