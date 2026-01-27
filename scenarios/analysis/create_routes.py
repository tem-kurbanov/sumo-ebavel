"""
Route file generator for EGO vehicle configurations.

Generates per-configuration .rou.xml files by injecting EGO vType parameters
and, optionally, additional vehicles into a base route file.
"""
import argparse
import itertools
import xml.etree.ElementTree as ET
import os
from pathlib import Path
from typing import List, Dict, Optional
import multiprocessing as mp
from functools import partial

def specify_parameters():
    """Define parameter groups for configuration permutations."""
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

    return groups


def generate_configurations(basic_driver, groups):
    """Generate all permutations of driver parameters."""
    permutations = []
    
    # Get the number of values for each group
    group_sizes = [len(list(group.values())[0]) for group in groups]
    
    # Generate all possible combinations where each position uses values from 1 to its group size
    for combo in itertools.product(*[range(1, size + 1) for size in group_sizes]):
        permutations.append(tuple(combo))
    
    configurations = {}

    for perm in permutations:
        config = basic_driver.copy()
        for i, group in enumerate(groups):
            for key, value in group.items():
                config[key] = value[perm[i] - 1]
        configurations[perm] = config

    print(f"Generated {len(configurations)} configurations")
    return configurations


def choose_vehicles(scenario):
    """Return vehicle IDs to convert to EGO for a given scenario."""
    vehicle_sets = {}
    vehicle_sets["straight"] = ["1000", "1001", "1002", "1003", "1004", "1005", "1006", "1007", "1008", "1009", "1010"]
    vehicle_sets["merge"] = ["1000", "1001", "1002", "1003", "1004", "1005", "1006", "1007", "1008", "1009", "1010"]
    vehicle_sets["secondary_merge"] = ["1000", "1001", "1002", "1003", "1004", "1005", "1006", "1007", "1008", "1009", "1010"]

    return vehicle_sets[scenario]

def add_vehicles_to_route_file(
    route_file_path: str,
    new_vehicles: List[Dict],
    output_file_path: Optional[str] = None
):
    """
    Add new vehicles to an existing SUMO route file, maintaining chronological order by departure time.
    
    Args:
        route_file_path: Path to the existing .rou.xml file
        new_vehicles: List of dictionaries containing vehicle information
                     Each dict should have: 'id', 'type', 'depart', 'route_edges'
                     Optional: 'color', 'departLane', 'departSpeed', etc.
        output_file_path: Path for the output file (if None, overwrites input file)
    """
    try:
        # Parse the existing route file
        tree = ET.parse(route_file_path)
        root = tree.getroot()
        
        # Find the routes element (parent of all vehicles)
        routes_elem = root
        
        # Get all existing vehicles and their departure times
        existing_vehicles = []
        for vehicle in routes_elem.findall('vehicle'):
            depart_time = float(vehicle.get('depart', '0'))
            existing_vehicles.append((depart_time, vehicle))
        
        # Sort existing vehicles by departure time
        existing_vehicles.sort(key=lambda x: x[0])
        
        # Remove existing vehicles from the XML tree (we'll re-add them in order)
        for _, vehicle in existing_vehicles:
            routes_elem.remove(vehicle)
        
        # Create new vehicle elements
        new_vehicle_elements = []
        for vehicle_info in new_vehicles:
            # Create vehicle element
            vehicle_elem = ET.Element('vehicle')
            
            # Add required attributes
            vehicle_elem.set('id', str(vehicle_info['id']))
            vehicle_elem.set('type', vehicle_info['type'])
            vehicle_elem.set('depart', str(vehicle_info['depart']))
            
            # Add optional attributes if provided
            optional_attrs = ['color', 'departLane', 'departSpeed', 'arrivalLane', 'arrivalSpeed']
            for attr in optional_attrs:
                if attr in vehicle_info:
                    vehicle_elem.set(attr, str(vehicle_info[attr]))
            
            # Create route element
            route_elem = ET.SubElement(vehicle_elem, 'route')
            route_elem.set('edges', vehicle_info['route_edges'])
            
            # Add SSM device parameter if this is an EGO vehicle
            if vehicle_info['type'] == 'EGO':
                ssm_param = ET.SubElement(vehicle_elem, 'param')
                ssm_param.set('key', 'has.ssm.device')
                ssm_param.set('value', 'true')
            
            new_vehicle_elements.append((float(vehicle_info['depart']), vehicle_elem))
        
        # Combine and sort all vehicles by departure time
        all_vehicles = existing_vehicles + new_vehicle_elements
        all_vehicles.sort(key=lambda x: x[0])
        
        # Add all vehicles back to the XML tree in chronological order
        for _, vehicle_elem in all_vehicles:
            routes_elem.append(vehicle_elem)
        
        # Write the modified file with proper formatting
        output_path = output_file_path if output_file_path else route_file_path
        
        # Create a properly formatted XML string
        xml_str = ET.tostring(root, encoding='unicode')
        
        # Add proper indentation for better readability
        import xml.dom.minidom
        dom = xml.dom.minidom.parseString(xml_str)
        pretty_xml = dom.toprettyxml(indent="    ")
        
        # Clean up excessive empty lines
        lines = pretty_xml.split('\n')
        cleaned_lines = []
        for line in lines:
            # Keep non-empty lines and lines with content
            if line.strip() or any(char in line for char in ['<', '>', '?']):
                cleaned_lines.append(line)
        
        # Write the cleaned formatted XML
        with open(output_path, 'w', encoding='UTF-8') as f:
            f.write('\n'.join(cleaned_lines))
        
        print(f"Successfully added {len(new_vehicles)} vehicles to {output_path}")
        print(f"Total vehicles in file: {len(all_vehicles)}")
        
    except ET.ParseError as e:
        print(f"Error parsing XML file: {e}")
    except FileNotFoundError:
        print(f"File not found: {route_file_path}")
    except Exception as e:
        print(f"Error adding vehicles: {e}")


def create_vehicle_dict(
    vehicle_id: str,
    vehicle_type: str,
    depart_time: float,
    route_edges: str,
    color: Optional[str] = None,
    depart_lane: Optional[str] = None,
    depart_speed: Optional[str] = None
) -> Dict:
    """
    Create a vehicle dictionary for adding to route file.
    
    Args:
        vehicle_id: Unique vehicle identifier
        vehicle_type: Vehicle type (must be defined in the route file)
        depart_time: Departure time in seconds
        route_edges: Space-separated list of edge IDs
        color: Optional color specification (e.g., "255,0,0" for red)
        depart_lane: Optional departure lane specification
        depart_speed: Optional departure speed specification
        
    Returns:
        Dictionary containing vehicle information
    """
    vehicle_dict = {
        'id': vehicle_id,
        'type': vehicle_type,
        'depart': depart_time,
        'route_edges': route_edges
    }
    
    if color:
        vehicle_dict['color'] = color
    if depart_lane:
        vehicle_dict['departLane'] = depart_lane
    if depart_speed:
        vehicle_dict['departSpeed'] = depart_speed
    
    return vehicle_dict


def create_ego_route_file(
    scenario,
    initial_route_file,
    configuration,
    vehicles,
    perm,
    new_vehicles=None,
    output_dir=None
):
    """
    Read the initial route file, add EGO vType with given configuration,
    update specified vehicles to use EGO vType, optionally add new vehicles,
    and save to new file.
    
    Args:
        initial_route_file: Path to the original route file
        configuration: Dictionary with driver parameters for EGO vType
        vehicles: List of vehicle IDs to update to EGO vType
        perm: Configuration key (tuple) to use in filename
        new_vehicles: Optional list of dictionaries containing new vehicle information
                     Each dict should have: 'id', 'type', 'depart', 'route_edges'
                     Optional: 'color', 'departLane', 'departSpeed', etc.
    """
    # Parse the XML file
    tree = ET.parse(initial_route_file)
    root = tree.getroot()
    
    # Create new EGO vType element
    ego_vtype = ET.Element('vType')
    ego_vtype.set('id', 'EGO')
    ego_vtype.set('vClass', 'passenger')
    
    # Add all configuration parameters as attributes
    for key, value in configuration.items():
        ego_vtype.set(key, value)
    
    # Insert EGO vType after the first vType (to keep it near the top)
    if len(root) > 0:
        root.insert(1, ego_vtype)
        # Add newline after EGO vType
        root.insert(2, ET.Comment(''))
    else:
        root.append(ego_vtype)
        # Add newline after EGO vType
        root.append(ET.Comment(''))
    
    # Update specified vehicles to use EGO vType and add SSM device
    for vehicle in root.findall('vehicle'):
        vehicle_id = vehicle.get('id')
        if vehicle_id in vehicles:
            vehicle.set('type', 'EGO')
            
            # Add SSM device parameter to EGO vehicles
            ssm_param = ET.Element('param')
            ssm_param.set('key', 'has.ssm.device')
            ssm_param.set('value', 'true')
            vehicle.append(ssm_param)
    
    # Create output directory if it doesn't exist
    output_dir = Path(output_dir or Path("experiments") / scenario)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # Create filename from perm tuple (remove commas and parentheses)
    perm_str = str(perm).replace('(', '').replace(')', '').replace(',', '').replace(' ', '')
    output_file = output_dir / f"{perm_str}.rou.xml"
    
    # If new vehicles are provided, add them to the route file
    if new_vehicles:
        # Write the current state to a temporary file
        temp_file = output_dir / f"temp_{perm_str}.rou.xml"
        
        # Write with proper formatting
        xml_str = ET.tostring(root, encoding='unicode')
        import xml.dom.minidom
        dom = xml.dom.minidom.parseString(xml_str)
        pretty_xml = dom.toprettyxml(indent="    ")
        
        # Clean up excessive empty lines
        lines = pretty_xml.split('\n')
        cleaned_lines = []
        for line in lines:
            # Keep non-empty lines and lines with content
            if line.strip() or any(char in line for char in ['<', '>', '?']):
                cleaned_lines.append(line)
        
        with open(temp_file, 'w', encoding='UTF-8') as f:
            f.write('\n'.join(cleaned_lines))
        
        # Add new vehicles using the existing function
        add_vehicles_to_route_file(temp_file, new_vehicles, output_file)
        
        # Clean up temporary file
        if temp_file.exists():
            temp_file.unlink()
    else:
        # Write the modified XML to the new file with proper formatting
        xml_str = ET.tostring(root, encoding='unicode')
        import xml.dom.minidom
        dom = xml.dom.minidom.parseString(xml_str)
        pretty_xml = dom.toprettyxml(indent="    ")
        
        # Clean up excessive empty lines
        lines = pretty_xml.split('\n')
        cleaned_lines = []
        for line in lines:
            # Keep non-empty lines and lines with content
            if line.strip() or any(char in line for char in ['<', '>', '?']):
                cleaned_lines.append(line)
        
        with open(output_file, 'w', encoding='UTF-8') as f:
            f.write('\n'.join(cleaned_lines))
    
    print(f"Created route file: {output_file}")


def create_route_file_worker(args):
    """
    Worker function for parallel processing of route file creation.
    
    Args:
        args: Tuple containing (scenario, initial_route_file, config, vehicles, perm, new_vehicles)
    
    Returns:
        Tuple of (perm, success_status, output_file_path)
    """
    scenario, initial_route_file, config, vehicles, perm, new_vehicles, output_dir = args
    
    try:
        create_ego_route_file(
            scenario,
            initial_route_file,
            config,
            vehicles,
            perm,
            new_vehicles,
            output_dir=output_dir
        )
        
        # Create output directory path for return
        output_dir = Path(output_dir or Path("experiments") / scenario)
        perm_str = str(perm).replace('(', '').replace(')', '').replace(',', '').replace(' ', '')
        output_file = output_dir / f"{perm_str}.rou.xml"
        
        return (perm, True, output_file)
    except Exception as e:
        print(f"Error processing configuration {perm}: {e}")
        return (perm, False, str(e))


def parse_args():
    """Parse CLI arguments for route generation."""
    parser = argparse.ArgumentParser(description="Generate EGO route files")
    parser.add_argument("--scenario", default="barrandov", help="Scenario name")
    parser.add_argument(
        "--initial-route-file",
        default=None,
        help="Path to base .rou.xml file"
    )
    parser.add_argument(
        "--output-dir",
        default=None,
        help="Directory for generated .rou.xml files"
    )
    parser.add_argument(
        "--processes",
        type=int,
        default=None,
        help="Number of worker processes (default: min(cpu_count, 8))"
    )
    parser.add_argument(
        "--add-ego-count",
        type=int,
        default=0,
        help="Number of additional EGO vehicles to insert into the route file (default: 0)"
    )
    parser.add_argument(
        "--add-ego-start-time",
        type=float,
        default=1000.0,
        help="Departure time of the first inserted EGO vehicle (default: 1000.0)"
    )
    parser.add_argument(
        "--add-ego-depart-interval",
        type=float,
        default=1.0,
        help="Time delta between inserted EGO vehicles (default: 1.0)"
    )
    parser.add_argument(
        "--add-ego-route-edges",
        type=str,
        default=None,
        help="Space-separated list of edge IDs defining the route for inserted EGO vehicles"
    )
    parser.add_argument(
        "--add-ego-color",
        type=str,
        default="255,255,255",
        help='RGB color string for inserted EGO vehicles, e.g. "255,255,255" (default: 255,255,255)'
    )
    return parser.parse_args()


if __name__ == "__main__":
    # Dictionaries of parameters to define
    args = parse_args()
    scenario = args.scenario
    initial_route_file = args.initial_route_file or str(Path(scenario) / "routes.rou.xml")
    output_dir = args.output_dir or str(Path("experiments") / scenario)
    
    #Basic driver definition
    basic_driver = {"accel": "3.3", "decel": "4.44", "minGap": "2.5", "startupDelay": "0.5", "tau": "1.6", "maxSpeed": "33", "lcStrategic": "1.0", "lcCooperative": "0.5", "lcSpeedGain": "2.0", "lcKeepRight": "0.5", "color": "255,255,255"}

    groups = specify_parameters()

    configurations = generate_configurations(basic_driver, groups)

    vehicles = []


    added_vehicles = []
    if args.add_ego_count and args.add_ego_count > 0:
        if not args.add_ego_route_edges:
            raise SystemExit(
                "Error: --add-ego-route-edges is required when --add-ego-count > 0"
            )
        start_time = float(args.add_ego_start_time)
        vehicle_type = "EGO"
        for v in range(args.add_ego_count):
            added_vehicles.append(
                create_vehicle_dict(
                    vehicle_id=f"EGO_{v}",
                    vehicle_type=vehicle_type,
                    depart_time=start_time,
                    route_edges=args.add_ego_route_edges,
                    color=args.add_ego_color
                )
            )
            start_time += float(args.add_ego_depart_interval)

    # Prepare arguments for parallel processing
    worker_args = []
    for perm, config in configurations.items():
        worker_args.append((scenario, initial_route_file, config, vehicles, perm, added_vehicles, output_dir))
    
    # Determine number of processes (use CPU count, but cap at 8 to avoid overwhelming the system)
    num_processes = args.processes or min(mp.cpu_count(), 8)
    print(f"Using {num_processes} processes for parallel processing")
    print(f"Processing {len(worker_args)} configurations...")
    
    # Create route files using parallel processing
    with mp.Pool(processes=num_processes) as pool:
        results = pool.map(create_route_file_worker, worker_args)
    
    # Report results
    successful = sum(1 for _, success, _ in results if success)
    failed = len(results) - successful
    
    print(f"\nProcessing complete!")
    print(f"Successfully created: {successful} route files")
    print(f"Failed: {failed} route files")
    
    if failed > 0:
        print("\nFailed configurations:")
        for perm, success, error in results:
            if not success:
                print(f"  {perm}: {error}")

# Example usage of the new vehicle addition functionality
    # Uncomment and modify the following code to add vehicles:
    
    # # Create some example vehicles
    # new_vehicles = [
    #     create_vehicle_dict(
    #         vehicle_id="test_vehicle_1",
    #         vehicle_type="CAR",
    #         depart_time=100.0,
    #         route_edges="585 406 586 218 98 477 341 E4 214 79 388 281 102 103 104 296 295 324",
    #         color="255,0,0"  # Red
    #     ),
    #     create_vehicle_dict(
    #         vehicle_id="test_vehicle_2", 
    #         vehicle_type="CAR",
    #         depart_time=105.0,
    #         route_edges="E8 E9 379 83 267 259 617 107 469 161 113 114 391",
    #         color="0,255,0"  # Green
    #     )
    # ]
    # 
    # # Add vehicles to route file
    # add_vehicles_to_route_file(
    #     route_file_path="Barrandov2D/routes.rou.xml",
    #     new_vehicles=new_vehicles,
    #     output_file_path="Barrandov2D/routes_with_new_vehicles.rou.xml"
    # )