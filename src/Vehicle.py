import logging
import xml.etree.ElementTree as ET

def find_aggression(xml_data, entity_ref):
    # XML-Daten analysieren
    tree = ET.parse(xml_data)
    root = tree.getroot()

    # Nach dem gewünschten Element suchen
    aggression_value = None
    for private_elem in root.findall(".//Private[@entityRef='" + entity_ref + "']"):
        for aggression_elem in private_elem.findall(".//Controller[@name='AggressiveController']"):
            for property_elem in aggression_elem.findall(".//Property[@name='aggression']"):
                aggression_value = float(property_elem.attrib['value']) if 'value' in property_elem.attrib else None
                print(aggression_value)
    return aggression_value
def find_positions(xml_data, entity_ref):
    # XML-Daten analysieren
    tree = ET.parse(xml_data)
    root = tree.getroot()

    # Nach dem gewünschten Element suchen
    positions = []
    for private_elem in root.findall(".//Private[@entityRef='" + entity_ref + "']"):
        for position_elem in private_elem.findall(".//WorldPosition"):
            position = {
                'x': float(
                    position_elem.attrib['x']) if position_elem is not None and 'x' in position_elem.attrib else None,
                'y': float(
                    position_elem.attrib['y']) if position_elem is not None and 'y' in position_elem.attrib else None,
                'z': float(
                    position_elem.attrib['z']) if position_elem is not None and 'z' in position_elem.attrib else None,
                'h': float(
                    position_elem.attrib['h']) if position_elem is not None and 'h' in position_elem.attrib else None
            }
            positions.append(position)
    return positions
def extract_traveled_distance_condition(root, entity_ref):
    traveled_distance_condition = None
    stop_triggers = root.findall('.//StopTrigger')

    for stop_trigger in stop_triggers:
        conditions = stop_trigger.findall('.//Condition')
        for condition in conditions:
            entity_conditions = condition.findall('.//ByEntityCondition')
            for entity_condition in entity_conditions:
                entity_refs = entity_condition.findall('.//EntityRef[@entityRef="{}"]'.format(entity_ref))
                for entity_ref_elem in entity_refs:
                    for parent in condition.iter():
                        traveled_distance_condition_elem = parent.find('.//TraveledDistanceCondition')
                        if traveled_distance_condition_elem is None:
                            traveled_distance_condition_elem = parent.find('.//TraveledDistanceCondition')
                        if traveled_distance_condition_elem is not None:
                            traveled_distance_condition = float(traveled_distance_condition_elem.attrib['value'])
                            return traveled_distance_condition
    return traveled_distance_condition



def extract_vehicle_objects(xml_file):
    tree = ET.parse(xml_file)
    root = tree.getroot()

    scenario_objects = []

    # Durchlaufe alle ScenarioObjects
    for scenario_object in root.findall('.//ScenarioObject'):
        obj_name = scenario_object.attrib['name']
        #print("obj_name: " + obj_name)
        positions = find_positions(xml_file, obj_name)
        if not positions:
            logging.info("Keine Positionen gefunden oder leere Liste.")
            positions = [None]
        # Extrahiere die Fahrzeugdetails
        vehicle = scenario_object.find('Vehicle')
        if vehicle is not None:
            vehicle_name = vehicle.attrib['name']
            if vehicle_name != "etk800":
                logging.INFO("vehicle not found, default vehicle (etk800) used")
                vehicle_name = "etk800"
            vehicle_category = vehicle.attrib['vehicleCategory']
            aggression_value = None
            aggression_value = find_aggression(xml_file, obj_name)
            # Extrahiere Performance-Details
            performance = None
            performance = vehicle.find('Performance')
            max_speed = None
            max_acceleration = None
            if performance is not None:
                max_acceleration = None

                if 'maxAcceleration' in performance.attrib:
                    try:
                        max_acceleration = float(performance.attrib['maxAcceleration'])
                    except ValueError:
                        print("maxAcceleration konnte nicht in einen Float umgewandelt werden")
                if 'maxSpeed' in performance.attrib:
                    try:
                        max_speed = float(performance.attrib['maxSpeed'])
                    except ValueError:
                        print("maxSpeed konnte nicht in einen Float umgewandelt werden")

            # Extrahiere Eigenschaften
            properties = vehicle.find('Properties')
            vehicle_type = None
            vehicle_color = None
            if properties is not None:
                for prop in properties.findall('Property'):
                    if prop.attrib['name'] == 'type':
                        vehicle_type = prop.attrib['value']
                    elif prop.attrib['name'] == 'color':
                        vehicle_color = tuple(map(int, prop.attrib['value'].split(',')))

            # Extrahiere Entfernungsinformationen
            traveled_distance_condition = extract_traveled_distance_condition(root, obj_name)
            onepos = None
            if positions and len(positions) > 0:
                onepos = positions[0]
            # Erstelle das Vehicle-Objekt
            vehicle_data = {
                'name': vehicle_name,
                'vehicleCategory': vehicle_category,
                'maxSpeed': max_speed,
                'maxAcceleration': max_acceleration,
                'type': vehicle_type,
                'color': vehicle_color,
                'aggression': aggression_value,
                'traveledDistanceCondition': traveled_distance_condition,
                'X': onepos['x'] if onepos and 'x' in onepos else None,
                'Y': onepos['y'] if onepos and 'y' in onepos else None,
                'Z': onepos['z'] if onepos and 'z' in onepos else None,
                'H': onepos['h'] if onepos and 'h' in onepos else None
            }

            scenario_objects.append({'name': obj_name, 'vehicle': vehicle_data})

    return scenario_objects
# Test
xml_file_path = "C:\\Users\\stefan\\Downloads\\FollowLeadingVehicle2.xosc"
scenario_objects = extract_vehicle_objects(xml_file_path)
#for obj in scenario_objects:
    #print(obj)
