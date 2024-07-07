import os
import sys
import math
import random
import numpy as np
import xml.etree.ElementTree as ET
from time import sleep
from typing import List, Dict, Tuple

from beamngpy import BeamNGpy, Scenario, Vehicle, Road
from beamngpy.sensors import Lidar, Radar, State, Damage, Camera, RoadsSensor

from Condition import parse_conditions_from_xml
from Vehicle import extract_vehicle_objects
from Event import parse_events_from_xml
from Act import parse_acts_from_xml
from Weather import Weather, extract_weather_info
from updateZIP import extend_file_in_zip, fillWeatherInXML, fillWaypointInXML, remove_line_from_file_in_zip, removeWaypointInXML

xml_file_path = "C:\\Users\\stefan\\Downloads\\FollowLeadingVehicle4.xosc"
vehicles = []
startPositions = []

def extract_Parameter(file_path):
    tree = ET.parse(file_path)
    root = tree.getroot()

    parameter_declarations = {}
    for param_decl in root.findall('.//ParameterDeclaration'):
        name = param_decl.attrib.get('name')
        param_type = param_decl.attrib.get('parameterType')
        value = param_decl.attrib.get('value')
        parameter_declarations[name] = {'type': param_type, 'value': value}

    return parameter_declarations

def replace_Parameter(input_file_path, parameter_declarations):
    file_name, file_extension = os.path.splitext(input_file_path)
    output_file_path = file_name + '_repl_para' + file_extension

    with open(input_file_path, 'r') as file:
        content = file.read()

    for name, param in parameter_declarations.items():
        content = content.replace('$' + name, str(param['value']))

    with open(output_file_path, 'w') as file:
        file.write(content)

    return output_file_path

def is_car_on_road(car_position: Tuple[float, float, float], road_edges: List[Dict[str, List[Tuple[float, float, float]]]]) -> bool:
    for edge_triplet in road_edges:
        left_edge = edge_triplet['left']
        right_edge = edge_triplet['right']
        if left_edge[0][0] <= car_position[0] <= right_edge[0][0] and \
           left_edge[0][1] <= car_position[1] <= left_edge[1][1]:
            return True
    return False

def vehicle_pos(name, position):
    vehiclepos = (name, position)
    startPositions.append(vehiclepos)

def main():
    parameters = sys.argv[2:]
    xml_file_paths = [os.path.join(sys.argv[1], param) for param in parameters]
    xml_file_paths.append("C:\\Users\\stefan\\Downloads\\FollowLeadingVehicle4.xosc")
    print("test")
    print(xml_file_paths)
    for xml_file_path in xml_file_paths:
        conditions = parse_conditions_from_xml(xml_file_path)
        acts = parse_acts_from_xml(xml_file_path)
        xmlVehicles = extract_vehicle_objects(xml_file_path)
        events = parse_events_from_xml(xml_file_path)
        print(events[0])
        parameters = extract_Parameter(xml_file_path)
        xml_file_path = replace_Parameter(xml_file_path, parameters)
        waypointLines = []
        waypointCounter = 1
        waypointNames = []
        for event in events:
            if event.action.action_type == "RoutingAction":
                for waypoint in event.action.routing_action.waypoints:
                    waypointString = f"[{waypoint['x']}, {waypoint['y']}, {waypoint['z']}]"
                    waypointName = f"waypoint{waypointCounter}"
                    waypointNames.append(waypointName)
                    waypointLine = fillWaypointInXML(waypointName, waypointString)
                    waypointCounter += 1
                    waypointLines.append(waypointLine)

        weather = extract_weather_info(xml_file_path)
        fillWeatherInXML(weather)

        bng = BeamNGpy('localhost', 64256, home='C:\\Users\\stefan\\Documents\\BeamNG.tech.v0.31.3.0\\BeamNG.tech.v0.31.3.0', user='C:\\Users\\stefan\\AppData\\Local\\BeamNG.drive')
        bng.open()

        scenario = Scenario('west_coast_usa', 'example')

        for vehicle in xmlVehicles:
            v = Vehicle(vehicle.get('name'), model=vehicle.get('vehicle').get('name'), license=vehicle.get('name'))
            vehicles.append(v)
            scenario.add_vehicle(v, pos=(vehicle.get('vehicle').get('X'), vehicle.get('vehicle').get('Y'), vehicle.get('vehicle').get('Z')), cling=True, rot_quat=(0, 0, 0.3826834, 0.9238795))

        scenario.make(bng)
        bng.set_tod(str(weather.time_of_day))
        bng.scenario.load(scenario)
        bng.scenario.start()
        bng.set_weather_preset('xml_weather')
        bng.set_tod(weather.time_of_day)

        for vehicle in vehicles:
            vehicle.sensors.poll()
            vehicle_pos(vehicle.vid, vehicle.state['pos'])
        print('sleep')
        sleep(5)

        for vehicle in xmlVehicles:
            for vehicle2 in vehicles:
                if vehicle2.vid == vehicle.get('name'):
                    vehicle2.ai.drive_in_lane(True)
                    vehicle2.ai.set_mode('span')
                    vehicle2.set_velocity(vehicle.get('vehicle').get('maxSpeed') / 3.6)
                    vehicle2.set_color(vehicle.get('vehicle').get('color'))
                    damage_sensor = Damage()
                    vehicle2.attach_sensor('damage', damage_sensor)

        handle_conditions(vehicles, conditions, events, acts, parameters, waypointNames)

        check_collision(vehicles)

        for waypointLine in waypointLines:
            removeWaypointInXML(waypointLine)

        #bng.close()

def handle_conditions(vehicles, conditions, events, acts, parameters, waypointNames):
    startConditionFinished = not any(cond.main_condition_type == 'OverallStartCondition' for cond in conditions)

    for condition in conditions:
        if condition.main_condition_type == 'EndCondition' and condition.condition_type == 'TraveledDistance':
            tdstartpos = get_vehicle_position(vehicles, condition.mainEntityRef)
        elif condition.main_condition_type == 'OverallStartCondition' and condition.condition_type == 'TraveledDistance':
            oscondstartpos = get_vehicle_position(vehicles, condition.mainEntityRef)
        elif condition.main_condition_type == 'OverallStartCondition' and condition.condition_type == 'ReachPosition':
            startReachCondition = condition
        elif condition.main_condition_type == 'EndCondition' and condition.condition_type == 'ReachPosition':
            endReachCondition = condition

    finishedCondition = None
    finishedEvents = []
    lastFinishedEvent = None

    while not startConditionFinished:
        startConditionFinished = check_start_conditions(vehicles, conditions, parameters)
    print ("bin hier")
    while not any(cond.main_condition_type == 'EndCondition' and cond.condition_type == 'TraveledDistance' for cond in conditions):
        for vehicle in vehicles:
            vehicle.sensors.poll()

            for act in acts:
                for event in act.events:
                    if vehicle.vid in act.actors and event.name not in finishedEvents:
                        handle_event(event, vehicle, vehicles, parameters, finishedEvents, lastFinishedEvent, waypointNames)

        if endConditionFinished(conditions, vehicles):
            break

def get_vehicle_position(vehicles, ref):
    for vehicle in vehicles:
        if vehicle.vid == ref:
            return vehicle.state['pos']
    return None

def check_start_conditions(vehicles, conditions, parameters):
    for condition in conditions:
        if condition.main_condition_type == 'OverallStartCondition':
            for vehicle in vehicles:
                if condition.mainEntityRef == vehicle.vid:
                    if condition.condition_type == 'TraveledDistance':
                        return check_traveled_distance(vehicle, condition, parameters)
                    elif condition.condition_type == 'ReachPosition':
                        return check_reach_position(vehicle, condition)
                    elif condition.condition_type == 'Speed':
                        return check_speed_condition(vehicle, condition)
    return False

def check_traveled_distance(vehicle, condition, parameters):
    pos1 = vehicle.state['pos']
    traveled_distance = np.linalg.norm(np.array(pos1) - condition.start_position)
    return traveled_distance >= condition.value

def check_reach_position(vehicle, condition):
    pos1 = vehicle.state['pos']
    return condition.position[0] - condition.tolerance <= pos1[0] <= condition.position[0] + condition.tolerance and \
           condition.position[1] - condition.tolerance <= pos1[1] <= condition.position[1] + condition.tolerance and \
           condition.position[2] - condition.tolerance <= pos1[2] <= condition.position[2] + condition.tolerance

def check_speed_condition(vehicle, condition):
    speed = vehicle.state['vel']
    if condition.rule == 'greaterThan':
        return speed > condition.value
    elif condition.rule == 'lessThan':
        return speed < condition.value
    elif condition.rule == 'equals':
        return speed == condition.value
    return False

def handle_event(event, vehicle, vehicles, parameters, finishedEvents, lastFinishedEvent, waypointNames):
    if event.condition.condition_type == 'RelativeDistance':
        handle_relative_distance(event, vehicle, vehicles, parameters, finishedEvents, lastFinishedEvent)
    elif event.condition.condition_type == 'StoryboardElementState':
        handle_storyboard_element_state(event, vehicle, finishedEvents, lastFinishedEvent)
    elif event.condition.condition_type == 'ReachPosition':
        handle_reach_position(event, vehicle, waypointNames, finishedEvents, lastFinishedEvent)

def handle_relative_distance(event, vehicle, vehicles, parameters, finishedEvents, lastFinishedEvent):
    ref_vehicle = next((v for v in vehicles if v.vid == event.condition.EntityRef), None)
    if ref_vehicle:
        relative_distance = np.linalg.norm(np.array(vehicle.state['pos']) - np.array(ref_vehicle.state['pos']))
        if relative_distance <= event.condition.value:
            execute_event(event, vehicle, finishedEvents, lastFinishedEvent)

def handle_storyboard_element_state(event, vehicle, finishedEvents, lastFinishedEvent):
    if event.condition.StoryboardElementRef == lastFinishedEvent and event.condition.state == "completeState":
        execute_event(event, vehicle, finishedEvents, lastFinishedEvent)

def handle_reach_position(event, vehicle, waypointNames, finishedEvents, lastFinishedEvent):
    pos1 = vehicle.state['pos']
    if event.condition.position[0] - event.condition.tolerance <= pos1[0] <= event.condition.position[0] + event.condition.tolerance and \
       event.condition.position[1] - event.condition.tolerance <= pos1[1] <= event.condition.position[1] + event.condition.tolerance and \
       event.condition.position[2] - event.condition.tolerance <= pos1[2] <= event.condition.position[2] + event.condition.tolerance:
        execute_event(event, vehicle, finishedEvents, lastFinishedEvent)

def execute_event(event, vehicle, finishedEvents, lastFinishedEvent):
    for action in event.actions:
        if action.type == 'SpeedAction':
            vehicle.ai.set_mode('manual')
            vehicle.ai.set_speed(action.speedAction.TargetSpeed)
        elif action.type == 'RoutingAction':
            vehicle.ai.set_waypoint(action.routing_action.waypoints)
        finishedEvents.append(event.name)
        lastFinishedEvent = event.condition.StoryboardElementRef

def endConditionFinished(conditions, vehicles):
    for condition in conditions:
        if condition.main_condition_type == 'EndCondition':
            if condition.condition_type == 'ReachPosition':
                return check_reach_position(vehicles, condition)
            elif condition.condition_type == 'TraveledDistance':
                return check_traveled_distance(vehicles, condition)
            elif condition.condition_type == 'Speed':
                return check_speed_condition(vehicles, condition)
    return False

def check_collision(vehicles):
    collision_happened = False
    for vehicle in vehicles:
        for damage in vehicle.sensors['damage'].data:
            if damage['damage'] > 0:
                collision_happened = True
                break
        if collision_happened:
            break
    return collision_happened

if __name__ == "__main__":
    main()