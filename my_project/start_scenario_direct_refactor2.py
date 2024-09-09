import math
import os
import random
from time import sleep
from typing import List, Dict, Tuple
import beamngpy.tools
from beamngpy import BeamNGpy, Scenario, Vehicle, Road
from beamngpy.sensors import Lidar
from beamngpy.sensors import Radar
from beamngpy.sensors import State
from beamngpy.vehicle import Vehicle
import numpy as np
from beamngpy.sensors import Damage, Camera
import sys
from beamngpy.sensors import RoadsSensor
import xml.etree.ElementTree as ET
import asyncio
from Condition import parse_conditions_from_xml
from Vehicle import extract_vehicle_objects
from Event import parse_events_from_xml
from Act import parse_acts_from_xml
from Weather import Weather, extract_weather_info
from updateZIP import extend_file_in_zip, fillWeatherInXML, fillWaypointInXML, remove_line_from_file_in_zip, \
    removeWaypointInXML

import math
import time
import pickle
import os
from shapely.geometry import Polygon, Point
from beamngpy import BeamNGpy, Scenario, Vehicle
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPolygon
from beamngpy.sensors import State


# Constants
POLYGON_SAVE_PATH = "road_polygons.pkl"

XML_FILE_PATH = "C:\\Users\\stefan\\Downloads\\FollowLeadingVehicle4.xosc"
BEAMNG_HOME = 'C:\\Users\\stefan\\Music\\BeamNG.tech.v0.31.3.0\\BeamNG.tech.v0.31.3.0'
BEAMNG_USER = 'C:\\Users\\stefan\\AppData\\Local\\BeamNG.drive'
BEAMNG_LEVEL = 'west_coast_usa'
POLYGON_SAVE_PATH = "road_polygons.pkl"  # Dateipfad zum Speichern der Polygone
waypointLines = []
off_the_road_vehicles = []
vehicles = []
startPositions = []

#startReachCondition = False
startTraveledDistanceCondition = False
endTraveledDistanceCondition = False
endReachCondition = False

# Funktion zum Erstellen eines Straßenpolygons aus den Straßenkanten
def create_road_polygon(road_edges):
    left_points = [edge['left'][:2] for edge in road_edges]
    right_points = [edge['right'][:2] for edge in reversed(road_edges)]
    road_polygon = Polygon(left_points + right_points)

    if not road_polygon.is_valid:
        print("Warning: Das Straßenpolygon ist ungültig.")

    return road_polygon


# Funktion zur Überprüfung, ob das Fahrzeug auf einer der Straßen ist
def is_vehicle_on_any_road(vehicle_position, road_polygons):
    vehicle_point = Point(vehicle_position)
    for road_polygon in road_polygons:
        if road_polygon.contains(vehicle_point):
            return True
    return False


# Funktion zur Überwachung der Fahrzeugposition
def monitor_vehicle_position(vehicle, road_polygons):
    vehicle.sensors.poll()
    curr_position = vehicle.state['pos'][:2]  # Fahrzeugposition abrufen (x, y)
    if not is_vehicle_on_any_road(curr_position, road_polygons):
        off_the_road_vehicles.append(vehicle)
        log_off_road(curr_position)
    else:
        print("Vehicle is on the road.")
        print(vehicle.state['pos'])


# Funktion zum Loggen, wenn das Fahrzeug die Straße verlässt
def log_off_road(position):
    print(f"Vehicle went off-road at position: {position}")


# Funktion zum Speichern der Polygone
def save_polygons(road_polygons):
    with open(POLYGON_SAVE_PATH, 'wb') as f:
        pickle.dump(road_polygons, f)
    print(f"Road polygons saved to {POLYGON_SAVE_PATH}")


# Funktion zum Laden der Polygone
def load_polygons():
    if os.path.exists(POLYGON_SAVE_PATH):
        with open(POLYGON_SAVE_PATH, 'rb') as f:
            road_polygons = pickle.load(f)
        print(f"Road polygons loaded from {POLYGON_SAVE_PATH}")
        return road_polygons
    return None


# Funktion zur Initialisierung der Visualisierung
def init_visualization(road_polygons):
    fig, ax = plt.subplots()

    # Zeichne jedes Straßenpolygon
    for polygon in road_polygons:
        if not polygon.is_empty and polygon.is_valid:  # Überprüfe, ob das Polygon gültig und nicht leer ist
            try:
                mpl_poly = MplPolygon(list(polygon.exterior.coords), closed=True, edgecolor='blue', facecolor='none')
                ax.add_patch(mpl_poly)
            except Exception as e:
                print(f"Error plotting polygon: {e}")

    ax.set_aspect('equal', 'box')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Live Vehicle Position on Road Polygons')
    plt.grid(True)

    # Initialisiere den Fahrzeugmarker als roten Punkt
    vehicle_marker, = ax.plot([], [], 'ro', markersize=10, label='Vehicle')

    plt.legend()
    plt.ion()  # Interaktive Mode für Live-Updates aktivieren
    plt.show()

    return fig, ax, vehicle_marker


# Funktion zur Aktualisierung der Fahrzeugposition in der Visualisierung
def update_vehicle_position(vehicle_marker, vehicle_position):
    vehicle_marker.set_data(vehicle_position[0], vehicle_position[1])
    plt.draw()
    plt.pause(0.001)  # Eine kleine Pause für das Rendering

def extract_Parameter(file_path):
    tree = ET.parse(file_path)
    root = tree.getroot()

    parameter_declarations = {}
    for param_decl in root.findall('.//ParameterDeclaration'):
        name = param_decl.attrib.get('name')
        print('name: ')
        print(name)
        param_type = param_decl.attrib.get('parameterType')
        value = param_decl.attrib.get('value')
        parameter_declarations[name] = {'type': param_type, 'value': value}

    return parameter_declarations
def replace_Parameter(input_file_path, parameter_declarations):
    file_name, file_extension = os.path.splitext(input_file_path)
    output_file_path = file_name + '_repl_para' + file_extension
    # Lese den Inhalt der Eingabedatei
    with open(input_file_path, 'r') as file:
        content = file.read()

    # Ersetze die Parameter im Text
    for name, param in parameter_declarations.items():
        content = content.replace('$' + name, str(param['value']))

    # Dateinamen für die Ausgabedatei generieren
    #output_file_path = input_file_path.replace('.xml', '_repl_para.xml')

    # Schreibe den aktualisierten Text in die Ausgabedatei
    with open(output_file_path, 'w') as file:
        file.write(content)

    return output_file_path
def is_car_on_road(car_position: Tuple[float, float, float], road_edges: List[Dict[str, List[Tuple[float, float, float]]]]) -> bool:
    for edge_triplet in road_edges:
        left_edge = edge_triplet['left']
        right_edge = edge_triplet['right']
        # Überprüfung, ob sich die X-, Y- und Z-Koordinaten des Autos zwischen den Kanten der Straße befinden
        if left_edge[0][0] <= car_position[0] <= right_edge[0][0] and \
           left_edge[0][1] <= car_position[1] <= left_edge[1][1]:
            return True
    return False
def create_vehicle_pos_tuples(name, position):
    vehiclepos = (name, position)  # Erstelle ein Tupel aus Autoname und Position
    startPositions.append(vehiclepos)

def extract_RoutingAction(events):
    # handle Routing Action
    waypointCounter = 1
    waypointNames = []

    for event in events:
        if event.action.action_type == "RoutingAction":
            print("bin RoutingAction")
            for waypoint in event.action.routing_action.waypoints:
                print("bin RoutingAction Waypoint")
                print(waypoint)

                waypointString = "[" + waypoint['x'] + ", " + waypoint['y'] + ", " + waypoint['z'] + "]"
                print(waypointString)
                waypointName = "waypoint" + str(waypointCounter)
                waypointNames.append(waypointName)
                waypointLine = fillWaypointInXML(waypointName, waypointString)
                waypointCounter = waypointCounter + 1
                waypointLines.append(waypointLine)

    return waypointNames
def load_or_create_polygons():
    road_polygons = load_polygons()
    # Wenn keine gespeicherten Polygone vorhanden sind, erstelle sie neu
    if road_polygons is None:
        roads = bng.get_roads()
        road_polygons = []
        for road in roads:
            try:
                road_edges = bng.get_road_edges(road)
                road_polygon = create_road_polygon(road_edges)
                road_polygons.append(road_polygon)
            except Exception as e:
                print(f"Error processing road {road}: {e}")

        # Speichere die erstellten Polygone für den nächsten Start
        save_polygons(road_polygons)

    return road_polygons


def prepare_conditions(conditions: List[Dict], vehicles: List[Vehicle]):
    """
    Prepare start and end conditions from the given conditions and vehicles.

    Args:
        conditions (List[Dict]): List of conditions to check.
        vehicles (List[Vehicle]): List of vehicles involved in the conditions.

    Returns:
        Tuple containing:
            - start_condition_finished (bool): Whether the start condition is finished or not.
            - start_condition (Optional[Dict]): The start condition, if any.
            - end_condition (Optional[Dict]): The end condition, if any.
            - td_start_pos (Optional[Dict]): The starting position of the vehicle for distance traveled condition, if applicable.
            - os_cond_start_pos (Optional[Dict]): The starting position of the vehicle for overall start condition, if applicable.
    """
    start_condition_finished = True
    #start_condition = None
    end_condition = None
    td_start_pos = None
    os_cond_start_pos = None
    startTraveledDistanceCondition = None
    startReachCondition = None
    endReachCondition = None
    for condition in conditions:
        if condition.main_condition_type == 'EndCondition' and condition.condition_type == 'TraveledDistance':
            endTraveledDistanceCondition = True
            end_condition = condition
            for vehicle in vehicles:
                if condition.mainEntityRef == vehicle.vid:
                    td_start_pos = vehicle.state['pos']
        elif condition.main_condition_type == 'OverallStartCondition' and condition.condition_type == 'TraveledDistance':
            startTraveledDistanceCondition = True
            start_condition = condition
            for vehicle in vehicles:
                if condition.mainEntityRef == vehicle.vid:
                    os_cond_start_pos = vehicle.state['pos']
        elif condition.main_condition_type == 'OverallStartCondition' and condition.condition_type == 'ReachPosition':
            print ("ReachPosition in condition")
            startReachCondition = True
            start_condition = condition
        elif condition.main_condition_type == 'EndCondition' and condition.condition_type == 'ReachPosition':
            endReachCondition = True
            end_condition = condition
    if 'start_condition' in locals():
        start_condition_finished = False
    else:
        start_condition_finished = True  # if there is no start_condition
    print("start Reach Condition")
    print(startReachCondition)
    return start_condition_finished, start_condition, end_condition, td_start_pos, os_cond_start_pos, startTraveledDistanceCondition, startReachCondition, endReachCondition
async def monitor_vehicles_periodically(vehicles, road_polygons):
    while True:
        for vehicle in vehicles:
            monitor_vehicle_position(vehicle, road_polygons)
        await asyncio.sleep(0.001)  # Warte eine Sekunde


def prepare_events(events, parameters):
    """
    Diese Funktion bereitet die Bedingungen für die Events vor und aktualisiert die Parameter.
    """

    rdconditionset = False
    startReachCondition = False
    sbesconditionref = None
    sbesconditionstate = None

    # Gehe durch die Events und prüfe auf Bedingungen
    for event in events:
        # RelativeDistance-Bedingung und OverallStartCondition
        if event.condition.condition_type == 'RelativeDistance' and event.condition.main_condition_type == 'OverallStartCondition':
            rdcondition = event.condition
            rdconditionset = True
            atsvalue = event.action.speed_action.target_speed_value

            # Überprüfe, ob der atsvalue ein Parameter ist
            if atsvalue[0] == '$':
                print('$')
                parameter_name = atsvalue[1:]  # Der Name des Parameters nach dem '$'
                if parameter_name in parameters:
                    atsvalue = parameters[parameter_name]['value']  # Ersetze den Parameterwert
                    print(f"Parameter {parameter_name} ersetzt durch {atsvalue}")

        # ReachPosition-Bedingung und OverallStartCondition
        #elif event.condition.condition_type == 'ReachPosition' and event.condition.main_condition_type == 'OverallStartCondition':
            #print("ReachPosition in event")
           # startReachCondition = True

        # StoryboardElementState-Bedingung
        if event.condition.condition_type == 'StoryboardElementState':
            sbesconditionref = event.condition.storyboardElementRef
            sbesconditionstate = event.condition.state
            print(f"StoryboardElementState gefunden: Ref={sbesconditionref}, State={sbesconditionstate}")

    # Weitere Logik kann hier folgen, z.B. die Initialisierung von Bedingungen oder die Anpassung der Szenarien
    return rdconditionset, sbesconditionref, sbesconditionstate


def check_start_conditions(vehicles, start_condition, road_polygons, bng):
    """Überprüft die Startbedingungen basierend auf 'TraveledDistance' oder 'ReachCondition'."""
    start_condition_finished = False

    while not start_condition_finished:
        if start_condition.condition_type == 'TraveledDistance':
            start_condition_finished = handle_start_traveled_distance(vehicles, start_condition, road_polygons, bng)
        elif start_condition.condition_type == 'ReachCondition':
            start_condition_finished = handle_start_reach_condition(vehicles, start_condition, bng)
    return start_condition_finished


def handle_start_traveled_distance(vehicles, start_condition, road_polygons, bng):
    """Überprüft die 'TraveledDistance'-Startbedingung für alle Fahrzeuge."""
    for vehicle in vehicles:
        vehicle.sensors.poll()
        monitor_vehicle_position(vehicle, road_polygons)
        bng.step(3)  # Simulation fortsetzen

        traveled_distance = np.linalg.norm(np.array(vehicle.state['pos']) - start_condition.start_position)
        if start_condition.main_entity_ref == vehicle.vid and traveled_distance >= start_condition.value:
            print(f"StartCondition TraveledDistance successful: {traveled_distance}")
            return True
    return False


def handle_start_reach_condition(vehicles, start_condition, bng):
    """Überprüft die 'ReachCondition'-Startbedingung für alle Fahrzeuge."""
    for vehicle in vehicles:
        vehicle.sensors.poll()
        bng.step(3)  # Simulation fortsetzen

        if is_vehicle_at_target_position(vehicle.state['pos'], start_condition.position, start_condition.tolerance):
            print(f"StartCondition ReachCondition successful: {vehicle.state['pos']}")
            return True
    return False


def is_vehicle_at_target_position(vehicle_pos, target_pos, tolerance):
    """Überprüft, ob das Fahrzeug sich innerhalb der Toleranz um die Zielposition befindet."""
    return all(target_pos[i] - tolerance <= vehicle_pos[i] <= target_pos[i] + tolerance for i in range(3))


def check_end_conditions(vehicles, acts, road_polygons, parameters, start_positions, waypoint_names, bng):
    """Überprüft die Endbedingungen und führt Aktionen aus, wenn Bedingungen erfüllt sind."""
    end_condition_finished = False
    finished_events = []
    last_finished_event = None

    while not end_condition_finished:
        for vehicle in vehicles:
            vehicle.sensors.poll()
            monitor_vehicle_position(vehicle, road_polygons)
            bng.step(3)  # Simulation fortsetzen

            for act in acts:
                for event in act.events:
                    if event.condition.condition_type == 'RelativeDistance' and vehicle.vid in act.actors:
                        if handle_relative_distance_condition(vehicle, vehicles, event, parameters, finished_events,
                                                              last_finished_event, bng):
                            finished_events.append(event.name)
                            last_finished_event = event.name
                    elif event.condition.condition_type == 'StoryboardElementState' and vehicle.vid in act.actors:
                        if handle_storyboard_element_state(event, vehicle, finished_events, last_finished_event,
                                                           parameters, bng):
                            finished_events.append(event.name)
                            last_finished_event = event.name
                    elif event.condition.condition_type == 'TraveledDistance' and vehicle.vid in act.actors:
                        handle_traveled_distance_event(vehicle, start_positions, event, waypoint_names, finished_events,
                                                       bng)

            if 'end_condition' in locals():
                if check_end_condition(vehicle, end_condition, bng):
                    end_condition_finished = True
                    break
    return end_condition_finished


def handle_relative_distance_condition(vehicle, vehicles, event, parameters, finished_events, last_finished_event, bng):
    """Überprüft und behandelt die 'RelativeDistance'-Bedingung."""
    rd_condition = event.condition
    main_position = vehicle.state['pos']

    for vehicle2 in vehicles:
        if vehicle2.vid == rd_condition.entity_ref:
            distance_to_car = np.linalg.norm(np.array(vehicle2.state['pos']) - main_position)

            if check_relative_distance_rule(rd_condition, distance_to_car):
                set_vehicle_speed(vehicle2, event, parameters, bng)
                print(f"Relative Distance Condition successful for {event.name}")
                return True
    return False


def check_relative_distance_rule(rd_condition, distance):
    """Überprüft die Regel für die 'RelativeDistance'-Bedingung."""
    if rd_condition.rule == 'lessThan':
        return distance <= rd_condition.value
    elif rd_condition.rule == 'equalTo':
        return distance == rd_condition.value
    elif rd_condition.rule == 'greaterThan':
        return distance >= rd_condition.value
    return False


def set_vehicle_speed(vehicle, event, parameters, bng):
    """Setzt die Geschwindigkeit des Fahrzeugs basierend auf dem Event."""
    target_speed = event.action.speed_action.target_speed_value
    if target_speed.startswith('$'):
        target_speed = parameters.get(target_speed[1:], {}).get('value', target_speed)

    vehicle.ai.set_speed(float(target_speed))
    print(f"Speed set to {target_speed} for vehicle {vehicle.vid}")
    bng.step(3)


def handle_storyboard_element_state(event, vehicle, finished_events, last_finished_event, parameters, bng):
    """Überprüft und behandelt die 'StoryboardElementState'-Bedingung."""
    if event.condition.state in ['endTransition',
                                 'completeState'] and event.condition.storyboard_element_ref == last_finished_event:
        print(f"Processing StoryboardElementState for {event.name}")

        sleep(event.condition.delay)
        set_vehicle_speed(vehicle, event, parameters, bng)

        speed_success = wait_for_speed(vehicle, event, bng)
        if speed_success:
            print(f"StoryboardElementState of Event {event.name} finished")
            return True
    return False


def wait_for_speed(vehicle, event, bng):
    """Wartet, bis das Fahrzeug die gewünschte Geschwindigkeit erreicht hat."""
    target_speed = float(event.action.speed_action.target_speed_value)

    while True:
        vehicle.sensors.poll()
        actual_speed = np.linalg.norm(vehicle.state['vel'])

        if target_speed - 1.5 <= actual_speed <= target_speed + 0.5:
            return True
        else:
            print(f"Current speed: {actual_speed} m/s. Waiting for {vehicle.vid}")
            bng.step(1)  # Warte 1 Sekunde


def handle_traveled_distance_event(vehicle, start_positions, event, waypoint_names, finished_events, bng):
    """Behandelt die 'TraveledDistance'-Bedingung für das Event."""
    vehicle.sensors.poll()

    for name, start_position in start_positions:
        if name == vehicle.vid:
            traveled_distance = np.linalg.norm(np.array(vehicle.state['pos']) - start_position)
            if traveled_distance >= event.condition.value:
                for waypoint_name in waypoint_names:
                    vehicle.ai.set_waypoint(waypoint_name)
                    print(f"Waypoint {waypoint_name} set for vehicle {vehicle.vid}")
                finished_events.append(event.name)


def check_end_condition(vehicle, end_condition, bng):
    """Überprüft und behandelt die Endbedingung."""
    traveled_distance = np.linalg.norm(np.array(vehicle.state['pos']) - end_condition.start_position)

    if end_condition.condition_type == 'TraveledDistance' and traveled_distance >= end_condition.value:
        stop_vehicle(vehicle, bng)
        print("EndCondition TraveledDistance successful")
        return True
    elif end_condition.condition_type == 'ReachCondition' and is_vehicle_at_target_position(vehicle.state['pos'],
                                                                                            end_condition.position,
                                                                                            end_condition.tolerance):
        stop_vehicle(vehicle, bng)
        print("EndCondition ReachCondition successful")
        return True
    return False


def stop_vehicle(vehicle, bng):
    """Stoppt das Fahrzeug."""
    vehicle.control(throttle=0, brake=1)
    vehicle.ai.set_speed(0)
    bng.step(3)  # Simulation kurz fortsetzen, um das Fahrzeug anzuhalten
    sleep(10)

def main():
    conditions = parse_conditions_from_xml(XML_FILE_PATH)
    acts = parse_acts_from_xml(XML_FILE_PATH)
    xmlVehicles = extract_vehicle_objects(XML_FILE_PATH)
    events = parse_events_from_xml(XML_FILE_PATH)
    parameters = extract_Parameter(XML_FILE_PATH)
    waypointNames = extract_RoutingAction(events)
    weather = extract_weather_info(XML_FILE_PATH)
    fillWeatherInXML(weather)

    endConditionFinished = False

    bng = BeamNGpy('localhost', 64256, home=BEAMNG_HOME, user=BEAMNG_USER)
    # Launch BeamNG.tech
    bng.open()
    scenario = Scenario(BEAMNG_LEVEL, 'example')
    state = State()
    for vehicle in xmlVehicles:
        v = Vehicle(vehicle.get('name'), model=vehicle.get('vehicle').get('name'), license=vehicle.get('name'))
        vehicles.append(v)
        scenario.add_vehicle(v, pos=(
        vehicle.get('vehicle').get('X'), vehicle.get('vehicle').get('Y'), vehicle.get('vehicle').get('Z')), cling=True,
                             rot_quat=(0, 0, 0.3826834, 0.9238795))

    scenario.make(bng)
    bng.set_tod(str(weather.time_of_day))
    # Entfernung zwischen den Fahrzeugen berechnen
    rdconditionset = False
    pos1 = 0
    pos2 = 0
    bng.scenario.load(scenario)
    bng.scenario.start()
    bng.set_tod(weather.time_of_day)
    bng.set_weather_preset('xml_weather')
    bng.set_tod(weather.time_of_day)

    for vehicle in vehicles:
        vehicle.sensors.poll()
        create_vehicle_pos_tuples(vehicle.vid, vehicle.state['pos'])
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
    # Versuche, die Straßenpolygone aus der Datei zu laden
    bng.pause()

    road_polygons = load_or_create_polygons()

    bng.resume()
    bng.step(3)
    start_condition_finished, startCondition, endCondition, tdstartpos, oscondstartpos, startTraveledDistanceCondition, startReachCondition, endReachCondition = prepare_conditions(
        conditions, vehicles)
    rdconditionset, sbesconditionref, sbesconditionstate = prepare_events(events, parameters)

    print(f"Start Condition Finished: {start_condition_finished}, "
          f"Start Condition: {startCondition}, "
          f"End Condition: {endCondition}, "
          f"TD Start Position: {tdstartpos}, "
          f"OSCond Start Position: {oscondstartpos}"
          f"startTraveledDistanceCondition: {startTraveledDistanceCondition}"
          f"startReachCondition: {startReachCondition}"
          f"endReachCondition: {endReachCondition}")

    startConditionFinished = False
    asyncio.run(monitor_vehicles_periodically(vehicles, road_polygons))
    # Initialisierung von Variablen und Fahrzeugen
    vehicles = initialize_vehicles()  # Diese Funktion müsste die Fahrzeuge initialisieren
    acts = initialize_acts()  # Diese Funktion initialisiert die Szenen und Events
    road_polygons = load_road_polygons()  # Lädt die Straßendaten
    parameters = load_parameters()  # Lädt Parameterwerte aus einem externen System
    start_positions = get_start_positions(vehicles)  # Speichert Startpositionen der Fahrzeuge
    waypoint_names = get_waypoint_names()  # Definiert die Wegpunkte für die Fahrzeuge

    # Simulationsumgebung initialisieren
    bng = init_simulation_environment()  # Initialisiert die Simulation (BeamNG)

    # Startbedingungen überprüfen
    start_condition = get_start_condition()  # Holt die Startbedingungen aus der Konfiguration
    start_condition_finished = check_start_conditions(vehicles, start_condition, road_polygons, bng)

    if start_condition_finished:
        print("Startbedingungen erfolgreich abgeschlossen.")

    # Endbedingungen überwachen und Szenario abspielen
    end_condition = get_end_condition()  # Holt die Endbedingungen aus der Konfiguration
    end_condition_finished = check_end_conditions(vehicles, acts, road_polygons, parameters, start_positions,
                                                  waypoint_names, bng)



    # Simulation beenden und aufräumen
    bng.close()
    print("Simulation erfolgreich beendet.")


if __name__ == "__main__":
    main()


