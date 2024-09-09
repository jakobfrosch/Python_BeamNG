import logging
import math
from time import sleep
from typing import List, Dict
import numpy as np
from beamngpy.sensors import Damage
import xml.etree.ElementTree as ET
import asyncio
import threading
from Condition import parse_conditions_from_xml
from Vehicle import extract_vehicle_objects
from Event import parse_events_from_xml
from Act import parse_acts_from_xml
from Weather import extract_weather_info
from updateZIP import fillWeatherInXML, fillWaypointInXML, remove_line_from_file_in_zip, \
    removeWaypointInXML
import pickle
import os
from shapely.ops import unary_union
from shapely.geometry import Polygon, Point
from beamngpy import BeamNGpy, Scenario, Vehicle

logging.basicConfig(level=logging.INFO)

# Constants
POLYGON_SAVE_PATH = "road_polygons.pkl"

XML_FILE_PATH = "C:\\Users\\stefan\\Downloads\\FollowLeadingVehicle4.xosc"
BEAMNG_HOME = 'C:\\Users\\stefan\\Music\\BeamNG.tech.v0.31.3.0\\BeamNG.tech.v0.31.3.0'
BEAMNG_USER = 'C:\\Users\\stefan\\AppData\\Local\\BeamNG.drive'
BEAMNG_LEVEL = 'west_coast_usa'
VEHICLE_LENGTH = 4.62  # in Meter
VEHICLE_WIDTH = 1.82  # in Meter
out_of_road_list = []
waypointLines = []
off_the_road_vehicles = []
vehicles = []
start_positions = []
sensor_lock = threading.Lock()
startTraveledDistanceCondition = False
endTraveledDistanceCondition = False
endReachCondition = False
process_stopped = False
connection_lock = threading.Lock()

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

# Funktion für den Thread, die die Coroutine ausführt
def run_monitor_in_thread(vehicles, road_polygons):
    # asyncio.run() führt die Coroutine in einer Event-Loop aus
    asyncio.run(monitor_vehicles_periodically(vehicles, road_polygons))

# Funktion zur Überprüfung, ob das Fahrzeug auf der Straße ist
def is_vehicle_on_road(vehicle_box, combined_polygon):
    return combined_polygon.contains(vehicle_box)
# Funktion zur Überwachung der Fahrzeugposition
def monitor_vehicle_position(vehicle, combined_polygon):
    with sensor_lock:
        vehicle.sensors.poll()  # Abfrage der Sensordaten
    curr_position = vehicle.state['pos'][:2]  # Fahrzeugposition abrufen (x, y)
    vehicle_direction = vehicle.state['dir'][1]  # Fahrzeugausrichtung in Grad

    # Erstelle das Fahrzeugrechteck
    vehicle_box = create_vehicle_box(curr_position, vehicle_direction)

    if not is_vehicle_on_road(vehicle_box, combined_polygon):
        log_off_road(curr_position)
        create_vehicle_pos_tuples(vehicle.vid, vehicle.state['pos'], False)

# Funktion zum Loggen, wenn das Fahrzeug die Straße verlässt
def log_off_road(position):
    print(f"Vehicle went off-road at position: {position}")



# Funktion zum Speichern der Polygone
def save_polygons(road_polygons):
    with open(POLYGON_SAVE_PATH, 'wb') as f:
        pickle.dump(road_polygons, f)
    print(f"Road polygons gespeichert in {POLYGON_SAVE_PATH}")


# Funktion zum Laden der Polygone
def load_polygons():
    if os.path.exists(POLYGON_SAVE_PATH):
        with open(POLYGON_SAVE_PATH, 'rb') as f:
            road_polygons = pickle.load(f)
        print(f"Road polygons geladen aus {POLYGON_SAVE_PATH}")
        return road_polygons
    return None

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
    # Lese den Inhalt der Eingabedatei
    with open(input_file_path, 'r') as file:
        content = file.read()

    # Ersetze die Parameter im Text
    for name, param in parameter_declarations.items():
        content = content.replace('$' + name, str(param['value']))

    # Schreibe den aktualisierten Text in die Ausgabedatei
    with open(output_file_path, 'w') as file:
        file.write(content)

    return output_file_path
def create_vehicle_pos_tuples(name, position, start_pos):
    vehicle_pos = (name, position)  # Erstelle ein Tupel aus Autoname und Position
    if start_pos is True:
        start_positions.append(vehicle_pos)
    else:
        out_of_road_list.append(vehicle_pos)

def extract_RoutingAction(events):
    # handle Routing Action
    waypointCounter = 1
    waypointNames = []

    for event in events:
        if event.action.action_type == "RoutingAction":
            for waypoint in event.action.routing_action.waypoints:
                waypointString = "[" + waypoint['x'] + ", " + waypoint['y'] + ", " + waypoint['z'] + "]"
                waypointName = "waypoint" + str(waypointCounter)
                waypointNames.append(waypointName)
                #activate for new waypoints
                #waypointLine = fillWaypointInXML(waypointName, waypointString, BEAMNG_HOME)
                #waypointCounter = waypointCounter + 1
                #waypointLines.append(waypointLine)

    return waypointNames
def load_or_create_polygons(bng):
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
                print(f"Error Verarbeitung für Road {road}: {e}")

        # Speichere die erstellten Polygone für den nächsten Start
        save_polygons(road_polygons)
    combined_polygon = unary_union([polygon for polygon in road_polygons if polygon.is_valid])

    return combined_polygon
def create_vehicle_box(vehicle_position, vehicle_direction):
    x, y = vehicle_position
    direction = math.radians(vehicle_direction)

    # Berechnung der Fahrzeug-Eckpunkte basierend auf Position und Ausrichtung
    dx = math.cos(direction) * VEHICLE_LENGTH / 2
    dy = math.sin(direction) * VEHICLE_LENGTH / 2

    corners = [
        (x - dx - dy, y - dy + dx),  # Ecke 1
        (x + dx - dy, y + dy + dx),  # Ecke 2
        (x + dx + dy, y + dy - dx),  # Ecke 3
        (x - dx + dy, y - dy - dx)  # Ecke 4
    ]

    return Polygon(corners)

def prepare_conditions(conditions: List[Dict], vehicles: List[Vehicle]):
    end_condition = None
    td_start_pos = None
    os_cond_start_pos = None

    for condition in conditions:
        if condition.main_condition_type == 'EndCondition' and condition.condition_type == 'TraveledDistance':
            endTraveledDistanceCondition = True
            end_condition = condition
            for vehicle in vehicles:
                if condition.mainEntityRef == vehicle.vid:
                    td_start_pos = vehicle.state['pos']
        elif condition.main_condition_type == 'OverallStartCondition' and condition.condition_type == 'TraveledDistance':
            start_condition = condition
            for vehicle in vehicles:
                if condition.mainEntityRef == vehicle.vid:
                    os_cond_start_pos = vehicle.state['pos']
        elif condition.main_condition_type == 'OverallStartCondition' and condition.condition_type == 'ReachPosition':
            startReachCondition = True
            start_condition = condition
        elif condition.main_condition_type == 'EndCondition' and condition.condition_type == 'ReachPosition':
            endReachCondition = True
            end_condition = condition
    if 'start_condition' in locals():
        start_condition_finished = False
    else:
        start_condition_finished = True  # if there is no start_condition
    return start_condition_finished, start_condition, end_condition, td_start_pos, os_cond_start_pos, startReachCondition
async def monitor_vehicles_periodically(vehicles, combined_polygon):
    #combined_polygon = unary_union([polygon for polygon in road_polygons if polygon.is_valid])
    while process_stopped is False:
        for vehicle in vehicles:
            monitor_vehicle_position(vehicle, combined_polygon)
            await asyncio.sleep(1)  # Warte eine Sekunde

def check_conditions(vehicles, condition, positions, bng):
    """Überprüft die Startbedingungen basierend auf 'TraveledDistance' oder 'ReachCondition'."""
    condition_finished = False
    while not condition_finished:
        if condition.condition_type == 'TraveledDistance':
            condition_finished = handle_traveled_distance(condition, positions, bng)
        elif condition.condition_type == 'ReachPosition':
            condition_finished = handle_reach_condition(vehicles, condition, bng)
    return condition_finished

def handle_traveled_distance(start_condition,start_positions, bng):
    """Überprüft die 'TraveledDistance'-Startbedingung für alle Fahrzeuge."""
    for vehicle in vehicles:
        for name, start_position in start_positions:
            if name == vehicle.vid:
                with sensor_lock:
                    vehicle.sensors.poll()  # Abfrage der Sensordaten
                bng.step(3)  # Simulation fortsetzen

                traveled_distance = np.linalg.norm(np.array(vehicle.state['pos']) - start_position)
                if start_condition.mainEntityRef == vehicle.vid and traveled_distance >= start_condition.value:
                    print(f"TraveledDistance erfolgreich: Gefahrene Distanz: {traveled_distance} von {start_condition.value}")
                    return True
    return False

def handle_reach_condition(vehicles, start_condition, bng):
    """Überprüft die 'ReachCondition'-Startbedingung für alle Fahrzeuge."""
    for vehicle in vehicles:
        with sensor_lock:
            vehicle.sensors.poll()  # Abfrage der Sensordaten
        if is_vehicle_at_target_position(vehicle.state['pos'], start_condition.position, start_condition.tolerance):
            print(f"Condition ReachCondition successful: {vehicle.state['pos']}")
            return True
    return False


def is_vehicle_at_target_position(vehicle_pos, target_pos, tolerance):
    """Überprüft, ob das Fahrzeug sich innerhalb der Toleranz um die Zielposition befindet."""
    return all(target_pos[i] - tolerance <= vehicle_pos[i] <= target_pos[i] + tolerance for i in range(3))


def check_act_conditions(vehicles, acts, parameters, start_positions, waypoint_names, bng):
    """Überprüft die Actbedingungen und führt Aktionen aus, wenn Bedingungen erfüllt sind."""
    last_condition_finished = False
    finished_events = []
    last_finished_event = None
    event_length = 0
    for act in acts:
        event_length += len(act.events)
    while not last_condition_finished:
        for vehicle in vehicles:
            with sensor_lock:
                vehicle.sensors.poll()  # Abfrage der Sensordaten
            bng.step(3)

            for act in acts:

                for event in act.events:
                    if event.action.action_type == 'SpeedAction' and vehicle.vid in act.actors and event.name not in finished_events and check_event_conditions(event.condition, start_positions, last_finished_event, vehicle, waypoint_names, bng) is True:
                        if speed_event(event, vehicle, finished_events, last_finished_event,parameters, start_positions, bng) is True:
                            finished_events.append(event.name)
                            last_finished_event = event.name
                    elif event.action.action_type == "RoutingAction" and vehicle.vid in act.actors and event.name not in finished_events and check_event_conditions(event.condition, start_positions, last_finished_event, vehicle, waypoint_names, bng) is True:
                        handle_routing_event(vehicle, start_positions, event, waypoint_names, finished_events, bng)
                        finished_events.append(event.name)
                        last_finished_event = event.name
                if len(finished_events) == event_length:
                    print(f"{len(finished_events)} von {event_length} Event-Actions beendet!")
                    last_condition_finished = True
                else:
                    print(f"{len(finished_events)} von {event_length} Event-Actions beendet!")
    return last_condition_finished
def delay(delay):
    if delay is not None:
        sleep(delay)

def check_event_conditions(condition, start_positions, last_finished_event, vehicle_main, waypoint_names, bng):
    if condition.condition_type == "StoryboardElementState":
        if condition.state in ['endTransition', 'completeState'] and condition.storyboardElementRef == last_finished_event:
            delay(condition.delay)
            return True
    elif condition.condition_type == 'RelativeDistance':
        with sensor_lock:
            vehicle_main.sensors.poll()  # Abfrage der Sensordaten
        main_position = vehicle_main.state['pos']
        for vehicle in vehicles:
            if vehicle.vid == condition.entity_ref:
                with sensor_lock:
                    vehicle.sensors.poll()  # Abfrage der Sensordaten
                distance_to_car = np.linalg.norm(np.array(vehicle.state['pos']) - main_position)
                if check_relative_distance_rule(condition, distance_to_car):
                    return True
    elif condition.condition_type == 'TraveledDistance':
        return handle_traveled_distance(condition, start_positions, bng)
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


def set_vehicle_speed(vehicle1, event, parameters, bng):
    """Setzt die Geschwindigkeit des Fahrzeugs basierend auf dem Event."""
    target_speed = event.action.speed_action.target_speed_value
    float_target_speed=float(target_speed)
    with sensor_lock:
        vehicle1.ai.set_speed(float_target_speed)
    #print("Lock released")
    #vehicle.ai.set_speed(float(target_speed))
    print(f"Setzte Zielgeschwindikeit {target_speed} für Fahrzeug {vehicle1.vid}")
    #bng.step(3)


def speed_event(event, vehicle, finished_events, last_finished_event, parameters, start_positions, bng):
    """Überprüft und behandelt die 'StoryboardElementState'-Bedingung."""
    set_vehicle_speed(vehicle, event, parameters, bng)
    speed_success = wait_for_speed(vehicle, event, bng)
    if speed_success:
        print(f"Speedevent {event.name} beendet")
        return True
    return False


def wait_for_speed(vehicle, event, bng):
    """Wartet, bis das Fahrzeug die gewünschte Geschwindigkeit erreicht hat."""
    target_speed = float(event.action.speed_action.target_speed_value)
    while True:
        with sensor_lock:
            vehicle.sensors.poll()  # Abfrage der Sensordaten
        actual_speed = np.linalg.norm(vehicle.state['vel'])

        if target_speed - 1.5 <= actual_speed <= target_speed + 0.5:
            print(f"Zielgeschwindigkeit {target_speed} erreicht")
            return True
        else:
            bng.step(1)  # Warte 1 Sekunde
def handle_routing_event(vehicle, start_positions, event, waypoint_names, finished_events, bng):
    """Behandelt die 'TraveledDistance'-Bedingung für das Event."""
    with sensor_lock:
        vehicle.sensors.poll()  # Abfrage der Sensordaten
    for waypoint_name in waypoint_names:
        vehicle.ai.set_waypoint(waypoint_name)
        print(f"Waypoint {waypoint_name} für Fahrzeug {vehicle.vid} gesetzt")



def stop_vehicle(vehicle, bng):
    """Stoppt das Fahrzeug."""
    vehicle.control(throttle=0, brake=1)
    vehicle.ai.set_speed(0)
    bng.step(3)
    #sleep(10)
def run_async_tasks(loop, vehicles, road_polygons):
    asyncio.set_event_loop(loop)

    # Starte die Überprüfung periodisch im Event-Loop
    loop.run_until_complete(monitor_vehicles_periodically(vehicles, road_polygons))

    # Schließe den Event-Loop
    loop.close()

parameters = extract_Parameter(XML_FILE_PATH)
XML_FILE_PATH2 = replace_Parameter(XML_FILE_PATH, parameters)
conditions = parse_conditions_from_xml(XML_FILE_PATH2)
acts = parse_acts_from_xml(XML_FILE_PATH2)
xmlVehicles = extract_vehicle_objects(XML_FILE_PATH2)
events = parse_events_from_xml(XML_FILE_PATH2)
waypointNames = extract_RoutingAction(events)
weather = extract_weather_info(XML_FILE_PATH2)
fillWeatherInXML(weather, BEAMNG_HOME)

bng = BeamNGpy('localhost', 64256, home=BEAMNG_HOME, user=BEAMNG_USER)
# Launch BeamNG.tech
bng.open()
scenario = Scenario(BEAMNG_LEVEL, 'example')
for vehicle in xmlVehicles:
    v = Vehicle(vehicle.get('name'), model=vehicle.get('vehicle').get('name'), license=vehicle.get('name'))
    vehicles.append(v)
    scenario.add_vehicle(v, pos=(
    vehicle.get('vehicle').get('X'), vehicle.get('vehicle').get('Y'), vehicle.get('vehicle').get('Z')), cling=True,
                         rot_quat=(0, 0, 0.3826834, 0.9238795))
scenario.make(bng)
bng.set_tod(str(weather.time_of_day))
bng.scenario.load(scenario)
bng.scenario.start()
bng.set_tod(weather.time_of_day)
bng.set_weather_preset('xml_weather')
bng.set_tod(weather.time_of_day)

for vehicle in vehicles:
    with sensor_lock:
        vehicle.sensors.poll()  # Abfrage der Sensordaten
    create_vehicle_pos_tuples(vehicle.vid, vehicle.state['pos'],True)
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
road_polygons = load_or_create_polygons(bng)
bng.resume()
bng.step(3)
start_condition_finished, startCondition, end_condition, tdstartpos, startPosition, startReachCondition = prepare_conditions(
    conditions, vehicles)

start_condition_finished = check_conditions(vehicles, startCondition, start_positions, bng)
if start_condition_finished:
    print("Startbedingungen erfolgreich abgeschlossen.")
loop = asyncio.new_event_loop()

# Starte einen neuen Thread, um die Aufgaben auszuführen

thread = threading.Thread(target=run_async_tasks, args=(loop, vehicles, road_polygons))
thread.start()
acts_finished = check_act_conditions(vehicles, acts, parameters, start_positions, waypointNames, bng)
if acts_finished:
    print("Act-Bedingungen erfolgreich abgeschlossen.")
end_condition_finished = check_conditions(vehicles, end_condition, start_positions, bng)
if end_condition_finished:
    print("Endbedingung erfolgreich abgeschlossen. Stop Vehicles: ")
for vehicle in vehicles:
    stop_vehicle(vehicle,bng)
print("Stopped Vehicles!")
for condition in conditions:
    if condition.main_condition_type == 'criteria_CollisionTest':
        for vehicle in vehicles:
            print(vehicle.vid + ': ')
            if vehicle.sensors['damage'].data['damage'] > 0:
                print("Fahrzeug hat Schaden")
                print(vehicle.sensors['damage'].data['damage'])
            else:
                print("Fahrzeug hat keinen Schaden")

if not out_of_road_list:
    print("Keine Fahrzeuge haben die Straße verlassen")
else:
    print("Folgende Fahrzeuge haben die Straße verlassen:")
    for name, out_position in out_of_road_list:
        print(f"Fahrzeug {name} an Position {out_position}")
process_stopped = True
thread.join()
bng.close()
