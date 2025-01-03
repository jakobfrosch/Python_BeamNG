import logging
import math
import os
import pickle
import shutil
import sys
import re
from time import sleep
from typing import List, Dict
import asyncio
import threading
import time
import beamngpy
import numpy as np
from shapely.geometry import Polygon, Point
from shapely.ops import unary_union

from beamngpy import BeamNGpy, Scenario, Vehicle
from beamngpy.sensors import Damage
from xml.etree import ElementTree as ET

from src.Condition import parse_conditions_from_xml
from src.Event import parse_events_from_xml
from src.Vehicle import extract_vehicle_objects
from src.Weather import extract_weather_info
from src.Act import parse_acts_from_xml
from src.updateZIP import (
    fillWeatherInXML
)

print(beamngpy.__file__)
# Constants
POLYGON_SAVE_PATH = "road_polygons_old.pkl"

XML_FILE_PATH = "C:\\Users\\stefan\\Documents\\OpenSCENARIO_output\\Adapted\\FollowLeadingVehicle4_4_Random.xosc"
BEAMNG_HOME = 'C:\\Users\\stefan\\Music\\BeamNG.tech.v0.31.3.0\\BeamNG.tech.v0.31.3.0'
BEAMNG_USER = 'C:\\Users\\stefan\\AppData\\Local\\BeamNG.drive'
BEAMNG_LEVEL = 'west_coast_usa'
VEHICLE_LENGTH = 4.69  # in Meter
VEHICLE_WIDTH = 1.84  # in Meter
vehicles = []
out_of_road_list = []
logging.basicConfig(filename='C:\\Users\\stefan\\git\\repository\\Szenarienerstellungstool\\gemeinsames_log.log',
                    level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

logging.info("Python-Programm gestartet")

# Funktion zum Erstellen eines Straßenpolygons aus den Straßenkanten
def create_road_polygon(road_edges):
    left_points = [edge['left'][:2] for edge in road_edges]
    right_points = [edge['right'][:2] for edge in reversed(road_edges)]
    road_polygon = Polygon(left_points + right_points)

    if not road_polygon.is_valid:
        logging.warning("Das Straßenpolygon ist ungültig.")
    return road_polygon


# Funktion zur Überprüfung, ob das Fahrzeug auf einer der Straßen ist
def is_vehicle_on_any_road(vehicle_position, road_polygons):
    vehicle_point = Point(vehicle_position)
    for road_polygon in road_polygons:
        if road_polygon.contains(vehicle_point):
            return True
    return False

# Funktion für den Thread, die die Coroutine ausführt
def run_monitor_in_thread(vehicles, colission_stop_cond, keepLane_stop_cond, road_polygons, XML_FILE_PATH2, folder_path):
    # asyncio.run() führt die Coroutine in einer Event-Loop aus
    asyncio.run(monitor_vehicles_periodically(vehicles, colission_stop_cond, keepLane_stop_cond, road_polygons, XML_FILE_PATH2, folder_path))

# Funktion zur Überprüfung, ob das Fahrzeug auf der Straße ist
def is_vehicle_on_road(vehicle_box, combined_polygon):
    return combined_polygon.contains(vehicle_box)
# Funktion zur Überwachung der Fahrzeugposition
def monitor_vehicle_position(vehicle, colission_stop_cond, keepLane_stop_cond, combined_polygon, XML_FILE_PATH2, folder_path):
    with sensor_lock:
        vehicle.sensors.poll()  # Abfrage der Sensordaten
    curr_position = vehicle.state['pos'][:2]  # Fahrzeugposition abrufen (x, y)
    vehicle_direction = vehicle.state['dir'][1]  # Fahrzeugausrichtung in Grad

    # Erstelle das Fahrzeugrechteck
    vehicle_box = create_vehicle_box(curr_position, vehicle_direction)

    if not is_vehicle_on_road(vehicle_box, combined_polygon):
        log_off_road(curr_position)
        create_vehicle_pos_tuples(vehicle.vid, vehicle.state['pos'], False)
        if keepLane_stop_cond:
            logging.info(f"Simulation wird beendet, weil ein Fahrzeug die Straße verließ.")
            os.makedirs(border_folder, exist_ok=True)
            line_folder = "" + folder_path + "//Rand"
            file_name = os.path.basename(XML_FILE_PATH2)
            target_path = os.path.join(line_folder, file_name)
            shutil.copy(XML_FILE_PATH2, target_path)
            terminate_software(bng, XML_FILE_PATH2)
    if colission_stop_cond:
        if damage_check(vehicle):
            logging.info(f"Simulation wird beendet, weil ein Fahrzeug Schaden genommen hat.")
            os.makedirs(border_folder, exist_ok=True)
            damage_folder = "" + folder_path + "//Schaden"
            file_name = os.path.basename(XML_FILE_PATH2)
            target_path = os.path.join(damage_folder, file_name)
            shutil.copy(XML_FILE_PATH2, target_path)
            terminate_software(bng, XML_FILE_PATH2)
def damage_check(vehicle):
    if vehicle.sensors['damage'].data['damage'] > 0:
        logging.warning(f"Fahrzeug {vehicle.vid} hat Schaden")
        logging.info(vehicle.sensors['damage'].data['damage'])
        return True
    else:
        return False
# Funktion zum Loggen, wenn das Fahrzeug die Straße verlässt
def log_off_road(position):
    logging.warning(f"Ein Fahrzeug verließ die Straße an Position: {position}")
# Funktion zum Speichern der Polygone
def save_polygons(road_polygons):
    with open(POLYGON_SAVE_PATH, 'wb') as f:
        pickle.dump(road_polygons, f)
    logging.info(f"Road polygons gespeichert in {POLYGON_SAVE_PATH}")

# Funktion zum Laden der Polygone
def load_polygons():
    if os.path.exists(POLYGON_SAVE_PATH):
        with open(POLYGON_SAVE_PATH, 'rb') as f:
            road_polygons = pickle.load(f)
        logging.info(f"Road polygons geladen aus {POLYGON_SAVE_PATH}")
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
                logging.error(f"Error Verarbeitung für Road {road}: {e}")
        # Speichere die erstellten Polygone für den nächsten Start
        save_polygons(road_polygons)
    combined_polygon = unary_union([polygon for polygon in road_polygons if polygon.is_valid])

    return combined_polygon
def create_vehicle_box(vehicle_position, vehicle_direction):
    x, y = vehicle_position
    direction = math.radians(vehicle_direction)
    dx = math.cos(direction) * VEHICLE_LENGTH / 2
    dy = math.sin(direction) * VEHICLE_LENGTH / 2

    corners = [
        (x - dx - dy, y - dy + dx),
        (x + dx - dy, y + dy + dx),
        (x + dx + dy, y + dy - dx),
        (x - dx + dy, y - dy - dx)
    ]

    return Polygon(corners)

def prepare_conditions(conditions: List[Dict]):
    end_condition = None
    td_start_pos = None
    os_cond_start_pos = None
    for condition in conditions:
        if condition.main_condition_type == 'EndCondition' and condition.condition_type == 'TraveledDistance':
            endTraveledDistanceCondition = True
            end_condition = condition
        elif (condition.main_condition_type == 'OverallStartCondition' and
              condition.condition_type == 'TraveledDistance'):
            start_condition = condition
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
        start_condition = None
    return start_condition_finished, start_condition, end_condition
async def monitor_vehicles_periodically(vehicles, colission_stop_cond, keepLane_stop_cond, combined_polygon, XML_FILE_PATH2, folder_path):
    #combined_polygon = unary_union([polygon for polygon in road_polygons if polygon.is_valid])
    while process_stopped is False:
        for vehicle in vehicles:
            monitor_vehicle_position(vehicle, colission_stop_cond, keepLane_stop_cond, combined_polygon, XML_FILE_PATH2, folder_path)
            await asyncio.sleep(1)  # Warte eine Sekunde

def check_conditions(vehicles, condition, positions):
    """Überprüft die Startbedingungen basierend auf 'TraveledDistance' oder 'ReachCondition'."""
    condition_finished = False
    while not condition_finished:
        if condition.condition_type == 'TraveledDistance':
            condition_finished = handle_traveled_distance(condition, positions)
        elif condition.condition_type == 'ReachPosition':
            condition_finished = handle_reach_condition(vehicles, condition)
    return condition_finished

def handle_traveled_distance(start_condition,start_positions):
    for vehicle in vehicles:
        for name, start_position in start_positions:
            if name == vehicle.vid:
                with sensor_lock:
                    vehicle.sensors.poll()
                traveled_distance = np.linalg.norm(np.array(vehicle.state['pos']) - start_position)
                if start_condition.mainEntityRef == vehicle.vid and traveled_distance >= start_condition.value:
                    logging.info(f"TraveledDistance erfolgreich: Gefahrene Distanz: {traveled_distance} von "
                          f"{start_condition.value}")
                    return True
    return False

def handle_reach_condition(vehicles, start_condition):
    for vehicle in vehicles:
        with sensor_lock:
            vehicle.sensors.poll()
        if is_vehicle_at_target_position(vehicle.state['pos'], start_condition.position, start_condition.tolerance):
            logging.info(f"Condition ReachCondition successful: {vehicle.state['pos']}")
            return True
    return False


def is_vehicle_at_target_position(vehicle_pos, target_pos, tolerance):
    return all(target_pos[i] - tolerance <= vehicle_pos[i] <= target_pos[i] + tolerance for i in range(3))


def check_events(vehicles, acts, start_positions, waypoint_names, bng):
    last_condition_finished = False
    finished_events = []
    last_finished_event = None
    event_length = 0
    for act in acts:
        event_length += len(act.events)
    while not last_condition_finished:
        for vehicle in vehicles:
            with sensor_lock:
                vehicle.sensors.poll()
            for act in acts:
                for event in act.events:
                    if (event.action.action_type == 'SpeedAction' and vehicle.vid in act.actors and
                            event.name not in finished_events and
                            check_event_conditions(event.condition, start_positions, last_finished_event, vehicle,
                                                   bng) is True):
                        if speed_event(event, vehicle) is True:
                            finished_events.append(event.name)
                            last_finished_event = event.name
                            logging.info(f"{len(finished_events)} von {event_length} Event-Actions beendet. "
                                         f"Letztes beendetes Event: {last_finished_event}")
                    elif (event.action.action_type == "RoutingAction" and vehicle.vid in act.actors and
                          event.name not in finished_events and check_event_conditions(event.condition, start_positions,
                                                                     last_finished_event, vehicle, bng)
                          is True):
                        handle_routing_event(vehicle, waypoint_names)
                        finished_events.append(event.name)
                        last_finished_event = event.name
                        logging.info(f"{len(finished_events)} von {event_length} Event-Actions beendet. "
                                     f"Letztes beendetes Event: {last_finished_event}")
                if len(finished_events) == event_length:
                    last_condition_finished = True
    return last_condition_finished
def delay(delay):
    if delay is not None:
        sleep(delay)

def check_event_conditions(condition, start_positions, last_finished_event, vehicle_main, bng):
    if condition.condition_type == "StoryboardElementState":
        if (condition.state in ['endTransition', 'completeState'] and
                condition.storyboardElementRef == last_finished_event):
            delay(condition.delay)
            return True
    elif condition.condition_type == 'RelativeDistance':
        with sensor_lock:
            vehicle_main.sensors.poll()
        main_position = vehicle_main.state['pos']
        for vehicle in vehicles:
            if vehicle.vid == condition.entity_ref:
                with sensor_lock:
                    vehicle.sensors.poll()
                distance_to_car = np.linalg.norm(np.array(vehicle.state['pos']) - main_position)
                if check_relative_distance_rule(condition, distance_to_car):
                    return True
    elif condition.condition_type == 'TraveledDistance':
        return handle_traveled_distance(condition, start_positions)
    return False

def check_relative_distance_rule(rd_condition, distance):
    if rd_condition.rule == 'lessThan':
        return distance <= rd_condition.value
    elif rd_condition.rule == 'equalTo':
        return distance == rd_condition.value
    elif rd_condition.rule == 'greaterThan':
        return distance >= rd_condition.value
    return False


def set_vehicle_speed(vehicle1, event):
    target_speed = event.action.speed_action.target_speed_value
    float_target_speed = float(target_speed)
    with sensor_lock:
        vehicle1.ai.set_speed(float_target_speed)
    logging.info(f"Setzte Zielgeschwindikeit {target_speed} für Fahrzeug {vehicle1.vid}")
def speed_event(event, vehicle):
    set_vehicle_speed(vehicle, event)
    speed_success = wait_for_speed(vehicle, event)
    if speed_success:
        logging.info(f"Speedevent {event.name} beendet")
        return True
    return False


def wait_for_speed(vehicle, event):
    target_speed = float(event.action.speed_action.target_speed_value)
    while True:
        with sensor_lock:
            vehicle.sensors.poll()  # Abfrage der Sensordaten
        actual_speed = np.linalg.norm(vehicle.state['vel'])

        if target_speed - 1.5 <= actual_speed <= target_speed + 0.5:
            logging.info(f"Zielgeschwindigkeit {target_speed} erreicht")
            return True
def handle_routing_event(vehicle, waypoint_names):
    with sensor_lock:
        vehicle.sensors.poll()  # Abfrage der Sensordaten
    for waypoint_name in waypoint_names:
        with sensor_lock:
            vehicle.ai.set_waypoint(waypoint_name)
        logging.info(f"Waypoint {waypoint_name} für Fahrzeug {vehicle.vid} gesetzt")



def stop_vehicle(vehicle):
    with sensor_lock:
        vehicle.ai.set_speed(0)
def run_async_tasks(loop, vehicles, colission_stop_cond, keepLane_stop_cond, road_polygons, XML_FILE_PATH2, folder_path):
    asyncio.set_event_loop(loop)
    loop.run_until_complete(monitor_vehicles_periodically(vehicles, colission_stop_cond, keepLane_stop_cond, road_polygons, XML_FILE_PATH2, folder_path))
    loop.close()
def terminate_software(bng, XML_FILE_PATH2):
    logging.info(f"Die Software wird bei {XML_FILE_PATH2} abgebrochen werden bei.")
    #thread.join()
    bng.close()
    logging.info("BNG closed in terminate_software")
    directory_path = os.path.dirname(XML_FILE_PATH2)


    # 2) Extrahiere den Dateinamen ohne Endung und alles nach "_repl_para"
    filename = os.path.basename(XML_FILE_PATH2)  # Nur den Dateinamen
    name_without_extension = os.path.splitext(filename)[0]  # Entferne die Endung
    name_before_repl_para = re.split(r"_repl_para", name_without_extension)[0]  # Trenne vor "_repl_para"

    delete_files_starting_with(directory_path, name_before_repl_para)
    sys.exit(1)  # Beendet das Programm


async def check_time(target_time, bng, XML_FILE_PATH2, folder_path):
    while True:
        current_time = time.time()
        if current_time >= target_time:
            logging.info("Maximale Zeit erreicht.")
            os.makedirs(border_folder, exist_ok=True)
            time_folder=""+folder_path+"//Zeit"
            file_name = os.path.basename(XML_FILE_PATH2)
            target_path = os.path.join(time_folder, file_name)
            shutil.copy(XML_FILE_PATH2, target_path)
            for vehicle in vehicles:
                with sensor_lock:
                    vehicle.sensors.poll()
                if vehicle.sensors['damage'].data['damage'] > 0:
                    logging.warning(f"Fahrzeug {vehicle.vid} hat Schaden")
                    logging.info(vehicle.sensors['damage'].data['damage'])
                    os.makedirs(damage_folder, exist_ok=True)
                    file_name = os.path.basename(XML_FILE_PATH2)
                    target_path = os.path.join(damage_folder, file_name)
                    shutil.copy(XML_FILE_PATH2, target_path)
                else:
                    logging.info(f"Fahrzeug {vehicle.vid} hat keinen Schaden")
            if not out_of_road_list:
                logging.info("Keine Fahrzeuge haben die Straße verlassen")
            else:
                logging.warning("Folgende Fahrzeuge haben die Straße verlassen:")
                os.makedirs(border_folder, exist_ok=True)
                file_name = os.path.basename(XML_FILE_PATH2)
                target_path = os.path.join(border_folder, file_name)
                shutil.copy(XML_FILE_PATH2, target_path)
                for name, out_position in out_of_road_list:
                    logging.warning(f"Fahrzeug {name} an Position {out_position}")

            terminate_software(bng, XML_FILE_PATH2)
            break
        await asyncio.sleep(1)
def run_async_task_time(loop, t, bng, XML_FILE_PATH2, folder_path):
    asyncio.set_event_loop(loop)
    target_time = time.time() + t
    loop.run_until_complete(check_time(target_time, bng, XML_FILE_PATH2, folder_path))

def delete_files_starting_with(directory_path, prefix):
    directory = Path(directory_path)

    # Überprüfen, ob der angegebene Pfad ein Verzeichnis ist
    if not directory.is_dir():
        logging.error(f"{directory_path} ist kein gültiges Verzeichnis.")
        return

    # Durchlaufe alle Dateien im Verzeichnis
    for file in directory.iterdir():
        # Prüfen, ob die Datei mit dem Präfix beginnt und ein File ist
        if file.is_file() and file.name.startswith(prefix):
            try:
                file.unlink()  # Datei löschen
                logging.info(f"Datei gelöscht: {file}")
            except Exception as e:
                logging.error(f"Konnte Datei nicht löschen: {file}. Fehler: {e}")


from pathlib import Path
import logging


def delete_files_starting_with(directory_path, prefix):
    directory = Path(directory_path)
    if not directory.is_dir():
        logging.error(f"{directory_path} ist kein gültiges Verzeichnis.")
        return

    for file in directory.iterdir():
        if file.is_file() and file.name.startswith(prefix):
            try:
                file.unlink()  # Datei löschen
                logging.info(f"Datei gelöscht: {file}")
            except Exception as e:
                logging.error(f"Konnte Datei nicht löschen: {file}. Fehler: {e}")

arguments = sys.argv[2:]
for argument in arguments:
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
    XML_FILE_PATH = "" + sys.argv[1] + argument
    logging.info(f"Führe {XML_FILE_PATH} aus.")
    damage_folder = "" + sys.argv[1] + "//Schaden"
    time_folder = "" + sys.argv[1] + "//Zeit"
    border_folder = "" + sys.argv[1] + "//Rand"
    success_folder = "" + sys.argv[1] + "//Erfolg"
    parameters = extract_Parameter(XML_FILE_PATH)
    XML_FILE_PATH2 = replace_Parameter(XML_FILE_PATH, parameters)
    conditions = parse_conditions_from_xml(XML_FILE_PATH2)
    acts = parse_acts_from_xml(XML_FILE_PATH2)
    xmlVehicles = extract_vehicle_objects(XML_FILE_PATH2)
    events = parse_events_from_xml(XML_FILE_PATH2)
    waypointNames = extract_RoutingAction(events)
    weather = extract_weather_info(XML_FILE_PATH2)
    fillWeatherInXML(weather, BEAMNG_HOME)
    colission_stop_cond = False
    keepLane_stop_cond = False
    time_stop_cond = False
    for condition in conditions:
        if condition.main_condition_type == 'criteria_CollisionTest' and condition.stop_condition:
            colission_stop_cond = True
        if condition.main_condition_type == 'criteria_KeepLaneTest' and condition.stop_condition:
            keepLane_stop_cond = True
        if condition.condition_type == 'SimulationTime' and condition.stop_condition:
            time_stop_cond = True

    bng = BeamNGpy('localhost', 64256, home=BEAMNG_HOME, user=BEAMNG_USER)
    # Launch BeamNG.tech
    bng.open()
    scenario = Scenario(BEAMNG_LEVEL, 'example')
    for vehicle in xmlVehicles:
        v = Vehicle(vehicle.get('name'), model=vehicle.get('vehicle').get('name'), license=vehicle.get('name'))
        vehicles.append(v)
        scenario.add_vehicle(v, pos=(
            vehicle.get('vehicle').get('X'), vehicle.get('vehicle').get('Y'), vehicle.get('vehicle').get('Z')),
                             cling=True,
                             rot_quat=(0, 0, 0.3826834, 0.9238795))
    start_condition_finished, start_condition, end_condition = prepare_conditions(conditions)
    scenario.make(bng)
    bng.scenario.load(scenario)
    bng.scenario.start()
    bng.set_tod(weather.time_of_day)
    bng.set_weather_preset('xml_weather')
    bng.set_tod(weather.time_of_day)
    for vehicle in vehicles:
        with sensor_lock:
            vehicle.sensors.poll()
        create_vehicle_pos_tuples(vehicle.vid, vehicle.state['pos'], True)
    bng.pause()
    road_polygons = load_or_create_polygons(bng)
    bng.resume()
    sleep(5)
    for vehicle in xmlVehicles:
        for vehicle2 in vehicles:
            if vehicle2.vid == vehicle.get('name'):
                vehicle2.ai.drive_in_lane(True)
                vehicle2.ai.set_mode('span')
                vehicle2.set_velocity(vehicle.get('vehicle').get('maxSpeed') / 3.6)
                vehicle2.set_color(vehicle.get('vehicle').get('color'))
                if vehicle.get('vehicle').get('aggression') is not None:
                    logging.info(f"Setze Aggression auf {vehicle.get('vehicle').get('aggression')} für Fahrzeug {vehicle2.vid}")
                    vehicle2.ai.set_aggression(vehicle.get('vehicle').get('aggression'))
                damage_sensor = Damage()
                vehicle2.attach_sensor('damage', damage_sensor)
    if time_stop_cond:
        t = float(condition.value)
        loop = asyncio.new_event_loop()
        thread = threading.Thread(target=run_async_task_time, args=(loop, t, bng, XML_FILE_PATH2, sys.argv[1]))
        thread.start()
    loop = asyncio.new_event_loop()
    thread = threading.Thread(target=run_async_tasks, args=(loop, vehicles, colission_stop_cond, keepLane_stop_cond, road_polygons, XML_FILE_PATH2, sys.argv[1]))
    thread.start()
    if start_condition_finished:
        logging.info("Startbedingungen nicht definiert. Springe gleich zu Events.")
    else:
        start_condition_finished = check_conditions(vehicles, start_condition, start_positions)
    if start_condition_finished:
        logging.info("Startbedingungen erfolgreich abgeschlossen.")
    acts_finished = check_events(vehicles, acts, start_positions, waypointNames, bng)
    if acts_finished:
        logging.info("Act-Bedingungen erfolgreich abgeschlossen.")

    end_condition_finished = check_conditions(vehicles, end_condition, start_positions)
    if end_condition_finished:
        logging.info("Endbedingung erfolgreich abgeschlossen. Stop Vehicles: ")
        os.makedirs(success_folder, exist_ok=True)
        file_name = os.path.basename(XML_FILE_PATH2)
        target_path = os.path.join(success_folder, file_name)
        shutil.copy(XML_FILE_PATH2, target_path)
    for vehicle in vehicles:
        stop_vehicle(vehicle)
    logging.info("Fahrzeuge gestoppt!")
    process_stopped = True
    for vehicle in vehicles:
        if vehicle.sensors['damage'].data['damage'] > 0:
            logging.warning(f"Fahrzeug {vehicle.vid} hat Schaden")
            logging.info(vehicle.sensors['damage'].data['damage'])
            os.makedirs(damage_folder, exist_ok=True)
            file_name = os.path.basename(XML_FILE_PATH2)
            target_path = os.path.join(damage_folder, file_name)
            shutil.copy(XML_FILE_PATH2, target_path)
        else:
            logging.info(f"Fahrzeug {vehicle.vid} hat keinen Schaden")
    if not out_of_road_list:
        logging.info("Keine Fahrzeuge haben die Straße verlassen")
    else:
        logging.warning("Folgende Fahrzeuge haben die Straße verlassen:")
        os.makedirs(border_folder, exist_ok=True)
        file_name = os.path.basename(XML_FILE_PATH2)
        target_path = os.path.join(border_folder, file_name)
        shutil.copy(XML_FILE_PATH2, target_path)
        for name, out_position in out_of_road_list:
            logging.warning(f"Fahrzeug {name} an Position {out_position}")
    thread.join()
    bng.close()
    logging.info(f"Ende der Ausführung von {XML_FILE_PATH}")
    delete_files_starting_with(sys.argv[1], os.path.splitext(argument)[0])
sys.exit(0)