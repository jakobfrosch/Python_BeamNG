from time import sleep
from typing import List, Dict, Tuple
import numpy as np
from beamngpy.sensors import Damage
import sys
import xml.etree.ElementTree as ET

from src.Condition import parse_conditions_from_xml
from src.Vehicle import extract_vehicle_objects
from src.Event import parse_events_from_xml
from src.Act import parse_acts_from_xml
from src.Weather import extract_weather_info
from src.updateZIP import fillWeatherInXML, fillWaypointInXML, removeWaypointInXML

import math
import pickle
import os
from shapely.geometry import Polygon, Point
from beamngpy import BeamNGpy, Scenario, Vehicle
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPolygon
from beamngpy.sensors import State

POLYGON_SAVE_PATH = "../road_polygons_old.pkl"  # Dateipfad zum Speichern der Polygone
xml_file_path = "C:\\Users\\stefan\\Downloads\\FollowLeadingVehicle4.xosc"
VEHICLE_LENGTH = 4.62  # in Meter
VEHICLE_WIDTH = 1.82  # in Meter
off_the_road_vehicles = []
vehicles = []
startPositions = []


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
def is_vehicle_on_road(vehicle_box, combined_polygon):
    return combined_polygon.contains(vehicle_box)
# Funktion zur Überwachung der Fahrzeugposition
def monitor_vehicle_position(vehicle, combined_polygon):
    vehicle.sensors.poll()
    curr_position = vehicle.state['pos'][:2]  # Fahrzeugposition abrufen (x, y)
    vehicle_direction = vehicle.state['dir'][1]  # Fahrzeugausrichtung in Grad

    # Erstelle das Fahrzeugrechteck
    vehicle_box = create_vehicle_box(curr_position, vehicle_direction)

    if not is_vehicle_on_road(vehicle_box, combined_polygon):
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
def vehicle_pos(name, position):
    vehiclepos = (name, position)  # Erstelle ein Tupel aus Autoname und Position
    startPositions.append(vehiclepos)


parameters = sys.argv[2:]
xml_file_paths = []
for parameter in parameters:
    xml_file_path = ""+sys.argv[1]+parameter
    xml_file_paths.append(xml_file_path)
    print("xml_file_path "+xml_file_path)
conditions = parse_conditions_from_xml(xml_file_path)
acts = parse_acts_from_xml(xml_file_path)
xmlVehicles = extract_vehicle_objects(xml_file_path)
events = parse_events_from_xml(xml_file_path)
parameters = extract_Parameter(xml_file_path)
xml_file_path = replace_Parameter(xml_file_path, parameters)

waypointLines = []
waypointCounter = 1
waypointNames = []
for event in events:
    if event.action.action_type == "RoutingAction":
        print("bin RoutingAction")
        for waypoint in event.action.routing_action.waypoints:
            print("bin RoutingAction Waypoint")
            print(waypoint)

            waypointString = "["+waypoint['x']+", "+waypoint['y']+", "+waypoint['z']+"]"
            print(waypointString)
            waypointName = "waypoint"+str(waypointCounter)
            waypointNames.append(waypointName)
            waypointLine = fillWaypointInXML(waypointName, waypointString)
            waypointCounter = waypointCounter + 1
            waypointLines.append(waypointLine)


tree = ET.parse(xml_file_path)
root = tree.getroot()
weather = extract_weather_info(xml_file_path)
print(weather.precipitation_type)
print(weather.precipitation_intensity)
print(weather.time_of_day)
fillWeatherInXML(weather)
#beamngpy.t
#weather.fillWeather(root)
if weather.precipitation_type == "rain":
    if float(weather.precipitation_intensity) <= 0.5:
        weather_preset = "rainy"
    else:
        weather_preset = "heavy_rain"
else:
    weather_preset = "sunny"
print(f"chose weather preset {weather_preset}")
weather_preset = 'rainy'
print(weather.time_of_day)
for name, param in parameters.items():
    print(f"Parameter '{name}': Type = {param['type']}, Value = {param['value']}")
endConditionFinished = False
bng = BeamNGpy('localhost', 64256, home='C:\\Users\\stefan\\Music\\BeamNG.tech.v0.31.3.0\\BeamNG.tech.v0.31.3.0', user='C:\\Users\\stefan\\AppData\\Local\\BeamNG.drive')
# Launch BeamNG.tech
bng.open()
#weather.time_of_day = "03:00:00"
#bng.set_tod(weather.time_of_day)
# Create a scenario in west_coast_usa called 'example'
scenario = Scenario('west_coast_usa', 'example')
state = State()
for vehicle in xmlVehicles:
    v = Vehicle(vehicle.get('name'), model=vehicle.get('vehicle').get('name'), license=vehicle.get('name'))
    vehicles.append(v)
    scenario.add_vehicle(v, pos=(vehicle.get('vehicle').get('X'), vehicle.get('vehicle').get('Y'), vehicle.get('vehicle').get('Z')), cling=True, rot_quat=(0, 0, 0.3826834, 0.9238795))


scenario.make(bng)
print(bng.get_road_edges)
print(bng.get_current_vehicles)
print(bng.get_current_vehicles_info)
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
    vehicle_pos(vehicle.vid, vehicle.state['pos'])
sleep(5)

for vehicle in xmlVehicles:
    for vehicle2 in vehicles:
        if vehicle2.vid == vehicle.get('name'):
            vehicle2.ai.drive_in_lane(True)
            vehicle2.ai.set_mode('span')
            vehicle2.set_velocity(vehicle.get('vehicle').get('maxSpeed')/3.6)
            vehicle2.set_color(vehicle.get('vehicle').get('color'))
            damage_sensor = Damage()
            vehicle2.attach_sensor('damage', damage_sensor)
            #vehicle2.ai.set_waypoint("tunnel_city_D_26")
            print(vehicle2.vid)
# Versuche, die Straßenpolygone aus der Datei zu laden
bng.pause()
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

# Initialisiere die Visualisierung
#fig, ax, vehicle_marker = init_visualization(road_polygons)

#distance = math.sqrt((pos2[0] - pos1[0])**2 + (pos2[1] - pos1[1])**2 + (pos2[2] - pos1[2])**2)
#print(distance)
startReachCondition = False
startTraveledDistanceCondition = False
endTraveledDistanceCondition = False
endReachCondition = False

#vehicles[0].ai.set_waypoint("scenario_test")
#vehicles[1].ai.set_waypoint("scenario_test")
#vehicles[1].ai_set_aggression(1.0)
#vehicles[0].ai_set_aggression(0.1)

bng.resume()
bng.step(3)
for condition in conditions:
    if condition.main_condition_type == 'EndCondition' and condition.condition_type == 'TraveledDistance':
        endTraveledDistanceCondition = True
        print("here1")
        endCondition = condition
        for vehicle in vehicles:
            if condition.mainEntityRef == vehicle.vid:
                print("here2")
                tdstartpos = vehicle.state['pos']
    elif condition.main_condition_type == 'OverallStartCondition' and condition.condition_type == 'TraveledDistance':
        startTraveledDistanceCondition = True
        startCondtion = condition
        for vehicle in vehicles:
            if condition.mainEntityRef == vehicle.vid:
                oscondstartpos = vehicle.state['pos']
    elif condition.main_condition_type == 'OverallStartCondition' and condition.condition_type == 'ReachPosition':
        startReachCondition = True
        startCondtion = condition
    elif condition.main_condition_type == 'EndCondition' and condition.condition_type == 'ReachPosition':
        endReachCondition = True
        endCondition = condition
if 'startCondtion' in locals():
    startConditionFinished = False
else:
    startConditionFinished = True #if there is no startCondition
for event in events:
    if event.condition.condition_type == 'RelativeDistance' and event.condition.main_condition_type == 'OverallStartCondition':
        rdcondition = event.condition
        rdconditionset = True
        atsvalue = event.action.speed_action.target_speed_value
        print(atsvalue)
        print(atsvalue[0])
        if atsvalue[0] == '$':
            print('$')
            if atsvalue[1:] in parameters:
                print(atsvalue)
                atsvalue = parameters[atsvalue[1:]]['value']
                print('atsvalue')
                print(atsvalue)
    elif event.condition.condition_type == 'ReachPosition' and event.condition.main_condition_type == 'OverallStartCondition':
        startReachCondition = True

    if event.condition.condition_type == 'StoryboardElementState':
        sbesconditionref = event.condition.storyboardElementRef
        sbesconditionstate = event.condition.state



while startConditionFinished == False:
    if startTraveledDistanceCondition == True:
        for vehicle in vehicles:
            vehicle.sensors.poll()
            pos1 = vehicle.state['pos']
            #print(pos1)
            vehicle_position = vehicle.state['pos'][:2]
            #update_vehicle_position(vehicle_marker, vehicle_position)
            monitor_vehicle_position(vehicle, road_polygons)
            bng.step(3)  # Simulation fortsetzen
            traveled_distance = np.linalg.norm(np.array(pos1) - oscondstartpos)
            if startCondtion.mainEntityRef == vehicle.vid and traveled_distance >= startCondtion.value:
                print("StartCondition TraveledDistance sucessfull")
                print(traveled_distance)
                startConditionFinished = True
                break
    elif startReachCondition == True:
        print('startReachCondition')
        for vehicle in vehicles:
            vehicle.sensors.poll()
            pos1 = vehicle.state['pos']
            vehicle_position = vehicle.state['pos'][:2]
            #update_vehicle_position(vehicle_marker, vehicle_position)
            monitor_vehicle_position(vehicle, road_polygons)
            bng.step(3)  # Simulation fortsetzen
            if startCondtion.position[0]-startCondtion.tolerance <= pos1[0] <= startCondtion.position[0]+startCondtion.tolerance and startCondtion.position[1] - startCondtion.tolerance <= pos1[1] <= startCondtion.position[1] + startCondtion.tolerance and startCondtion.position[2] - startCondtion.tolerance <= pos1[2] <= startCondtion.position[2] + startCondtion.tolerance:
                print("StartCondition ReachCondition sucessfull")
                print(pos1)
                startConditionFinished = True
                break

print("here")
finishedCondition = None
finishedEvents = []
lastFinishedEvent = None
while endConditionFinished == False:
    for vehicle in vehicles:
        vehicle.sensors.poll()
        vehicle_position = vehicle.state['pos'][:2]
        #update_vehicle_position(vehicle_marker, vehicle_position)
        monitor_vehicle_position(vehicle, road_polygons)
        bng.step(3)  # Simulation fortsetzen
        vehicles[1].sensors.poll()
        print(vehicles[1].vid)
        print(vehicles[1].state['pos'])
        for act in acts:
            for event in act.events:
                if event.condition.condition_type == 'RelativeDistance' and vehicle.vid in act.actors and event.name not in finishedEvents:
                    rdcondition = event.condition
                    rdconditionset = True
                    atsvalue = event.action.speed_action.target_speed_value
                    if atsvalue[0] == '$':
                        if atsvalue[1:] in parameters:
                            atsvalue = parameters[atsvalue[1:]]['value']
                    mainpos = vehicle.state['pos']
                    for vehicle2 in vehicles:
                        if vehicle2.vid == rdcondition.entity_ref:
                            refpos = vehicle2.state['pos']
                            distance_to_car = np.linalg.norm(np.array(refpos) - mainpos)
                            if rdcondition.rule == 'lessThan':
                                if distance_to_car <= rdcondition.value:
                                    vehicle2.ai.set_speed(atsvalue)
                                    print('speed set to '+atsvalue)
                                    #events.remove(event)
                                    print('Relative Distance Condition successful')
                                    lastFinishedEvent = event.name
                                    finishedEvents.append(event.name)
                                    rdconditionset = False
                                    #events.remove(event)
                            elif rdcondition.rule == 'equalTo':
                                if distance_to_car == rdcondition.value:
                                    vehicle2.ai.set_speed(atsvalue)
                                    print('speed set to ')
                                    print(atsvalue)
                                    print('Relative Distance Condition successful')
                                    lastFinishedEvent = event.name
                                    finishedEvents.append(event.name)
                                    #events.remove(event)
                                    rdconditionset = False
                            elif rdcondition.rule == 'greaterThan':
                                if distance_to_car >= rdcondition.value:
                                    vehicle2.ai.set_speed(atsvalue)
                                    print('speed set to ')
                                    print(atsvalue)
                                    print('Relative Distance Condition successful')
                                    lastFinishedEvent = event.name
                                    finishedEvents.append(event.name)
                                    #events.remove(event)
                elif event.condition.condition_type == 'StoryboardElementState' and vehicle.vid in act.actors and event.name not in finishedEvents:
                    print('here1 StoryboardElementState')
                    if event.condition.state == 'endTransition' or event.condition.state == 'completeState':
                        print('here2 endTransition')
                        for finishedEvent in finishedEvents:
                            print("finishedEvent")
                            print(finishedEvent)
                        if event.condition.storyboardElementRef == lastFinishedEvent:
                            print('here3 lastFinishedEvent')
                            print(event.action.action_type)
                            print('delay')
                            print(event.condition.delay)
                            sleep(event.condition.delay)
                            speed = event.action.speed_action.target_speed_value
                            if speed[0] == '$':
                                if speed[1:] in parameters:
                                    speed = parameters[speed[1:]]['value']
                            vehicle.ai.set_speed(int(float(speed)))
                            actSpeed = np.linalg.norm(vehicle.state['vel'])
                            speedSuccess = False
                            while speedSuccess == False:
                                vehicle.sensors.poll()
                                actSpeed = np.linalg.norm(vehicle.state['vel'])
                                if actSpeed < float(speed)+0.5 and actSpeed > float(speed)-1.5:
                                    speedSuccess = True
                                else:
                                    print(f'Aktuelle Geschwindigkeit: {actSpeed} m/s. Warten...')
                                    print(vehicle.vid)
                                    print(vehicle.state['pos'])
                                    bng.step(1)  # Warte 1 Sekunde
                            #w vehicle.state['vel'] == speed
                            print(f"speed: {speed}")
                            print(f"StoryboardElementState of Event: {event.name} finished")
                            finishedEvents.append(event.name)
                            lastFinishedEvent = event.name
                elif event.condition.condition_type == 'TraveledDistance' and vehicle.vid in act.actors and event.name not in finishedEvents:
                    vehicle.sensors.poll()
                    pos1 = vehicle.state['pos']
                    for name, startPosition in startPositions:
                        if name == vehicle.vid:
                            traveled_distance = np.linalg.norm(np.array(pos1) - startPosition)
                            if traveled_distance >= event.condition.value:
                                for waypointname in waypointNames:
                                    vehicle.sensors.poll()
                                    pos1 = vehicle.state['pos']
                                    vehicleOnPosition = False
                                    vehicle.ai.set_waypoint("waypointname")
                                    print("Waypoint "+waypointname + " set.")
                    finishedEvents.append(event.name)



        if 'endCondition' in locals():
            vehicle.sensors.poll()
            pos1 = vehicle.state['pos']
            #print(pos1)
            traveled_distance = np.linalg.norm(np.array(pos1) - tdstartpos)
            print(traveled_distance)
            print('Test')
            if endCondition.main_condition_type == 'EndCondition' and endCondition.condition_type == 'TraveledDistance':
                if endCondition.mainEntityRef == vehicle.vid and traveled_distance >= endCondition.value:
                    vehicle.control(throttle=0, brake=1)
                    vehicle.ai.set_speed(0)
                    sleep(10)
                    endConditionFinished = True
                    print("Endcondition TraveledDistance successfull")
                    break
            if endCondition.main_condition_type == 'EndCondition' and endCondition.condition_type == 'ReachCondition':
                if endCondition.mainEntityRef == vehicle.vid and traveled_distance >= endCondition.value:
                    pos1 = vehicle.state['pos']
                    if endCondition.position[0] - endCondition.tolerance <= pos1[0] <= endCondition.position[0] + endCondition.tolerance and endCondition.position[1] - endCondition.tolerance <= pos1[1] <= endCondition.position[1] + endCondition.tolerance and endCondition.position[2] - endCondition.tolerance <= pos1[2] <= endCondition.position[2] + endCondition.tolerance:
                        print("EndCondition ReachCondition sucessfull")
                        print(pos1)
                        vehicle.control(throttle=0, brake=1)
                        vehicle.ai.set_speed(0)
                        endConditionFinished = True
                        break

for condition in conditions:
    if condition.main_condition_type == 'criteria_CollisionTest':
        state = State()
        for vehicle in vehicles:
            print(vehicle.vid + ': ')
            if vehicle.sensors['damage'].data['damage'] > 0:
                print("Fahrzeug hat Schaden")
                print(vehicle.sensors['damage'].data['damage'])
            else:
                print("Fahrzeug hat keinen Schaden")
    if not off_the_road_vehicles:
        print("Keine Fahrzeuge haben die Straße verlassen")
    else:
        print("Folgende Fahrzeuge haben die Straße verlassen:")
        for vehicle in off_the_road_vehicles:
            print(vehicle.vid)
for waypointLine in waypointLines:
    removeWaypointInXML(waypointLine)
bng.close()

