import math
import os
import random
from time import sleep
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

from Condition import parse_conditions_from_xml
from Vehicle import extract_vehicle_objects
from Event import parse_events_from_xml
from Act import parse_acts_from_xml
from Weather import Weather, extract_weather_info
from updateZIP import extend_file_in_zip, fillWeatherInXML

vehicles = []

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

xml_file_path = "C:\\Users\\stefan\\Downloads\\FollowLeadingVehicle3.xosc"
parameters = sys.argv[2:]
for parameter in parameters:
    xml_file_path = ""+sys.argv[1]+parameter
    print(xml_file_path)
conditions = parse_conditions_from_xml(xml_file_path)
acts = parse_acts_from_xml(xml_file_path)
scenario_objects = extract_vehicle_objects(xml_file_path)
events = parse_events_from_xml(xml_file_path)
parameters = extract_Parameter(xml_file_path)
xml_file_path = replace_Parameter(xml_file_path, parameters)


tree = ET.parse(xml_file_path)
root = tree.getroot()
weather = extract_weather_info(xml_file_path)
print(weather.precipitation_type)
print(weather.precipitation_intensity)
print(weather.time_of_day)
fillWeatherInXML(weather)

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
bng = BeamNGpy('localhost', 64256, home='C:\\Users\\stefan\\Downloads\\BeamNG.tech.v0.31.3.0\\BeamNG.tech.v0.31.3.0', user='C:\\Users\\stefan\\AppData\\Local\\BeamNG.drive')
# Launch BeamNG.tech
bng.open()
#weather.time_of_day = "03:00:00"
#bng.set_tod(weather.time_of_day)
# Create a scenario in west_coast_usa called 'example'
scenario = Scenario('west_coast_usa', 'example')
state = State()
for vehicle in scenario_objects:
    v = Vehicle(vehicle.get('name'), model=vehicle.get('vehicle').get('name'), license=vehicle.get('name'))
    vehicles.append(v)
    scenario.add_vehicle(v, pos=(vehicle.get('vehicle').get('X'), vehicle.get('vehicle').get('Y'), vehicle.get('vehicle').get('Z')), cling=True, rot_quat=(0, 0, 0.3826834, 0.9238795))


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

sleep(5)
for vehicle in scenario_objects:
    for vehicle2 in vehicles:
        if vehicle2.vid == 'hero':
            vehicle2.sensors.poll()
            pos1 = vehicle2.state['pos']
        if vehicle2.vid == 'adversary':
            vehicle2.sensors.poll()
            pos2 = vehicle2.state['pos']
        if vehicle2.vid == vehicle.get('name'):
            vehicle2.ai.drive_in_lane(True)
            vehicle2.ai.set_mode('span')
            vehicle2.set_velocity(vehicle.get('vehicle').get('maxSpeed')/3.6)
            vehicle2.set_color(vehicle.get('vehicle').get('color'))
            damage_sensor = Damage()
            vehicle2.attach_sensor('damage', damage_sensor)
            print(vehicle2.vid)
distance = math.sqrt((pos2[0] - pos1[0])**2 + (pos2[1] - pos1[1])**2 + (pos2[2] - pos1[2])**2)
print(distance)
startReachCondition = False
startTraveledDistanceCondition = False
endTraveledDistanceCondition = False
endReachCondition = False

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
                print("here3")
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
    startConditionFinished = True
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
                                    print('speed set to ')
                                    print(atsvalue)
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
                                    bng.step(1)  # Warte 1 Sekunde
                            #w vehicle.state['vel'] == speed
                            print(f"speed: {speed}")
                            print(f"StoryboardElementState of Event: {event.name} finished")
                            finishedEvents.append(event.name)
                            lastFinishedEvent = event.name


        if 'endCondition' in locals():
            vehicle.sensors.poll()
            pos1 = vehicle.state['pos']
            print(pos1)
            traveled_distance = np.linalg.norm(np.array(pos1) - tdstartpos)
            print(traveled_distance)
            print('Test')
            if endCondition.main_condition_type == 'EndCondition' and endCondition.condition_type == 'TraveledDistance':
                if endCondition.mainEntityRef == vehicle.vid and traveled_distance >= endCondition.value:
                    print('Test2')
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
                        sleep(10)
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

#bng.close()

