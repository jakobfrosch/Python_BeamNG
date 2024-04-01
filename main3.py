import math
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
vehicles = []

xml_file_path = "C:\\Users\\stefan\\Downloads\\FollowLeadingVehicle2.xosc"
conditions = parse_conditions_from_xml(xml_file_path)
scenario_objects = extract_vehicle_objects(xml_file_path)
endcondition = False
bng = BeamNGpy('localhost', 64256, home='C:\\Users\\stefan\\Downloads\\BeamNG.tech.v0.31.3.0\\BeamNG.tech.v0.31.3.0', user='C:\\Users\\stefan\\AppData\\Local\\BeamNG.drive')
# Launch BeamNG.tech
bng.open()
# Create a scenario in west_coast_usa called 'example'
scenario = Scenario('west_coast_usa', 'example')
state = State()
for vehicle in scenario_objects:
    v = Vehicle(vehicle.get('name'), model=vehicle.get('vehicle').get('name'), license=vehicle.get('name'))
    vehicles.append(v)
    scenario.add_vehicle(v, pos=(vehicle.get('vehicle').get('X'), vehicle.get('vehicle').get('Y'), vehicle.get('vehicle').get('Z')), cling=True, rot_quat=(0, 0, 0.3826834, 0.9238795))


scenario.make(bng)



# Entfernung zwischen den Fahrzeugen berechnen
rdconditionset = False
pos1 = 0
pos2 = 0
bng.scenario.load(scenario)
bng.scenario.start()
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
for condition in conditions:
    if condition.main_condition_type == 'EndCondition' and condition.condition_type == 'TraveledDistance':
        print("here1")
        tdcondition = condition
        for vehicle in vehicles:
            if condition.mainEntityRef == vehicle.vid:
                print("here2")
                tdstartpos = vehicle.state['pos']
    elif condition.main_condition_type == 'OverallStartCondition' and condition.condition_type == 'TraveledDistance':
        oscondition = condition
        for vehicle in vehicles:
            if condition.mainEntityRef == vehicle.vid:
                print("here3")
                oscondstartpos = vehicle.state['pos']
    elif condition.condition_type == 'RelativeDistance':
        rdcondition = condition
        rdconditionset = True
if 'oscondstartpos' in locals() or 'oscondstartpos' in globals():
    startCondition = False
else:
    startCondition = True


while startCondition == False:
    for vehicle in vehicles:
        vehicle.sensors.poll()
        pos1 = vehicle.state['pos']
        #print(pos1)
        traveled_distance = np.linalg.norm(np.array(pos1) - oscondstartpos)
        if oscondition.mainEntityRef == vehicle.vid and traveled_distance >= oscondition.value:
            print("StartCondition TraveledDistance sucessfull")
            print(traveled_distance)
            startCondition = True
            break
print("here")
while endcondition == False:

    for vehicle in vehicles:
        vehicle.sensors.poll()
        if rdconditionset:
            if rdcondition.mainEntityRef == vehicle.vid:
                mainpos = vehicle.state['pos']
                for vehicle2 in vehicles:
                    if vehicle2.vid == rdcondition.entity_ref:
                        refpos = vehicle2.state['pos']
                        distance_to_car = np.linalg.norm(np.array(refpos) - mainpos)
                        if rdcondition.rule == 'lessThan':
                            if distance_to_car <= rdcondition.value:
                                print('Relative Distance Condition successful')
                                rdconditionset = False
                        elif rdcondition.rule == 'equalTo':
                            if distance_to_car == rdcondition.value:
                                print('Relative Distance Condition successful')
                                rdconditionset = False
                        elif rdcondition.rule == 'greaterThan':
                            if distance_to_car >= rdcondition.value:
                                print('Relative Distance Condition successful')
                                rdconditionset = False
                        if rdconditionset == False:
                            vehicle2.ai.set_speed(0)
                            sleep(10)
                            vehicle2.ai_set_speed(30)
        if 'oscondstartpos' in locals() or 'oscondstartpos' in globals():
            vehicle.sensors.poll()
            pos1 = vehicle.state['pos']
            print(pos1)
            traveled_distance = np.linalg.norm(np.array(pos1) - tdstartpos)
            print(traveled_distance)
            print('Test')
            if tdcondition.mainEntityRef == vehicle.vid and traveled_distance >= tdcondition.value:
                print('Test2')
                vehicle.control(throttle=0, brake=1)
                vehicle.ai.set_speed(0)
                sleep(100)
                endcondition = True
                print("Endcondition TraveledDistance successfull")
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

bng.close()

