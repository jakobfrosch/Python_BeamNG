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

from Vehicle import extract_vehicle_objects
vehicles = []
xml_file_path = "C:\\Users\\stefan\\Downloads\\FollowLeadingVehicle2.xosc"
scenario_objects = extract_vehicle_objects(xml_file_path)

bng = BeamNGpy('localhost', 64256, home='C:\\Users\\stefan\\Downloads\\BeamNG.tech.v0.31.3.0\\BeamNG.tech.v0.31.3.0', user='C:\\Users\\stefan\\AppData\\Local\\BeamNG.drive')
# Launch BeamNG.tech
bng.open()
# Create a scenario in west_coast_usa called 'example'
scenario = Scenario('west_coast_usa', 'example')

for vehicle in scenario_objects:
    v = Vehicle(vehicle.get('name'), model=vehicle.get('vehicle').get('name'), license=vehicle.get('name'))
    vehicles.append(v)
    scenario.add_vehicle(v, pos=(vehicle.get('vehicle').get('X'), vehicle.get('vehicle').get('Y'), vehicle.get('vehicle').get('Z')), rot_quat=(0, 0, 0.3826834, 0.9238795))
    print(v.vid)
#bng.settings.change("disableDynamicCollision", "true")
# Place files defining our scenario for the simulator to read
scenario.make(bng)

state = State()
bng.scenario.load(scenario)
sleep(10)
for vehicle in scenario_objects:
    for vehicle2 in vehicles:
        if vehicle2.vid == vehicle.get('name'):
            vehicle2.ai.drive_in_lane(True)
            vehicle2.ai.set_mode('span')
            vehicle2.set_velocity(vehicle.get('vehicle').get('maxSpeed')/3.6)
            #vehicle.control(engineOutput=0.3)
            #vehicle2.control(throttle=0.3)
            vehicle2.set_color(vehicle.get('vehicle').get('color'))
            print(vehicle2.vid)

bng.scenario.start()
while True:
    for vehicle2 in vehicles:
        vehicle2.ai.drive_in_lane(True)
        vehicle2.sensors.poll()
        print(vehicle2.vid)

#bng.close()

