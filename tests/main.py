from time import sleep
from beamngpy import BeamNGpy, Scenario
from beamngpy.sensors import Lidar
from beamngpy.sensors import Radar
from beamngpy.sensors import State
from beamngpy.vehicle import Vehicle
import numpy as np
from beamngpy.sensors import Damage
import sys

import xml.etree.ElementTree as ET

from src import Weather
from src.Condition import parse_conditions_from_xml
# Instantiate BeamNGpy instance running the simulator from the given path,
# communicating over localhost:64256
parameters = sys.argv[2:]

# Gib die Übergabeparameter aus
print("Jakob Test2")
print (sys.argv[2])
dis=0
xml_file_path = "C:\\Users\\Stefan\\PedestrianCrossingFront.xosc"
conditions = parse_conditions_from_xml(xml_file_path)
for condition in conditions:
    if condition.condition_type == 'TraveledDistance' and condition.main_condition_type == 'EndCondition':
        dis = condition.value
        print('distance ')
        print(dis)

for parameter in parameters:
    xml_file_path = ""+sys.argv[1]+parameter
    print(xml_file_path)

print(len(sys.argv[1:]))
bng = BeamNGpy('localhost', 64256, home='C:\\Users\\stefan\\Downloads\\BeamNG.tech.v0.31.3.0\\BeamNG.tech.v0.31.3.0', user='C:\\Users\\stefan\\AppData\\Local\\BeamNG.drive')
# Launch BeamNG.tech
bng.open()
# Create a scenario in west_coast_usa called 'example'
scenario = Scenario('west_coast_usa', 'example')
# Create an ETK800 with the licence plate 'PYTHON'
config = 'vehicles/etk800/854_police_A.pc'
vehicle = Vehicle('ego_vehicle', model='etk800', partConfig=config, license='Jakob<3')
vehicle2 = Vehicle('vehicle', model='etk800', license='blue')
bng.settings.change("disableDynamicCollision", "true")
damage_sensor = Damage()
vehicle.attach_sensor('damage', damage_sensor)
# Add it to our scenario at this position and rotation
scenario.add_vehicle(vehicle, pos=(-725, 101, 118), rot_quat=(0, 0, 0.3826834, 0.9238795))
scenario.add_vehicle(vehicle2, pos=(-737.4384916126728, 88.3228699862957, 118.36918145092204), rot_quat=(0, 0, 0.3826834, 0.9238795))

# Place files defining our scenario for the simulator to read
scenario.make(bng)
print(bng.env.get_tod)
#vehicle.set_color((245, 40, 145, 0.8))
# Load and start our scenario

bng.scenario.load(scenario)
vehicle2.ai.set_mode('span')
vehicle2.ai.drive_in_lane(True)
vehicle.set_velocity(150/3.6, 1.0)
vehicle.ai.set_waypoint('Bridge23_1')
vehicle2.set_velocity(70/3.6, 1.0)
vehicle2.ai.set_waypoint('Bridge23_1')
vehicle.ai.drive_in_lane(True)
bng.scenario.start()
vehicle.ai.set_mode('span')
vehicle.ai.set_waypoint()
lidar = Lidar('lidar1', bng, vehicle, requested_update_time=0.01, is_using_shared_memory=False)
radar = Radar('radar1', bng, vehicle, requested_update_time=0.01)
state = State()
# Make the vehicle's AI span the map



#vehicle2.ai.set_speed(100/3.6, 'set')

start_pos1=(-725, 101, 118)
start_pos2=(-730, 102, 119)
vehicle.sensors.poll()
vehicle2.sensors.poll()
distance1 = np.linalg.norm(np.array(vehicle.state['pos']) - start_pos1)
distance2 = np.linalg.norm(np.array(vehicle.state['pos']) - start_pos2)
print(distance1)
print(distance2)

#vehicle.ai.set_speed(100/3.6, 'set')

print()

#sleep(10)
#vehicle2.ai.set_speed(0,1)
#vehicle.ai.set_target('vehicle','chase')
#vehicle.set_license_plate('Jakob')
#bng.set_weather_preset('')
vehicle2.set_color('red')
vehicle.set_color('red');
vehicle.set_velocity(150/3.6, 1.0)
vehicle2.set_velocity(70/3.6, 1.0)
# Pfad zur XML-Datei

# XML-Datei einlesen
tree = ET.parse(xml_file_path)
root = tree.getroot()

# Wetterinformationen extrahieren und in der Weather-Klasse speichern
weather = Weather()
weather.fillWeather(root)

# Ausgabe der extrahierten Informationen
print(f"Weather_cloudState: {weather.cloud_state}")
print(f"Sun_azimuth: {weather.sun_azimuth}")
print(f"Sun_elevation: {weather.sun_elevation}")
print(f"Sun_intensity: {weather.sun_intensity}")
print(f"TimeOfDay_dateTime: {weather.time_of_day}")

# Beispiel für die Verwendung der saveWeather-Methode
weather.saveWeather()
bng.env.set_tod(weather.time_of_day)
bng.env.set_weather_preset("sunny")
#bng.traffic.spawn()
#vehicle.ai_set_aggression
#bng.env.set_weather_preset()
State.connect(state, bng, vehicle)
print(state.get('pos'))
vehicle.sensors.poll()                                                      # Plot vehicle position and direction.
pos = vehicle.state['pos']
print(pos)
print(pos[0])
distance1 = np.linalg.norm(np.array(vehicle.state['pos']) - start_pos1)
print(distance1)



print(pos)
print('Driving around, polling the LiDAR sensor every 5 seconds...')


def calculate_brake_force(current_speed, target_distance, deceleration_rate):
    # Berechne die Zeit, die benötigt wird, um die gewünschte Distanz zu erreichen
    time_to_target = np.sqrt(2 * target_distance / deceleration_rate)
    # Berechne die erforderliche Geschwindigkeitsänderung, um innerhalb dieser Zeit zum Stillstand zu kommen
    required_speed_change = current_speed - (deceleration_rate * time_to_target)
    # Berechne die Bremskraft basierend auf der Geschwindigkeitsänderung und dem Fahrzeuggewicht
    vehicle_info = bng.get_vehicle_info(vehicle)
    vehicle_mass = vehicle_info['mass']
    brake_force = required_speed_change / vehicle_mass
    return brake_force


for i in range(1000):
    sleep(1)
    readings_data = lidar.poll() # Fetch the latest readings from the sensor, over either shared memory or the lua socket.
    print('LiDAR Point Cloud Data after ', i, ' seconds: ', readings_data['pointCloud'][0:10])       # The first 10 points from LiDAR point cloud data.
    # print('LiDAR Colour Data after ', i * 5, ' seconds: ', readings_data['colours'][0:10])             # The colour data (corresponds to each point).
    vehicle.sensors.poll()
    vehicle2.sensors.poll()
    pos1 = vehicle.state['pos']
    pos2 = vehicle2.state['pos']
    distance1 = np.linalg.norm(np.array(pos1) - start_pos1)
    distance2 = np.linalg.norm(np.array(pos2) - start_pos2)
    target_distance = dis
    pos = vehicle.state['pos']
    vel = vehicle.state['vel']
    # Rate der Verzögerung (Bremskraft), um das Fahrzeug zu stoppen (in m/s^2)
    deceleration_rate = 5
    # Berechne die Distanz zum Ziel
    distance_to_target = np.linalg.norm(np.array(pos)) - target_distance
    if distance_to_target <= 0:
        vehicle.control(throttle=0, brake=1)
        break
    else:
        # Berechne die Bremskraft basierend auf der aktuellen Geschwindigkeit
        brake_force = calculate_brake_force(np.linalg.norm(np.array(vel)), distance_to_target, deceleration_rate)
        # Steuere das Fahrzeug mit der berechneten Bremskraft
        vehicle.control(brake=brake_force)
    #if distance1 >= dis:
        #vehicle.ai_set_speed(0)
    #    vehicle.control(throttle=0, brake=1)
    #if distance2 > 310:
    #    vehicle2.ai.set_speed(0)
    print(pos1)
    print(distance1)
    print(distance2)
    speed = vehicle.state['vel']

    # Berechne den Betrag des Geschwindigkeitsvektors
    speed_magnitude = (speed[0] ** 2 + speed[1] ** 2 + speed[2] ** 2) ** 0.5

    # Wenn die Geschwindigkeit nahezu null ist, gilt das Fahrzeug als gestoppt
    if speed_magnitude < 0.1:
        damage_info = vehicle.sensors['damage'].data
        print(np.sum(damage_info))

        # Überprüfen, ob ein Crash stattgefunden hat
        print(vehicle.sensors['damage'].data['damage'] > 0)

        bng.close()

RANGE_MIN = 0.1
RANGE_MAX = 100.0
RESOLUTION = (200, 200)
FOV = 70
radar = Radar('radar1', bng, vehicle,
              requested_update_time=0.01,
              pos=(0, 0, 1.7), dir=(0, -1, 0), up=(0, 0, 1),
              resolution=RESOLUTION, field_of_view_y=FOV, near_far_planes=(RANGE_MIN, RANGE_MAX),
              range_roundess=-2.0, range_cutoff_sensitivity=0.0, range_shape=0.23, range_focus=0.12, range_min_cutoff=0.5, range_direct_max_cutoff=RANGE_MAX)
for _ in range(1000):
        sleep(5)
        # Fetch the latest readings from the sensor, over either shared memory or the lua socket.
        readings_data = radar.poll()
        radar.plot_data(readings_data, RESOLUTION, FOV, RANGE_MIN, RANGE_MAX, 200, 200)

lidar.remove()
radar.remove()
#print(bng.traffic.)
bng.close()

