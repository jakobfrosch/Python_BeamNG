import beamngpy
from beamngpy import Vehicle, Scenario, Road, BeamNGpy
from beamngpy import BeamNGpy, Scenario, Vehicle, Road
from time import sleep
# Connect to the BeamNG.tech game instance

try:
    # Load a scenario
    beamng = BeamNGpy('localhost', 64256, home='D:\BeamNG.tech.v0.31.2.0\BeamNG.tech.v0.31.2.0',
                      user='C:\\Users\Stefan\AppData\Local\BeamNG.drive')
    beamng.open()
    scenario = Scenario('smallgrid', 'roads_importer')
    vehicle = Vehicle('vehicle', model='pickup', licence='PYTHON')
    scenario.add_vehicle(vehicle, pos=(0, 0, 0), rot_quat=(0, 0, 45, 0))
    road = Road('track_editor_C_center', looped=False)
    road.nodes.append((0, 0, 0, 5))
    road.nodes.append((200, 0, 0, 5))  # Change 200 to the desired distance
    scenario.add_road(road)

    # Start the scenario
    scenario.make(beamng)
    beamng.load_scenario(scenario)
    beamng.start_scenario()

    # Drive the vehicle
    vehicle.ai_set_mode('span')
    vehicle.ai_drive_in_lane(True)
    vehicle.ai_set_speed(20)  # Change 20 to the desired speed in m/s

    # Wait until the vehicle has driven the specified distance
    #while vehicle.state['pos'][0] < 200:  # Change 200 to the desired distance
    #    beamng.step(60)  # Adjust the step duration as needed
    #sleep(10)
    # Stop the vehicle
    while True:
        current_pos = vehicle.state['pos']
        if current_pos[0] >= 200:  # Change 200 to the desired distance
            break
        beamng.step(60)  # Adjust the step duration as needed
    vehicle.ai_set_speed(0)

finally:
    # Cleanup and close the BeamNG.tech instance
    beamng.close()
