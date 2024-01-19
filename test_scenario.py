from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.tools import (OpenDriveImporter, OpenStreetMapImporter,
                            SumoImporter)


def main():
    set_up_simple_logging()

    # Initialize BeamNG.
    print("Initializing BeamNG...")
    beamng = BeamNGpy('localhost', 64256, home='D:\BeamNG.tech.v0.30.6.0\BeamNG.tech.v0.30.6.0', user='C:\\Users\Stefan\AppData\Local\BeamNG.drive')
    beamng.open(launch=True)
    scenario = Scenario('smallgrid', 'roads_importer')
    vehicle = Vehicle('ego_vehicle', model='etk800')
    scenario.add_vehicle(vehicle)

    # Import OpenDrive (.xodr).
    filename = 'D:\\BeamNG.tech.v0.30.6.0\BeamNG.tech.v0.30.6.0\Ex_LHT-Complex-X-Junction.xodr'
    OpenDriveImporter.import_xodr(filename, scenario)                      # import an OpenDrive file (.xodr).

    # Import OpenStreetMap (.osm).
    # filename = 'map.osm'
    # OpenStreetMapImporter.import_osm(filename, scenario)                 # import an OpenStreetMap file (.osm).

    # prefix = 'back'                                                      # Import Sumo files (.nod.xml, .edg.xml).
    # SumoImporter.import_sumo(prefix, scenario)

    # Start up BeamNG with the imported road network.
    print("Making scenario...")
    scenario.make(beamng)
    beamng.scenario.load(scenario)
    beamng.scenario.start()

    # Execute BeamNG until the user closes it.
    print("Completed.")
    while(True):
        pass
    beamng.close()
