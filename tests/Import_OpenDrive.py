from beamngpy import BeamNGpy, MeshRoad, Road, Scenario, Vehicle, set_up_simple_logging, tools, logging
from beamngpy.tools import (OpenDriveImporter, OpenStreetMapImporter,
                            SumoImporter)
from matplotlib import pyplot as plt
from shapely.geometry import MultiLineString

def main():
    set_up_simple_logging()

    # Initialize BeamNG.
    print("Initializing BeamNG...")
    beamng = BeamNGpy('localhost', 64256, home='D:\BeamNG.tech.v0.30.6.0\BeamNG.tech.v0.30.6.0', user='C:\\Users\Stefan\AppData\Local\BeamNG.drive')

    beamng.open(launch=True)
    scenario = Scenario('smallgrid', 'roads_importer')
    vehicle = Vehicle('ego_vehicle', model='citybus')

    scenario.add_vehicle(vehicle,pos=(1.75, -17.12251064972856, 5.0))

    #scenario.add_road(road_a)
    #scenario.add_road()
    # Import OpenDrive (.xodr).
    filename = 'D:\\BeamNG.tech.v0.30.6.0\BeamNG.tech.v0.30.6.0\Ex_LHT-Complex-X-Junction.xodr'
    OpenDriveImporter.import_xodr(filename, scenario)                      # import an OpenDrive file (.xodr).


    # Start up BeamNG with the imported road network.
    print("Making scenario...")
    scenario.make(beamng)
    beamng.scenario.load(scenario)
    beamng.scenario.start()
    roads = beamng.scenario.get_roads()
    print(roads)
    road_names = list(roads.keys())
    road_spec = {}
    for r_id, r_inf in roads.items():
        if r_inf['drivability'] != '-1':
            road_spec[r_id] = beamng.scenario.get_road_edges(r_id)

    road = list()
    lines = list()
    for r_id in road_spec.keys():
        left = list()
        right = list()
        for r_point in road_spec[r_id]:
            x = r_point['left'][0]
            y = r_point['left'][1]
            left.append((x, y))
            x = r_point['right'][0]
            y = r_point['right'][1]
            right.append((x, y))
        if left:
            lines.append(tuple(left))
        if right:
            lines.append(tuple(right))

    network = MultiLineString(lines)

    # plot map
    def plot_lines(ax, ob):
        blue = '#6699cc'
        for line in ob.geoms:
            x, y = line.xy
            ax.plot(x, y, color=blue, linewidth=1,
                    solid_capstyle='round', zorder=2, alpha=0.7)

    fig = plt.figure(1, figsize=[9.6, 9.6], dpi=100)

    ax = fig.add_subplot()
    plot_lines(ax, network)

    _ = ax.set_axis_off()
    _ = ax.set_title('road network West Coast, USA')
    plt.savefig('leftright.png')




    # Execute BeamNG until the user closes it.
    print("Completed.")
    while(True):
        pass
    #beamng.close()


if __name__ == '__main__':
    main()

