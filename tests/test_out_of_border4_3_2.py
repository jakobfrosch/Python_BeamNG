import math
import pickle
import os
from shapely.geometry import Point, MultiLineString
from beamngpy import BeamNGpy, Scenario, Vehicle
import matplotlib.pyplot as plt
from beamngpy.sensors import State
import numpy as np
#t-wise-coverage / random /..
# Pfad zur Datei, in der das Straßennetzwerk gespeichert wird
NETWORK_SAVE_PATH = "../road_network.pkl"


# Funktion zur Überprüfung, ob sich das Fahrzeug in einem definierten Puffer um die Straßen befindet
def is_vehicle_on_road(vehicle_position, road_network):  # Buffer-Größe auf 2.0 Meter gesetzt
    vehicle_point = Point(vehicle_position)  # Fahrzeug als Punkt
    #road_buffer = road_network.buffer(buffer_size)  # Puffer um die Straße
    return road_network.contains(vehicle_point)


# Visualisierung des Straßennetzwerks und der Fahrzeugposition (mit kontinuierlicher Aktualisierung)
def plot_lines(ax, road_network):
    blue = '#6699cc'
    for line in road_network.geoms:
        x, y = line.xy
        ax.plot(x, y, color=blue, linewidth=1, solid_capstyle='round', zorder=2, alpha=0.7)


# Visualisierung des Straßennetzwerks und der Fahrzeugposition
def plot_network_and_vehicle(road_network, vehicle_position):
    fig, ax = plt.subplots()

    # Zeichne das Straßennetzwerk
    plot_lines(ax, road_network)

    # Zeichne die Fahrzeugposition als roten Punkt
    if vehicle_position:
        ax.plot(vehicle_position[0], vehicle_position[1], 'ro', markersize=10, label='Vehicle')

    ax.set_aspect('equal', 'box')
    ax.set_title('Road Network with Vehicle Position')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.grid(True)

    return fig, ax


# Funktion zum Speichern des Straßennetzwerks
def save_network(road_network):
    with open(NETWORK_SAVE_PATH, 'wb') as f:
        pickle.dump(road_network, f)
    print(f"Road network saved to {NETWORK_SAVE_PATH}")


# Funktion zum Laden des Straßennetzwerks
def load_network():
    if os.path.exists(NETWORK_SAVE_PATH):
        with open(NETWORK_SAVE_PATH, 'rb') as f:
            road_network = pickle.load(f)
        print(f"Road network loaded from {NETWORK_SAVE_PATH}")
        return road_network
    return None


# Funktion zur Protokollierung, wenn das Fahrzeug von der Straße abkommt
def log_off_road(position):
    print(f"Vehicle went off-road at position: {position}")

def update_vehicle_position(ax, road_network, vehicle_position):
    ax.cla()  # Achse leeren
    plot_lines(ax, road_network)  # Straßennetzwerk neu zeichnen
    ax.plot(vehicle_position[0], vehicle_position[1], 'ro', markersize=10, label='Vehicle')
    ax.set_aspect('equal', 'box')
    plt.pause(0.01)  # Kurz pausieren, um die Anzeige zu aktualisieren
# Hauptprogramm
def main():
    bng = BeamNGpy('localhost', 64256, home='C:\\Users\\stefan\\Music\\BeamNG.tech.v0.31.3.0\\BeamNG.tech.v0.31.3.0',
                   user='C:\\Users\\stefan\\AppData\\Local\\BeamNG.drive')
    bng.open()

    scenario = Scenario('west_coast_usa', 'example')
    state = State()
    vehicle = Vehicle('ego_vehicle', model='etk800', licence='PYTHON')
    scenario.add_vehicle(vehicle, pos=(-462.3130026726867, -748.9811885876843, 141.67463761260296),
                         rot_quat=(0, 0, -0.5, 0.8660254))

    scenario.make(bng)
    bng.scenario.load(scenario)
    bng.scenario.start()
    vehicle.sensors.poll()

    # Versuche, das Straßennetzwerk zu laden
    road_network = load_network()

    # Wenn kein gespeichertes Netzwerk vorhanden ist, extrahiere die Straßendaten und speichere es
    if road_network is None:
        print("Extracting road network...")
        roads = bng.scenario.get_roads()
        road_names = list(roads.keys())
        road_spec = {}
        for r_id, r_inf in roads.items():
            if r_inf['drivability'] != '-1':
                print('drivability')
                road_spec[r_id] = bng.scenario.get_road_edges(r_id)

        # Erstellen eines MultiLineString für das Straßennetzwerk
        lines = []
        for r_id in road_spec.keys():
            left = []
            right = []
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

        road_network = MultiLineString(lines)
        save_network(road_network)  # Speichern des Straßennetzwerks

    # Visualisierung des Straßennetzwerks
    fig, ax = plot_network_and_vehicle(road_network, vehicle.state['pos'][:2])

    # Funktion zur Aktualisierung der Fahrzeugposition auf der Karte
    try:
        while True:
            vehicle.sensors.poll()
            curr_position = vehicle.state['pos'][:2]

            # Überprüfe, ob das Fahrzeug sich auf der Straße befindet (als Punkt)
            if not is_vehicle_on_road(curr_position, road_network):
                log_off_road(curr_position)
            else:
                print('in')

            # Aktualisiere die Visualisierung der Fahrzeugposition
            update_vehicle_position(ax, road_network, curr_position)

            # Simulationsschritt
            #bng.step(3)
    finally:
        bng.close()


if __name__ == "__main__":
    main()
