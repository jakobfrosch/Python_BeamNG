import math
import pickle
import os
from shapely.geometry import Polygon, Point
from shapely.ops import unary_union
from beamngpy import BeamNGpy, Scenario, Vehicle
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPolygon
from beamngpy.sensors import State
import numpy as np

POLYGON_SAVE_PATH = "../road_polygons.pkl"

# Fahrzeugabmessungen (zum Beispiel)
VEHICLE_LENGTH = 4.69  # in Meter
VEHICLE_WIDTH = 1.84  # in Meter
counter = 0


# Funktion zum Prüfen, ob zwei Punkte nah beieinander liegen
def are_close(coord1, coord2, tolerance=1.0):
    return np.linalg.norm(np.array(coord1) - np.array(coord2)) < tolerance


# Filtert doppelte oder überlappende Straßenkanten
def filter_duplicate_edges(road_edges, tolerance=1.0):
    filtered_edges = []

    for edge in road_edges:
        start = edge['left'][:2]
        end = edge['right'][:2]

        duplicate = False
        for filtered_edge in filtered_edges:
            existing_start = filtered_edge['left'][:2]
            existing_end = filtered_edge['right'][:2]

            if are_close(start, existing_start, tolerance) and are_close(end, existing_end, tolerance):
                duplicate = True
                break

        if not duplicate:
            filtered_edges.append(edge)

    return filtered_edges


# Funktion zum Erstellen eines Straßenpolygons aus den Straßenkanten
def create_road_polygon(road_edges):
    filtered_edges = filter_duplicate_edges(road_edges, tolerance=2.0)  # Filter mit Toleranz
    left_points = [edge['left'][:2] for edge in filtered_edges]
    right_points = [edge['right'][:2] for edge in reversed(filtered_edges)]

    road_polygon = Polygon(left_points + right_points)

    if not road_polygon.is_valid:
        print("Warning: Das Straßenpolygon ist ungültig.")
    else:
        print(f"Straßenpolygon ist gültig.")
    return road_polygon


# Fahrzeugrechteck basierend auf der Position und Ausrichtung berechnen
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


# Funktion zur Überprüfung, ob das Fahrzeug auf der Straße ist
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


def log_off_road(position):
    print(f"Vehicle went off-road at position: {position}")


# Polygone speichern
def save_polygons(road_polygons):
    with open(POLYGON_SAVE_PATH, 'wb') as f:
        pickle.dump(road_polygons, f)
    print(f"Road polygons saved to {POLYGON_SAVE_PATH}")


# Polygone laden
def load_polygons():
    if os.path.exists(POLYGON_SAVE_PATH):
        with open(POLYGON_SAVE_PATH, 'rb') as f:
            road_polygons = pickle.load(f)
        print(f"Road polygons loaded from {POLYGON_SAVE_PATH}")
        return road_polygons
    return None


# Visualisierung der Straßenpolygone
def visualize_polygons(road_polygons, vehicle_position=None):
    fig, ax = plt.subplots()

    for polygon in road_polygons:
        if not polygon.is_empty and polygon.is_valid:
            try:
                mpl_poly = MplPolygon(list(polygon.exterior.coords), closed=True, edgecolor='blue', facecolor='none')
                ax.add_patch(mpl_poly)
            except Exception as e:
                print(f"Error plotting polygon: {e}")

    if vehicle_position:
        ax.plot(vehicle_position[0], vehicle_position[1], 'ro', markersize=10, label='Vehicle')

    ax.set_aspect('equal', 'box')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Road Polygons Visualization')
    plt.legend()
    plt.grid(True)
    plt.show()


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

    road_polygons = load_polygons()

    if road_polygons is None:
        roads = bng.get_roads()
        road_polygons = []
        for road in roads:
            try:
                road_edges = bng.get_road_edges(road)
                road_polygon = create_road_polygon(road_edges)

                if road_polygon.is_valid:
                    road_polygons.append(road_polygon)
            except Exception as e:
                print(f"Error processing road {road}: {e}")
        save_polygons(road_polygons)

    combined_polygon = unary_union([polygon for polygon in road_polygons if polygon.is_valid])

    vehicle_position = vehicle.state['pos'][:2]
    visualize_polygons(road_polygons, vehicle_position=vehicle_position)

    try:
        while True:
            vehicle.sensors.poll()
            monitor_vehicle_position(vehicle, combined_polygon)
            bng.step(3)
    finally:
        bng.close()


if __name__ == "__main__":
    main()
