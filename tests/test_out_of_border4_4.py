import math
import pickle
import os
from shapely.geometry import Polygon, Point
from shapely.ops import unary_union
from beamngpy import BeamNGpy, Scenario, Vehicle
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPolygon
from beamngpy.sensors import State

POLYGON_SAVE_PATH = "../road_polygons_neuer.pkl"

# Fahrzeugabmessungen (zum Beispiel)
VEHICLE_LENGTH = 4.69  # in Meter
VEHICLE_WIDTH = 1.84  # in Meter
counter = 0

# Funktion zum Erstellen eines Straßenpolygons aus den Straßenkanten
def create_road_polygon(road_edges):
    left_points = [edge['left'][:2] for edge in road_edges]
    right_points = [edge['right'][:2] for edge in reversed(road_edges)]
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
    #print(f"Direction: {direction}")
    #print(f"vehicle_direction: {vehicle_direction}")
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


# Visualisierung des kombinierten Straßenpolygons (statisches Bild)
def visualize_combined_polygon(combined_polygon):
    fig, ax = plt.subplots()

    if not combined_polygon.is_empty and combined_polygon.is_valid:
        if combined_polygon.geom_type == 'Polygon':
            # Zeichne das Polygon
            mpl_poly = MplPolygon(list(combined_polygon.exterior.coords), closed=True, edgecolor='blue',
                                  facecolor='none')
            ax.add_patch(mpl_poly)
        elif combined_polygon.geom_type == 'MultiPolygon':
            # Zeichne jedes Polygon im MultiPolygon
            for poly in combined_polygon:
                mpl_poly = MplPolygon(list(poly.exterior.coords), closed=True, edgecolor='blue', facecolor='none')
                ax.add_patch(mpl_poly)
        else:
            print("Unexpected geometry type.")

    ax.set_aspect('equal', 'box')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Combined Road Polygons')
    plt.grid(True)

    # Zoom auf das Straßenpolygon
    minx, miny, maxx, maxy = combined_polygon.bounds
    ax.set_xlim([minx - 10, maxx + 10])
    ax.set_ylim([miny - 10, maxy + 10])

    plt.show()


# Funktion zur Visualisierung der Straßenpolygone
def visualize_polygons(road_polygons, vehicle_position=None):
    """
    Visualisiert die Straßenpolygone mit matplotlib.
    Optional kann die Fahrzeugposition ebenfalls visualisiert werden.

    Args:
        road_polygons (list): Liste von shapely Polygonen.
        vehicle_position (tuple): Optional. Die aktuelle Position des Fahrzeugs als (x, y).
    """
    fig, ax = plt.subplots()

    # Zeichne jedes Straßenpolygon
    for polygon in road_polygons:
        if not polygon.is_empty and polygon.is_valid:  # Überprüfe, ob das Polygon gültig und nicht leer ist
            try:
                mpl_poly = MplPolygon(list(polygon.exterior.coords), closed=True, edgecolor='blue', facecolor='none')
                ax.add_patch(mpl_poly)
            except Exception as e:
                print(f"Error plotting polygon: {e}")
        #break
    # Optional: Fahrzeugposition zeichnen
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
        road_spec = {}
        for r_id, r_inf in roads.items():
            if r_inf['drivability'] != '-1':

                print('drivability')
                road_edges = bng.scenario.get_road_edges(r_id)
                road_polygon = create_road_polygon(road_edges)
                if road_polygon.is_valid:
                    road_polygons.append(road_polygon)
        save_polygons(road_polygons)

    # Polygone zu einem einzigen Polygon oder MultiPolygon zusammenführen
    combined_polygon = unary_union([polygon for polygon in road_polygons if polygon.is_valid])

    # Visualisierung des kombinierten Polygons
    visualize_combined_polygon(combined_polygon)
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
