import math
import time
import pickle
import os
from shapely.geometry import Polygon, Point
from beamngpy import BeamNGpy, Scenario, Vehicle
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPolygon
from beamngpy.sensors import State

POLYGON_SAVE_PATH = "../road_polygons_old.pkl"  # Dateipfad zum Speichern der Polygone


# Funktion zum Erstellen eines Straßenpolygons aus den Straßenkanten
def create_road_polygon(road_edges):
    left_points = [edge['left'][:2] for edge in road_edges]
    right_points = [edge['right'][:2] for edge in reversed(road_edges)]
    road_polygon = Polygon(left_points + right_points)

    if not road_polygon.is_valid:
        print("Warning: Das Straßenpolygon ist ungültig.")

    return road_polygon


# Funktion zur Überprüfung, ob das Fahrzeug auf einer der Straßen ist
def is_vehicle_on_any_road(vehicle_position, road_polygons):
    vehicle_point = Point(vehicle_position)
    for road_polygon in road_polygons:
        if road_polygon.contains(vehicle_point):
            return True
    return False


# Funktion zur Überwachung der Fahrzeugposition
def monitor_vehicle_position(vehicle, road_polygons):
    vehicle.sensors.poll()
    curr_position = vehicle.state['pos'][:2]  # Fahrzeugposition abrufen (x, y)
    if not is_vehicle_on_any_road(curr_position, road_polygons):
        log_off_road(curr_position)
    else:
        print("Vehicle is on the road.")
        print(vehicle.state['pos'])


# Funktion zum Loggen, wenn das Fahrzeug die Straße verlässt
def log_off_road(position):
    print(f"Vehicle went off-road at position: {position}")


# Funktion zum Speichern der Polygone
def save_polygons(road_polygons):
    with open(POLYGON_SAVE_PATH, 'wb') as f:
        pickle.dump(road_polygons, f)
    print(f"Road polygons saved to {POLYGON_SAVE_PATH}")


# Funktion zum Laden der Polygone
def load_polygons():
    if os.path.exists(POLYGON_SAVE_PATH):
        with open(POLYGON_SAVE_PATH, 'rb') as f:
            road_polygons = pickle.load(f)
        print(f"Road polygons loaded from {POLYGON_SAVE_PATH}")
        return road_polygons
    return None


# Funktion zur Initialisierung der Visualisierung
def init_visualization(road_polygons):
    fig, ax = plt.subplots()

    # Zeichne jedes Straßenpolygon
    for polygon in road_polygons:
        if not polygon.is_empty and polygon.is_valid:  # Überprüfe, ob das Polygon gültig und nicht leer ist
            try:
                mpl_poly = MplPolygon(list(polygon.exterior.coords), closed=True, edgecolor='blue', facecolor='none')
                ax.add_patch(mpl_poly)
            except Exception as e:
                print(f"Error plotting polygon: {e}")

    ax.set_aspect('equal', 'box')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Live Vehicle Position on Road Polygons')
    plt.grid(True)

    # Initialisiere den Fahrzeugmarker als roten Punkt
    vehicle_marker, = ax.plot([], [], 'ro', markersize=10, label='Vehicle')

    plt.legend()
    plt.ion()  # Interaktive Mode für Live-Updates aktivieren
    plt.show()

    return fig, ax, vehicle_marker


# Funktion zur Aktualisierung der Fahrzeugposition in der Visualisierung
def update_vehicle_position(vehicle_marker, vehicle_position):
    vehicle_marker.set_data(vehicle_position[0], vehicle_position[1])
    plt.draw()
    plt.pause(0.001)  # Pause für Rendering



# Hauptfunktion, um die Logik zu starten
def main():
    bng = BeamNGpy('localhost', 64256, home='C:\\Users\\stefan\\Pictures\\BeamNG.tech.v0.31.3.0\\BeamNG.tech.v0.31.3.0',
                   user='C:\\Users\\stefan\\AppData\\Local\\BeamNG.drive')
    bng.open()

    scenario = Scenario('west_coast_usa', 'example')
    state = State()
    vehicle = Vehicle('ego_vehicle', model='etk800', licence='PYTHON')
    scenario.add_vehicle(vehicle, pos=(-606.5587361711077, -758.9599711584742, 134.69950140020228), rot_quat=(0,0,0.9238795,-0.3826834))

    scenario.make(bng)
    bng.scenario.load(scenario)
    bng.scenario.start()
    vehicle.sensors.poll()

    # Warte einen Moment, bis das Fahrzeug geladen ist
    vehicle.ai_set_mode('span')

    # Versuche, die Straßenpolygone aus der Datei zu laden
    road_polygons = load_polygons()

    # Wenn keine gespeicherten Polygone vorhanden sind, erstelle sie neu
    if road_polygons is None:
        roads = bng.get_roads()
        road_polygons = []
        for road in roads:
            try:
                road_edges = bng.get_road_edges(road)
                road_polygon = create_road_polygon(road_edges)
                road_polygons.append(road_polygon)
            except Exception as e:
                print(f"Error processing road {road}: {e}")

        # Speichere die erstellten Polygone für den nächsten Start
        save_polygons(road_polygons)

    # Initialisiere die Visualisierung
    fig, ax, vehicle_marker = init_visualization(road_polygons)

    # Hauptüberwachungs-Loop
    try:
        while True:
            # Fahrzeugposition abrufen und aktualisieren
            vehicle.sensors.poll()
            vehicle_position = vehicle.state['pos'][:2]
            update_vehicle_position(vehicle_marker, vehicle_position)

            # Überwache die Fahrzeugposition und logge, wenn es die Straße verlässt
            monitor_vehicle_position(vehicle, road_polygons)

            bng.step(3)  # Simulation fortsetzen
    finally:
        bng.close()


if __name__ == "__main__":
    main()
