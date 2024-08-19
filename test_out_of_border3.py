from beamngpy import BeamNGpy, Scenario, Vehicle
from beamngpy.sensors import State
import time
import math
from rtree import index


def build_road_index(beamng):
    """
    Erstellt einen räumlichen Index für alle Straßen im Szenario.

    Args:
        beamng: BeamNGpy-Instanz.

    Returns:
        road_idx: R-Tree-Index für die Straßenkanten.
        road_data_dict: Dictionary, das den Index mit Straßennamen und Straßenkanten verknüpft.
    """
    roads = beamng.get_roads()
    road_idx = index.Index()
    road_data_dict = {}  # Hier speichern wir sowohl die Kanten als auch die Straßennamen

    for i, road in enumerate(roads):
        road_edges = beamng.get_road_edges(road)
        if not road_edges:
            continue

        min_x = min(edge['middle'][0] for edge in road_edges)
        max_x = max(edge['middle'][0] for edge in road_edges)
        min_y = min(edge['middle'][1] for edge in road_edges)
        max_y = max(edge['middle'][1] for edge in road_edges)

        # Speichere den Straßennamen und die Kanten im Dictionary
        road_data_dict[i] = {'name': road, 'edges': road_edges}
        road_idx.insert(i, (min_x, min_y, max_x, max_y))  # Speichere die Bounding Box in den Index

    return road_idx, road_data_dict


def get_interpolated_road_edges(vehicle_position, road_edges):
    """
    Findet die nächstgelegenen Straßenkanten durch Interpolation entlang der Straße.

    Args:
        vehicle_position (tuple): Die aktuelle Position des Fahrzeugs (x, y).
        road_edges (list): Liste der Straßenkanten mit 'left', 'middle' und 'right'.

    Returns:
        tuple: Interpolierte (left_edge, right_edge) Kanten in (x, y).
    """
    nearest_edge = None
    nearest_distance = float('inf')

    # Finde den nächsten Punkt auf der Straße
    for edge in road_edges:
        middle_point = edge['middle'][:2]
        distance = math.dist(vehicle_position, middle_point)

        if distance < nearest_distance:
            nearest_distance = distance
            nearest_edge = edge

    # Rückgabe der nächstgelegenen linken und rechten Kante an dieser interpolierten Position
    if nearest_edge:
        return nearest_edge['left'][:2], nearest_edge['right'][:2]

    return None, None


def is_vehicle_out_of_road(vehicle_position, road_edges):
    """
    Überprüft, ob das Fahrzeug außerhalb der Straßenkanten fährt, indem die Position interpoliert wird.

    Args:
        vehicle_position (tuple): Die aktuelle Position des Fahrzeugs (x, y).
        road_edges (list): Liste der Straßenkanten mit 'left', 'middle' und 'right'.

    Returns:
        bool: True, wenn das Fahrzeug außerhalb der Straße ist, False, wenn es sich innerhalb der Straßenbegrenzungen befindet.
    """
    left_edge, right_edge = get_interpolated_road_edges(vehicle_position, road_edges)

    if left_edge is None or right_edge is None:
        return False  # Keine gültigen Kanten gefunden, daher wird kein Off-Road angenommen

    # Berechnung der Distanzen zu den Kanten
    dist_to_left = math.dist(vehicle_position, left_edge)
    print("dist_to_left")
    print(dist_to_left)
    dist_to_right = math.dist(vehicle_position, right_edge)
    print("dist_to_left")
    print(dist_to_right)
    # Berechnung der Breite der Straße an der interpolierten Position
    road_width = math.dist(left_edge, right_edge)
    print("road_width")
    print(road_width)
    print("vehicle_position")
    print(vehicle_position)
    print("left_edge")
    print(left_edge)
    print("right_edge")
    print(right_edge)

    # Wenn das Fahrzeug außerhalb der Kanten liegt
    if dist_to_left > road_width or dist_to_right > road_width:
        return True

    return False


def find_nearest_road(vehicle_position, road_idx, road_data_dict, max_distance=50):
    """
    Findet den Namen der nächstgelegenen Straße basierend auf der aktuellen Fahrzeugposition.

    Args:
        vehicle_position (tuple): Die aktuelle Position des Fahrzeugs (x, y).
        road_idx (index.Index): Der räumliche Index der Straßenkanten.
        road_data_dict (dict): Dictionary, das den Index mit Straßennamen und Straßenkanten verknüpft.
        max_distance (float): Maximale Distanz, um eine Straße in Betracht zu ziehen.

    Returns:
        tuple: Der Name der nächstgelegenen Straße und die zugehörigen Straßenkanten.
    """
    nearest_road_name = None
    nearest_road_edges = None
    nearest_distance = max_distance

    # Suchen von Straßenkandidaten in der Nähe der Fahrzeugposition
    candidate_ids = list(road_idx.intersection((vehicle_position[0] - max_distance,
                                                vehicle_position[1] - max_distance,
                                                vehicle_position[0] + max_distance,
                                                vehicle_position[1] + max_distance)))

    for idx in candidate_ids:
        road_data = road_data_dict.get(idx)
        if road_data is None:
            continue

        road_edges = road_data['edges']
        road_name = road_data['name']

        for edge in road_edges:
            middle_point = edge['middle'][:2]
            distance = math.dist(vehicle_position, middle_point)
            if distance < nearest_distance:
                nearest_distance = distance
                nearest_road_name = road_name
                nearest_road_edges = road_edges

    return nearest_road_name, nearest_road_edges


def log_off_road(position):
    """
    Dokumentiert, wenn das Fahrzeug außerhalb der Straßenkanten fährt.

    Args:
        position (tuple): Die Position, an der das Fahrzeug die Straße verlassen hat.
    """
    with open('offroad.log', 'a') as log_file:
        log_file.write(f"Vehicle went off-road at {time.time()} at position: {position}\n")
    print(f"Vehicle went off-road at position {position}")


def monitor_vehicle_position(vehicle, road_edges):
    """
    Überwacht die Position des Fahrzeugs und prüft, ob es die Straße verlässt.

    Args:
        vehicle (Vehicle): Das zu überwachende Fahrzeug.
        road_edges (list): Liste der Straßenkanten mit 'left', 'middle' und 'right'.
    """
    vehicle.sensors.poll()
    curr_position = vehicle.state['pos'][:2]  # Fahrzeugposition abrufen (x, y)

    if is_vehicle_out_of_road(curr_position, road_edges):
        log_off_road(curr_position)

def main():
    bng = BeamNGpy('localhost', 64256, home='C:\\Users\\stefan\\Pictures\\BeamNG.tech.v0.31.3.0\\BeamNG.tech.v0.31.3.0',
                   user='C:\\Users\\stefan\\AppData\\Local\\BeamNG.drive')
    bng.open()

    scenario = Scenario('west_coast_usa', 'example')
    state = State()
    vehicle = Vehicle('ego_vehicle', model='etk800', licence='PYTHON')
    scenario.add_vehicle(vehicle, pos=(-905.239197, -423.966522, 101.552147), rot_quat=(0, 0, 0.3826834, 0.9238795))

    scenario.make(bng)
    bng.scenario.load(scenario)
    bng.scenario.start()
    print("szenario started")
    vehicle.sensors.poll()
    print("sensors polled")
    # Erstelle den räumlichen Index für die Straßen
    road_idx, road_data_dict = build_road_index(bng)
    print("road built")
    prev_position = vehicle.state['pos'][:2]  # Initiale Position des Fahrzeugs (kann angepasst werden)
    print("init pos")
    try:
        while True:
            #print("while")
            vehicle.sensors.poll()
            curr_position = vehicle.state['pos'][:2]

            # Automatische Ermittlung der nächstgelegenen Straße
            road_name, road_edges = find_nearest_road(curr_position, road_idx, road_data_dict)
            #print("find_nearest_road done")
            if road_name:
                monitor_vehicle_position(vehicle, road_edges)
                print("monitor_vehicle_position done")
            else:
                print("No nearby road found")

            time.sleep(0.1)  # Monitoring-Intervall
    finally:
        bng.close()


if __name__ == '__main__':
    main()