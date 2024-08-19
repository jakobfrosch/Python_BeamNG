from beamngpy import BeamNGpy, Scenario, Vehicle
from beamngpy.sensors import State
import time
import math
from operator import length_hint



def vector_cross_product(v1, v2):
    """ Berechnet das Kreuzprodukt von zwei Vektoren. """
    return v1[0] * v2[1] - v1[1] * v2[0]


def is_point_on_one_side_of_line(p, line):
    """ Überprüft, ob ein Punkt auf einer bestimmten Seite einer Linie liegt. """
    line_start, line_end = line
    vec_line = (line_end[0] - line_start[0], line_end[1] - line_start[1])
    vec_point = (p[0] - line_start[0], p[1] - line_start[1])
    return vector_cross_product(vec_line, vec_point)


def is_vehicle_over_line(prev_pos, curr_pos, line):
    """
    Überprüft, ob das Fahrzeug eine Linie überschritten hat, indem es die Position des Fahrzeugs
    im vorherigen und aktuellen Frame vergleicht.
    """
    prev_side = is_point_on_one_side_of_line(prev_pos, line)
    curr_side = is_point_on_one_side_of_line(curr_pos, line)
    return prev_side * curr_side < 0  # Wenn das Vorzeichen sich ändert, hat das Fahrzeug die Linie überquert


def monitor_vehicle_position(vehicle, boundary_lines, prev_position):
    curr_position = vehicle.state['pos'][:2]  # Fahrzeugposition abrufen (x, y)

    for line in boundary_lines:
        if is_vehicle_over_line(prev_position, curr_position, line):
            log_crossing(curr_position, line)

    return curr_position  # Die aktuelle Position für die nächste Iteration zurückgeben


def log_crossing(position, line):
    """ Dokumentiert das Überschreiten einer Linie in einer Logdatei. """
    with open('crossings.log', 'a') as log_file:
        log_file.write(f"Crossed line {line} at {time.time()} at position: {position}\n")
    print(f"Line {line} crossed at position {position}")


def get_boundary_lines_from_road_edges(road_edges):
    """ Wandelt die Straßenkanten in Liniensegmente um, die zur Überwachung verwendet werden. """
    boundary_lines = []

    for i in range(len(road_edges) - 1):
        left_edge_start = road_edges[i]['left'][:2]
        left_edge_end = road_edges[i + 1]['left'][:2]
        right_edge_start = road_edges[i]['right'][:2]
        right_edge_end = road_edges[i + 1]['right'][:2]

        boundary_lines.append((left_edge_start, left_edge_end))
        boundary_lines.append((right_edge_start, right_edge_end))

    return boundary_lines


def build_road_index(beamng):
    """Erstellt einen räumlichen Index für alle Straßen im Szenario."""
    roads = beamng.get_roads()
    road_idx = index.Index()
    road_edges_dict = {}

    for i, road in enumerate(roads):
        road_edges = beamng.get_road_edges(road)
        if not road_edges:
            continue

        min_x = min(edge['middle'][0] for edge in road_edges)
        max_x = max(edge['middle'][0] for edge in road_edges)
        min_y = min(edge['middle'][1] for edge in road_edges)
        max_y = max(edge['middle'][1] for edge in road_edges)

        road_idx.insert(i, (min_x, min_y, max_x, max_y))
        road_edges_dict[i] = road_edges

    return road_idx, road_edges_dict, roads


def find_nearest_road(vehicle_position, road_idx, road_edges_dict, roads, max_distance=50):
    """Findet den Namen der nächstgelegenen Straße basierend auf der aktuellen Fahrzeugposition."""
    nearest_road = None
    nearest_distance = max_distance
    nearest_edge_id = None

    # Suchen von Straßenkandidaten in der Nähe der Fahrzeugposition
    candidate_ids = list(road_idx.intersection((vehicle_position[0] - max_distance,
                                                vehicle_position[1] - max_distance,
                                                vehicle_position[0] + max_distance,
                                                vehicle_position[1] + max_distance)))

    for idx in candidate_ids:
        road_edges = road_edges_dict[idx]
        for edge in road_edges:
            middle_point = edge['middle'][:2]
            distance = math.dist(vehicle_position, middle_point)
            if distance < nearest_distance:
                nearest_distance = distance
                nearest_road = roads[idx]
                nearest_edge_id = idx

    return nearest_road


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
    vehicle.sensors.poll()

    # Erstelle den räumlichen Index für die Straßen
    road_idx, road_edges_dict, roads = build_road_index(bng)
    prev_position = vehicle.state['pos'][:2]  # Initiale Position des Fahrzeugs (kann angepasst werden)

    try:
        while True:
            vehicle.sensors.poll()
            curr_position = vehicle.state['pos'][:2]

            # Automatische Ermittlung der nächstgelegenen Straße
            road_name = find_nearest_road(curr_position, road_idx, road_edges_dict, roads)
            if road_name:
                road_edges = road_edges_dict[roads.index(road_name)]
                boundary_lines = get_boundary_lines_from_road_edges(road_edges)
                prev_position = monitor_vehicle_position(vehicle, boundary_lines, prev_position)
            else:
                print("No nearby road found")

            time.sleep(0.1)  # Monitoring-Intervall
    finally:
        bng.close()


if __name__ == '__main__':
    main()