from shapely.geometry import Polygon

def create_road_polygon(road_edges):
    left_points = [edge['left'][:2] for edge in road_edges]
    right_points = [edge['right'][:2] for edge in reversed(road_edges)]
    road_polygon = Polygon(left_points + right_points)
    if not road_polygon.is_valid:
        print("Warning: Das Straßenpolygon ist ungültig.")
    return road_polygon

def load_polygons(file_path):
    if os.path.exists(file_path):
        with open(file_path, 'rb') as f:
            road_polygons = pickle.load(f)
        print(f"Road polygons loaded from {file_path}")
        return road_polygons
    return None

def save_polygons(road_polygons, file_path):
    with open(file_path, 'wb') as f:
        pickle.dump(road_polygons, f)
    print(f"Road polygons saved to {file_path}")

def monitor_vehicle_position(vehicle, road_polygons):
    vehicle.sensors.poll()
    curr_position = vehicle.state['pos'][:2]
    vehicle_point = Point(curr_position)
    if not any(road_polygon.contains(vehicle_point) for road_polygon in road_polygons):
        print(f"Fahrzeug {vehicle.vid} ist von der Straße abgekommen. Position: {curr_position}")

def fillWaypointInXML(waypointName, waypointString):
    return f"""
        <Waypoint name="{waypointName}" position="{waypointString}" />
    """
