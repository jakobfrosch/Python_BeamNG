import xml.etree.ElementTree as ET
import Condition

class BaseCondition:
    def __init__(self, condition_type, condition_type_type, mainEntityRef, triggeringEntities, main_condition_type):
        self.condition_type = condition_type
        self.condition_type_type = condition_type_type
        self.mainEntityRef = mainEntityRef
        self.triggeringEntities = triggeringEntities
        self.main_condition_type = main_condition_type

    def check_condition(self, *args, **kwargs):
        raise NotImplementedError("Subclasses must implement check_condition method")

    def __str__(self):
        return f"Condition: {self.condition_type_type} - Main Entity Ref: {self.mainEntityRef}, Triggering Entities: {self.triggeringEntities}"


class TraveledDistanceCondition(BaseCondition):
    def __init__(self, value, condition_type_type, mainEntityRef, triggeringEntities, main_condition_type):
        super().__init__("TraveledDistance", condition_type_type, mainEntityRef, triggeringEntities, main_condition_type)
        self.value = value

    def check_condition(self, traveled_distance):
        return traveled_distance >= self.value

    def __str__(self):
        return f"{super().__str__()}, Value = {self.value}"


class SpeedAction:
    def __init__(self, dynamics_shape, value, dynamics_dimension, target_speed_value):
        self.dynamics_shape = dynamics_shape
        self.value = value
        self.dynamics_dimension = dynamics_dimension
        self.target_speed_value = target_speed_value

    def __str__(self):
        return f"SpeedActionDynamics: dynamicsShape = {self.dynamics_shape}, value= {self.value}, dynamicsDimension= {self.dynamics_dimension}\n" \
               f"SpeedActionTarget: AbsoluteTargetSpeed: Value = {self.target_speed_value}"


class RoutingAction:
    def __init__(self, route_name, waypoints):
        self.route_name = route_name
        self.waypoints = waypoints

    def __str__(self):
        return f"Route: {self.route_name}, Waypoints: {self.waypoints}"

class LongitudinalDistanceAction:
    def __init__(self, route_name, waypoints):
        self.route_name = route_name
        self.waypoints = waypoints

    def __str__(self):
        return f"Route: {self.route_name}, Waypoints: {self.waypoints}"
class Action:
    def __init__(self, name, action_type, speed_action=None, routing_action=None):
        self.name = name
        self.action_type = action_type
        self.speed_action = speed_action
        self.routing_action = routing_action

    def __str__(self):
        action_str = f"Action: Name = {self.name}\n, Action Type: {self.action_type}, "
        if self.speed_action:
            action_str += str(self.speed_action) + "\n"
        if self.routing_action:
            action_str += str(self.routing_action) + "\n"
        return action_str


class Event:
    def __init__(self, name, priority, action, condition):
        self.name = name
        self.priority = priority
        self.action = action
        self.condition = condition

    def __str__(self):
        return f"Event: Name = {self.name}, Priority = {self.priority}\n{self.action}, condition = {self.condition}"


def parse_conditions_from_xml(xml_path):
    conditions = []

    tree = ET.parse(xml_path)
    root = tree.getroot()
    condition_type_types = ['ByValueCondition', 'EntityCondition']

    for element in root.iter('Condition'):
        main_condition_type = element.attrib.get('name')

        mainEntityRef = element.find(".//EntityRef")
        if mainEntityRef is not None:
            mainEntityRef = mainEntityRef.attrib.get('entityRef')
        condition_type_type = None
        for elem in element.iter():
            if elem.tag != "Condition":
                condition_type_type = elem
                triggeringEntities = elem.find("./TriggeringEntities")
                if triggeringEntities is not None:
                    triggeringEntities = triggeringEntities.get("triggeringEntitiesRule")
                break
        condition_type2 = None
        condition_params = {}
        condition_params2 = {}

        for key, value in element.attrib.items():
            if key != 'name':
                condition_params[key] = value
        for condition_type_type in condition_type_types:
            for node in element.iter(condition_type_type):
                for node2 in node:
                    if node2.tag.endswith('Condition'):
                        condition_type2 = node2.tag
                        for key, value in node2.items():
                            if key != 'name':
                                condition_params2[key] = value

        if condition_type2 == 'TraveledDistanceCondition':
            value = float(condition_params2.get('value'))
            condition = TraveledDistanceCondition(value, condition_type_type, mainEntityRef, triggeringEntities,
                                                  main_condition_type)
            conditions.append(condition)

    return conditions


def parse_actions(element):
    speed_action = None
    routing_action = None
    action_type = None
    action_name = element.attrib.get('name')
    for action_elem in element.iter():

        if action_elem.tag == "SpeedAction":
            action_type = action_elem.tag
            try:
                dynamics_shape = action_elem.find("./SpeedActionDynamics").attrib.get("dynamicsShape")
            except AttributeError:
                dynamics_shape = None
            try:
                value = action_elem.find("./SpeedActionDynamics").attrib.get("value")
            except AttributeError:
                value = None
            try:
                dynamics_dimension = action_elem.find("./SpeedActionDynamics").attrib.get("dynamicsDimension")
            except AttributeError:
                dynamics_dimension = None
            target_speed_value = action_elem.find("./SpeedActionTarget/AbsoluteTargetSpeed").attrib.get("value")
            speed_action = SpeedAction(dynamics_shape, value, dynamics_dimension, target_speed_value)
        elif action_elem.tag == "RoutingAction":
            action_type = action_elem.tag
            route_name = action_elem.find("./AssignRouteAction/Route").attrib.get("name")
            route_type = action_elem.tag
            print(route_type)
            waypoints = []
            for waypoint_elem in action_elem.findall("./AssignRouteAction/Route/Waypoint"):
                position_elem = waypoint_elem.find("./Position/WorldPosition")
                waypoint = {
                    "routeStrategy": waypoint_elem.attrib.get("routeStrategy"),
                    "x": position_elem.attrib.get("x"),
                    "y": position_elem.attrib.get("y"),
                    "z": position_elem.attrib.get("z"),
                    "h": position_elem.attrib.get("h")
                }
                waypoints.append(waypoint)
            routing_action = RoutingAction(route_name, waypoints)

    return Action(action_name, action_type, speed_action, routing_action)


def parse_events_from_xml(xml_path):
    events = []

    tree = ET.parse(xml_path)
    root = tree.getroot()

    for event_elem in root.findall(".//Event"):
        event_name = event_elem.attrib.get('name')
        priority = event_elem.attrib.get('priority')

        action_elem = event_elem.find("./Action")
        action = parse_actions(action_elem)
        conditions = Condition.parse_conditions_from_xml(xml_path)
        condition_elem = event_elem.find(".//Condition")
        condition_name = None
        c1 = None
        if condition_elem is not None:
            condition_name = condition_elem.attrib.get('name')
        for condition in conditions:
            if condition.main_condition_type == condition_name:
                c1 = condition
        event = Event(event_name, priority, action, c1)

        events.append(event)

    return events


def main():
    xml_path = "C:\\Users\\stefan\\Downloads\\FollowLeadingVehicle2.xosc"
    xml_path4 = "C:\\Users\\stefan\\Downloads\\FollowLeadingVehicle4.xosc"
    xml_path5 = "C:\\Users\\stefan\\Downloads\\FollowLeadingVehicle5.xosc"
    xml_path2 = "C:\\Users\\stefan\\Documents\\PedestrianCrossingFront.xosc"
    events = parse_events_from_xml(xml_path4)
    conditions = Condition.parse_conditions_from_xml(xml_path5)

    print("\nExtracted Events:")
    for event in events:
        print(event)
        print()


if __name__ == "__main__":
    main()
