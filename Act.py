import xml.etree.ElementTree as ET
from Event import Event, SpeedAction, Action  # Importiere die Event-Klasse
import Condition  # Importiere die Condition-Klassen


class Act:
    def __init__(self, name, maneuver_group_name, maximum_execution_count, actors, events):
        self.name = name
        self.maneuver_group_name = maneuver_group_name
        self.maximum_execution_count = maximum_execution_count
        self.actors = actors
        self.events = events  # Neue Eigenschaft für Events hinzugefügt

    @staticmethod
    def from_xml(element, xml_path):
        name = element.get('name')
        maneuver_group_name = element.find('ManeuverGroup').get('name')
        maximum_execution_count = int(element.find('ManeuverGroup').get('maximumExecutionCount'))
        actors = [actor.get('entityRef') for actor in element.find('.//Actors')]

        # Parse Events
        events = []
        for event_elem in element.findall('.//Event'):
            event_name = event_elem.get('name')
            priority = event_elem.get('priority')

            # Parse Action
            action_elem = event_elem.find('Action')
            action = None
            if action_elem is not None:
                action_name = action_elem.get('name')
                action = parse_actions(action_elem)

            # Parse Condition
            condition_elem = event_elem.find('StartTrigger/ConditionGroup/Condition')
            condition = None
            if condition_elem is not None:
                condition_name = condition_elem.get('name')
                # Finde die passende Bedingung für das Event
                for cond in Condition.parse_conditions_from_xml(xml_path):
                    if cond.main_condition_type == condition_name:
                        condition = cond
                        break

            event = Event(event_name, priority, action, condition)
            events.append(event)

        return Act(name, maneuver_group_name, maximum_execution_count, actors, events)


def parse_actions(element):
    speed_action = None
    action_name = element.attrib.get('name')
    action_type = None
    for action_elem in element.iter():
        if action_elem.tag == "SpeedAction":
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
            action_type = action_elem.tag
            break
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
            #routing_action = RoutingAction(route_name, waypoints)
    return Action(action_name, action_type, speed_action)


def parse_acts_from_xml(xml_path):
    acts = []
    tree = ET.parse(xml_path)
    root = tree.getroot()
    for act_elem in root.findall(".//Act"):
        act = Act.from_xml(act_elem, xml_path)
        acts.append(act)
    return acts


if __name__ == "__main__":
    xml_path = "C:\\Users\\stefan\\Downloads\\FollowLeadingVehicle4.xosc"
    xml_path2 = "C:\\Users\\stefan\\Documents\\PedestrianCrossingFront.xosc"
    acts = parse_acts_from_xml(xml_path)
    for act in acts:
        print(f"Name: {act.name}")
        print(f"Maneuver Group Name: {act.maneuver_group_name}")
        print(f"Maximum Execution Count: {act.maximum_execution_count}")
        print(f"Actors: {act.actors}")
        for event in act.events:
            print(f"Events: {event.action}")
        print()
