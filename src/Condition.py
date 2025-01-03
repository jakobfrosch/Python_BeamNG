import xml.etree.ElementTree as ET

class BaseCondition:
    def __init__(self, condition_type, condition_type_type, mainEntityRef, triggeringEntities, main_condition_type, stop_condition):

        self.condition_type = condition_type
        self.condition_type_type = condition_type_type
        self.mainEntityRef = mainEntityRef
        self.triggeringEntities = triggeringEntities
        self.main_condition_type = main_condition_type
        self.stop_condition = stop_condition
    def check_condition(self, *args, **kwargs):
        raise NotImplementedError("Subclasses must implement check_condition method")

class TimeOfDayCondition(BaseCondition):
    def __init__(self, start_time, end_time):
        super().__init__("TimeOfDay")
        self.start_time = start_time
        self.end_time = end_time

    def check_condition(self, current_time):
        return self.start_time <= current_time <= self.end_time

    def __str__(self):
        return f"Time of Day Condition: Start Time = {self.start_time}, End Time = {self.end_time}"


#class SpeedCondition(BaseCondition):
#    def __init__(self, value, comparison_operator):
#        super().__init__("Speed")
#        self.value = value
#        self.comparison_operator = comparison_operator

#    def check_condition(self, current_speed):
#        if self.comparison_operator == ">":
#            return current_speed > self.value
#        elif self.comparison_operator == "<":
#            return current_speed < self.value
#        elif self.comparison_operator == ">=":
#            return current_speed >= self.value
#        elif self.comparison_operator == "<=":
#            return current_speed <= self.value
#        elif self.comparison_operator == "==":
#            return current_speed == self.value
#        else:
#            raise ValueError("Invalid comparison operator")

#    def __str__(self):
#        return f"Speed Condition: Value = {self.value}, Comparison Operator = {self.comparison_operator}"

class ReachPositionCondition(BaseCondition):
    def __init__(self, condition_type_type, mainEntityRef, triggeringEntities, main_condition_type, tolerance, position, stop_condition):
        super().__init__("ReachPosition", condition_type_type, mainEntityRef, triggeringEntities, main_condition_type, stop_condition)
        self.tolerance = tolerance
        self.position = position

    def check_condition(self, current_position):
        # Check if the current position is within the tolerance range of the target position
        return abs(current_position - self.position) <= self.tolerance

    def __str__(self):
        return f"Reach Position Condition: (Main Condition Type: {self.main_condition_type}, Condition Type Type: {self.condition_type_type}, Main Entity Ref: {self.mainEntityRef}, Stop Condition: {self.stop_condition}, Triggering Entities: {self.triggeringEntities}), Tolerance = {self.tolerance}, Target Position = {self.position}"

def parse_position_from_xml(position_element):
    x = None
    y = None
    z = None
    h = None
    if (position_element.get('x') is not None) :
        x = float(position_element.get('x'))
    if(position_element.get('y') is not None) :
        y = float(position_element.get('y'))
    if(position_element.get('z') is not None) :
        z = float(position_element.get('z'))
    if(position_element.get('h') is not None) :
        h = float(position_element.get('h'))
    return (x, y, z, h)
class RelativeDistanceCondition(BaseCondition):
    def __init__(self, entity_ref, relative_distance_type, value, freespace, rule, delay, condition_type_type, mainEntityRef, triggeringEntities, main_condition_type, stop_condition):
        super().__init__("RelativeDistance", condition_type_type, mainEntityRef, triggeringEntities, main_condition_type, stop_condition)
        self.entity_ref = entity_ref
        self.relative_distance_type = relative_distance_type
        self.value = value
        self.freespace = freespace
        self.rule = rule
        self.delay = delay

    def check_condition(self, current_distance, condition_type_type, mainEntityRef, triggeringEntities, main_condition_type, stop_condition):
        if self.rule == "lessThan":
            return current_distance < self.value + self.delay
        elif self.rule == "greaterThan":
            return current_distance > self.value + self.delay
        elif self.rule == "equalTo":
            return current_distance == self.value + self.delay
        else:
            raise ValueError("Invalid rule for RelativeDistanceCondition")

    def __str__(self):
        return f"Relative Distance Condition: (Main Condition Type: {self.main_condition_type} Condition Type Type: {self.condition_type_type} Main Entity Ref: {self.mainEntityRef}, Stop Condition: {self.stop_condition} Triggering Entities: {self.triggeringEntities}) Entity Ref = {self.entity_ref}, Relative Distance Type = {self.relative_distance_type}, Value = {self.value}, Freespace = {self.freespace}, Rule = {self.rule}, Delay = {self.delay}"

class TraveledDistanceCondition(BaseCondition):
    def __init__(self, value, condition_type_type, mainEntityRef, triggeringEntities, main_condition_type, stop_condition):
        super().__init__("TraveledDistance", condition_type_type, mainEntityRef, triggeringEntities, main_condition_type, stop_condition)
        self.value = value

    def check_condition(self, traveled_distance):
        return traveled_distance >= self.value

    def __str__(self):
        return f"Traveled Distance Condition: (Main Condition Type: {self.main_condition_type} Condition Type Type: {self.condition_type_type} Main Entity Ref: {self.mainEntityRef}, Stop Condition: {self.stop_condition} Triggering Entities: {self.triggeringEntities}) Value = {self.value}"

class SpeedCondition(BaseCondition):
    def __init__(self, value, rule, condition_type_type, mainEntityRef, triggeringEntities, main_condition_type, stop_condition):
        super().__init__("Speed", condition_type_type, mainEntityRef, triggeringEntities, main_condition_type, stop_condition)
        self.value = value
        self.rule = rule


    def __str__(self):
        return f"Speed Condition: (Main Condition Type: {self.main_condition_type} Condition Type Type: {self.condition_type_type} Main Entity Ref: {self.mainEntityRef}, Stop Condition: {self.stop_condition} Triggering Entities: {self.triggeringEntities}) Value = {self.value}, rule= {self.rule}"


class SimulationTimeCondition(BaseCondition):
    def __init__(self, value, rule, condition_type_type, mainEntityRef, triggeringEntities, main_condition_type, stop_condition):
        super().__init__("SimulationTime", condition_type_type, mainEntityRef, triggeringEntities, main_condition_type, stop_condition)
        self.value = value
        self.rule = rule

    def check_condition(self, current_time):
        if self.rule == "equalTo":
            return current_time == self.value
        elif self.rule == "lessThan":
            return current_time < self.value
        elif self.rule == "greaterThan":
            return current_time > self.value
        else:
            raise ValueError("Invalid rule for SimulationTimeCondition")

    def __str__(self):
        return f"Simulation Time Condition: (Main Condition Type: {self.main_condition_type} Condition Type Type: {self.condition_type_type} Main Entity Ref: {self.mainEntityRef}, Stop Condition: {self.stop_condition} Triggering Entities: {self.triggeringEntities}) Value = {self.value}, Rule = {self.rule}"
class StoryboardElementStateCondition(BaseCondition):
    def __init__(self, storyboardElementType, storyboardElementRef, state, delay, condition_type_type, mainEntityRef, triggeringEntities, main_condition_type, stop_condition):
        super().__init__("StoryboardElementState", condition_type_type, mainEntityRef, triggeringEntities,
                         main_condition_type, stop_condition)
        self.storyboardElementType = storyboardElementType
        self.storyboardElementRef = storyboardElementRef
        self.state = state
        self.delay = delay
        #print(self.storyboardElementType)
    def __str__(self):
        return f"Storyboard Element State Condition: (Main Condition Type: {self.main_condition_type}, Condition Type Type: {self.condition_type_type}, Main Entity Ref: {self.mainEntityRef}, Stop Condition: {self.stop_condition}, Triggering Entities: {self.triggeringEntities}), storyboardElementType = {self.storyboardElementType}, storyboardElementRef = {self.storyboardElementRef}, state = {self.state}"


class ParameterCondition(BaseCondition):
    def __init__(self, parameterRef, value, rule, condition_type_type, mainEntityRef, triggeringEntities, main_condition_type, stop_condition):
        super().__init__("Parameter", condition_type_type, mainEntityRef, triggeringEntities, main_condition_type, stop_condition)
        self.value = value
        self.rule = rule
        self.parameterRef = parameterRef
    def check_condition(self, current_time):
        if self.rule == "equalTo":
            return current_time == self.value
        elif self.rule == "lessThan":
            return current_time < self.value
        elif self.rule == "greaterThan":
            return current_time > self.value
        else:
            raise ValueError("Invalid rule for ParameterCondition")

    def __str__(self):
        return f"Parameter Condition: (Main Condition Type: {self.main_condition_type}, Condition Type Type: {self.condition_type_type}, Main Entity Ref: {self.mainEntityRef}, Stop Condition: {self.stop_condition}, Triggering Entities: {self.triggeringEntities}), Parameter Ref = {self.parameterRef}, Value = {self.value}, Rule = {self.rule}"
def is_in_stop_trigger(element):
    """Hilfsfunktion, um zu prüfen, ob ein Element in einem StopTrigger enthalten ist."""
    # Wir durchlaufen alle Elternknoten von 'element'
    while element is not None:
        # Wenn wir ein 'StopTrigger'-Element finden, geben wir True zurück
        if element.tag == "StopTrigger":
            return True
        # Gehe zum übergeordneten Element, indem wir die gesamte Baumstruktur durchsuchen
        element = element.getparent()  # Dies funktioniert in ElementTree nicht

    # Wenn kein 'StopTrigger' gefunden wurde, geben wir False zurück
    return False
def find_parent_with_tag(element, tag):
    """Hilfsfunktion, um den ersten übergeordneten Knoten mit einem bestimmten Tag zu finden."""
    # Suche nach einem übergeordneten Element mit dem gewünschten Tag
    for parent in element.iterancestors():
        if parent.tag == tag:
            return True
    return False
def parse_conditions_from_xml(xml_path):
    conditions = []

    tree = ET.parse(xml_path)
    root = tree.getroot()
    condition_type_types = ['ByValueCondition', 'EntityCondition']

    # Hilfsfunktion, um die Elternknoten zu finden
def find_if_in_stop_trigger(element, root):
    """Findet, ob ein Element in einem 'StopTrigger'-Knoten enthalten ist."""
    for stop_trigger in root.iter('StopTrigger'):
        if stop_trigger.find('.//Condition[@name="{}"]'.format(element.attrib['name'])) is not None:
            return True
    return False
def parse_conditions_from_xml(xml_path):
    conditions = []

    tree = ET.parse(xml_path)
    root = tree.getroot()
    condition_type_types = ['ByValueCondition', 'EntityCondition']

    for element in root.iter('Condition'):
        main_condition_type = element.attrib.get('name')
        is_stop_condition = find_if_in_stop_trigger(element, root)
        #print(is_stop_condition)
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
                        delay = 0  # Standardwert für das Delay
                        delay_element = node2.find('StartTrigger/ConditionGroup/Condition')
                        if delay_element is not None and 'delay' in delay_element.attrib:
                            delay = int(delay_element.attrib['delay'])
                        condition_params2['delay'] = delay

        if condition_type2 == 'TimeOfDayCondition':
            start_time = float(condition_params.get('startTime'))
            end_time = float(condition_params.get('endTime'))
            condition = TimeOfDayCondition(start_time, end_time)
        #elif condition_type2 == 'SpeedCondition':
        #    print(condition_params)
        #    value = int(condition_params.get('value'))
        #    comparison_operator = condition_params.get('rule')
        #    condition = SpeedCondition(value, comparison_operator)
        elif condition_type2 == 'RelativeDistanceCondition':
            entity_ref = condition_params2.get('entityRef')
            relative_distance_type = condition_params2.get('relativeDistanceType')
            value = float(condition_params2.get('value'))
            freespace = condition_params2.get('freespace') == 'true'
            rule = condition_params2.get('rule')
            delay = int(condition_params2.get('delay', '0'))
            condition = RelativeDistanceCondition(entity_ref, relative_distance_type, value, freespace, rule, delay,
                                                  condition_type_type, mainEntityRef, triggeringEntities,
                                                  main_condition_type, is_stop_condition)
        elif condition_type2 == 'TraveledDistanceCondition':
            value = float(condition_params2.get('value'))
            condition = TraveledDistanceCondition(value, condition_type_type, mainEntityRef, triggeringEntities,
                                                  main_condition_type, is_stop_condition)
        elif condition_type2 == 'SpeedCondition':
            value = float(condition_params2.get('value'))
            rule = str(condition_params2.get('rule'))
            condition = SpeedCondition(value, rule, condition_type_type, mainEntityRef, triggeringEntities,
                                                  main_condition_type, is_stop_condition)
        elif condition_type2 == 'ReachPositionCondition':
            position_element = node2.find('.//Position/WorldPosition')
            position = parse_position_from_xml(position_element)
            tolerance = float(condition_params2.get('tolerance'))
            condition = ReachPositionCondition(condition_type_type, mainEntityRef, triggeringEntities,
                                                main_condition_type,tolerance, position, is_stop_condition)
        elif condition_type2 == 'SimulationTimeCondition':
            value = float(condition_params2.get('value'))
            rule = condition_params2.get('rule')
            condition = SimulationTimeCondition(value, rule, condition_type_type, mainEntityRef, triggeringEntities,
                                                main_condition_type, is_stop_condition)
        elif condition_type2 == 'ParameterCondition':
            parameterRef = condition_params2.get('parameterRef')
            value = condition_params2.get('value')
            rule = condition_params2.get('rule')
            condition = ParameterCondition(parameterRef, value, rule, condition_type_type, mainEntityRef,
                                           triggeringEntities, main_condition_type, is_stop_condition)
        elif condition_type2 == 'StoryboardElementStateCondition':
            storyboardElementType = condition_params2.get('storyboardElementType')
            storyboardElementRef = condition_params2.get('storyboardElementRef')
            state = condition_params2.get('state')
            delay = int(condition_params2.get('delay', '0'))
            condition = StoryboardElementStateCondition(storyboardElementType, storyboardElementRef, state, delay,
                                                        condition_type_type, mainEntityRef, triggeringEntities,
                                                        main_condition_type, is_stop_condition)
        else:
           # print(condition_type2)
            continue

        conditions.append(condition)

    return conditions



def main():
    xml_path = "C:\\Users\\stefan\\Downloads\\FollowLeadingVehicle2.xosc"
    xml_path4 = "C:\\Users\\stefan\\Documents\\FollowLeadingVehicle4_time.xosc"
    xml_path5 = "C:\\Users\\stefan\\Downloads\\FollowLeadingVehicle5.xosc"
    xml_path5_time = "C:\\Users\\stefan\\Documents\\FollowLeadingVehicle5_time.xosc"
    xml_path2 = "C:\\Users\\stefan\\Documents\\PedestrianCrossingFront.xosc"
    conditions = parse_conditions_from_xml(xml_path5_time)

    # Drucke die extrahierten Conditions
    print("Extracted Conditions:")
    for condition in conditions:
        print(condition)


if __name__ == "__main__":
    main()