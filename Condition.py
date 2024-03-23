import xml.etree.ElementTree as ET

class BaseCondition:
 #   def __init__(self, condition_type):
    def __init__(self, condition_type, condition_type_type, mainEntityRef, triggeringEntities, main_condition_type):

        self.condition_type = condition_type
        self.condition_type_type = condition_type_type
        self.mainEntityRef = mainEntityRef
        self.triggeringEntities = triggeringEntities
        self.main_condition_type = main_condition_type
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


class SpeedCondition(BaseCondition):
    def __init__(self, value, comparison_operator):
        super().__init__("Speed")
        self.value = value
        self.comparison_operator = comparison_operator

    def check_condition(self, current_speed):
        if self.comparison_operator == ">":
            return current_speed > self.value
        elif self.comparison_operator == "<":
            return current_speed < self.value
        elif self.comparison_operator == ">=":
            return current_speed >= self.value
        elif self.comparison_operator == "<=":
            return current_speed <= self.value
        elif self.comparison_operator == "==":
            return current_speed == self.value
        else:
            raise ValueError("Invalid comparison operator")

    def __str__(self):
        return f"Speed Condition: Value = {self.value}, Comparison Operator = {self.comparison_operator}"


class RelativeDistanceCondition(BaseCondition):
    def __init__(self, entity_ref, relative_distance_type, value, freespace, rule, condition_type_type, mainEntityRef, triggeringEntities, main_condition_type):
        super().__init__("RelativeDistance", condition_type_type, mainEntityRef, triggeringEntities, main_condition_type)
        self.entity_ref = entity_ref
        self.relative_distance_type = relative_distance_type
        self.value = value
        self.freespace = freespace
        self.rule = rule

    def check_condition(self, current_distance, condition_type_type, mainEntityRef, triggeringEntities, main_condition_type):
        if self.rule == "lessThan":
            return current_distance < self.value
        elif self.rule == "greaterThan":
            return current_distance > self.value
        elif self.rule == "equalTo":
            return current_distance == self.value
        else:
            raise ValueError("Invalid rule for RelativeDistanceCondition")

    def __str__(self):
        return f"Relative Distance Condition: (Main Condition Type: {self.main_condition_type} Condition Type Type: {self.condition_type_type} Main Entity Ref: {self.mainEntityRef} Triggering Entities: {self.triggeringEntities}) Entity Ref = {self.entity_ref}, Relative Distance Type = {self.relative_distance_type}, Value = {self.value}, Freespace = {self.freespace}, Rule = {self.rule}"


class TraveledDistanceCondition(BaseCondition):
    def __init__(self, value, condition_type_type, mainEntityRef, triggeringEntities, main_condition_type):
        super().__init__("TraveledDistance", condition_type_type, mainEntityRef, triggeringEntities, main_condition_type)
        self.value = value

    def check_condition(self, traveled_distance):
        return traveled_distance >= self.value

    def __str__(self):
        return f"Traveled Distance Condition: (Main Condition Type: {self.main_condition_type} Condition Type Type: {self.condition_type_type} Main Entity Ref: {self.mainEntityRef} Triggering Entities: {self.triggeringEntities}) Value = {self.value}"


class SimulationTimeCondition(BaseCondition):
    def __init__(self, value, rule, condition_type_type, mainEntityRef, triggeringEntities, main_condition_type):
        super().__init__("SimulationTime", condition_type_type, mainEntityRef, triggeringEntities, main_condition_type)
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
        return f"Simulation Time Condition: (Main Condition Type: {self.main_condition_type} Condition Type Type: {self.condition_type_type} Main Entity Ref: {self.mainEntityRef} Triggering Entities: {self.triggeringEntities}) Value = {self.value}, Rule = {self.rule}"

class ParameterCondition(BaseCondition):
    def __init__(self, parameterRef, value, rule, condition_type_type, mainEntityRef, triggeringEntities, main_condition_type):
        super().__init__("Parameter", condition_type_type, mainEntityRef, triggeringEntities, main_condition_type)
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
        return f"Parameter Condition: (Main Condition Type: {self.main_condition_type}, Condition Type Type: {self.condition_type_type}, Main Entity Ref: {self.mainEntityRef}, Triggering Entities: {self.triggeringEntities}), Parameter Ref = {self.parameterRef}, Value = {self.value}, Rule = {self.rule}"

# Beispiel-Nutzung
#time_condition = TimeOfDayCondition(8, 18)
#speed_condition = SpeedCondition(50, ">")
#relative_distance_condition = RelativeDistanceCondition(entity_ref="adversary", relative_distance_type="longitudinal", value=40.0, freespace=True, rule="lessThan")
#traveled_distance_condition = TraveledDistanceCondition(value=200.0)
#simulation_time_condition = SimulationTimeCondition(value=0, rule="equalTo")

# Überprüfen der Bedingungen
# (Hier müssten Sie die entsprechenden aktuellen Werte einfügen)
#current_time = 12
#current_speed = 60
#current_distance = 30.0
#traveled_distance = 150.0

#if time_condition.check_condition(current_time):
#    print("Time of day condition met")
#else:
#    print("Time of day condition not met")

#if speed_condition.check_condition(current_speed):
#    print("Speed condition met")
#else:
#    print("Speed condition not met")

#if relative_distance_condition.check_condition(current_distance):
#    print("Relative distance condition met")
#else:
#    print("Relative distance condition not met")

#if traveled_distance_condition.check_condition(traveled_distance):
#    print("Traveled distance condition met")
#else:
#    print("Traveled distance condition not met")

# Hier würden Sie auch die SimulationTimeCondition überprüfen,
# aber da sie auf die Simulationszeit bezogen ist, benötigen Sie Zugriff auf die aktuelle Simulationszeit.
# Das gleiche gilt für die StopTrigger-Bedingungen, die Parameterbedingungen erfordern,
# die von externen Parametern abhängen, die in der Simulation aktualisiert werden müssen.


def parse_conditions_from_xml(xml_path):
    conditions = []

    tree = ET.parse(xml_path)
    root = tree.getroot()
    condition_type_types = ['ByValueCondition','EntityCondition']

    # Iteriere über alle Elemente im XML, um Conditions zu finden
    for element in root.iter('Condition'):
        main_condition_type = element.attrib.get('name')

        mainEntityRef=element.find(".//EntityRef")
        if mainEntityRef is not None:
            mainEntityRef=mainEntityRef.attrib.get('entityRef')
        condition_type_type = None
        for elem in element.iter():
            if elem.tag != "Condition":
                condition_type_type = elem
                triggeringEntities=elem.find("./TriggeringEntities")
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
                        condition_type2=node2.tag
                        for key, value in node2.items():
                            if key != 'name':
                                condition_params2[key] = value



        # Erstelle die entsprechende Condition-Instanz basierend auf dem Typ
        if condition_type2 == 'TimeOfDayCondition':
            start_time = float(condition_params.get('startTime'))
            end_time = float(condition_params.get('endTime'))
            condition = TimeOfDayCondition(start_time, end_time)
        elif condition_type2 == 'SpeedCondition':
            value = float(condition_params.get('value'))
            comparison_operator = condition_params.get('rule')
            condition = SpeedCondition(value, comparison_operator)
        elif condition_type2 == 'RelativeDistanceCondition':
            entity_ref = condition_params2.get('entityRef')
            relative_distance_type = condition_params2.get('relativeDistanceType')
            value = float(condition_params2.get('value'))
            freespace = condition_params2.get('freespace') == 'true'
            rule = condition_params2.get('rule')
            condition = RelativeDistanceCondition(entity_ref, relative_distance_type, value, freespace, rule, condition_type_type, mainEntityRef, triggeringEntities, main_condition_type)

        elif condition_type2 == 'TraveledDistanceCondition':
            value = float(condition_params2.get('value'))
            condition = TraveledDistanceCondition(value, condition_type_type, mainEntityRef, triggeringEntities, main_condition_type)
        elif condition_type2 == 'SimulationTimeCondition':
            value = float(condition_params2.get('value'))
            rule = condition_params2.get('rule')
            condition = SimulationTimeCondition(value, rule, condition_type_type, mainEntityRef, triggeringEntities, main_condition_type)
        elif condition_type2 == 'ParameterCondition':
            parameterRef = condition_params2.get('parameterRef')
            value = condition_params2.get('value')
            rule = condition_params2.get('rule')
            condition = ParameterCondition(parameterRef, value, rule, condition_type_type, mainEntityRef, triggeringEntities, main_condition_type)
        else:
            # Wenn der Condition-Typ nicht erkannt wird, überspringe ihn
            print(condition_type2)
            continue

        conditions.append(condition)

    return conditions


def main():
    xml_path = "C:\\Users\\stefan\\Downloads\\FollowLeadingVehicle.xosc"  # Pfad zur XML-Datei
    conditions = parse_conditions_from_xml(xml_path)

    # Drucke die extrahierten Conditions
    print("Extracted Conditions:")
    for condition in conditions:
        print(condition)


if __name__ == "__main__":
    main()