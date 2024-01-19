import xml.etree.ElementTree as ET
from datetime import datetime
class Weather:
    def __init__(self, cloud_state=None, sun_azimuth=None, sun_elevation=None, sun_intensity=None, time_of_day=None):
        self.cloud_state = cloud_state
        self.sun_azimuth = sun_azimuth
        self.sun_elevation = sun_elevation
        self.sun_intensity = sun_intensity
        self.time_of_day = time_of_day

    def fillWeather(self, root):
        environment_element = root.find(".//Environment")
        if environment_element is not None:
            weather_element = environment_element.find("Weather")
            if weather_element is not None:
                self.cloud_state = weather_element.get("cloudState")
                sun_element = weather_element.find("Sun")
                if sun_element is not None:
                    self.sun_azimuth = sun_element.get("azimuth")
                    self.sun_elevation = sun_element.get("elevation")
                    self.sun_intensity = sun_element.get("intensity")

                time_of_day_element = environment_element.find("TimeOfDay")
                if time_of_day_element is not None:
                    # Datum in ein datetime-Objekt konvertieren
                    date_object = datetime.strptime(time_of_day_element.get("dateTime"), '%Y-%m-%dT%H:%M:%S')

                    # Nur die Uhrzeit extrahieren
                    self.time_of_day = date_object.strftime('%H:%M:%S')


    def saveWeather(self):
        # Hier kannst du die Logik implementieren, um die Wetterdaten zu speichern
        pass

# Pfad zur XML-Datei
xml_file_path = "C:\\Users\\Stefan\\PedestrianCrossingFront.xosc"

# XML-Datei einlesen
tree = ET.parse(xml_file_path)
root = tree.getroot()

# Wetterinformationen extrahieren und in der Weather-Klasse speichern
weather = Weather()
weather.fillWeather(root)

# Ausgabe der extrahierten Informationen
print(f"Weather_cloudState: {weather.cloud_state}")
print(f"Sun_azimuth: {weather.sun_azimuth}")
print(f"Sun_elevation: {weather.sun_elevation}")
print(f"Sun_intensity: {weather.sun_intensity}")
print(f"TimeOfDay_dateTime: {weather.time_of_day}")

# Beispiel für die Verwendung der saveWeather-Methode
weather.saveWeather()
