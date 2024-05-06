import xml.etree.ElementTree as ET
from datetime import datetime


def time_to_decimal(time_str):
    hours, minutes, seconds = map(int, time_str.split(':'))
    total_seconds = hours * 3600 + minutes * 60 + seconds
    seconds_since_midnight = total_seconds % (24 * 3600)
    decimal_time = seconds_since_midnight / (24 * 3600)
    decimal_time = 1 - (2 * abs(decimal_time - 0.5))
    return decimal_time
class Weather:
    def __init__(self, cloud_state=None, sun_azimuth=None, sun_elevation=None, sun_intensity=None, time_of_day=None, precipitation_type=None, precipitation_intensity=None, fog_visualRange=None):
        self.fog_visualRange = fog_visualRange
        self.cloud_state = cloud_state
        self.sun_azimuth = sun_azimuth
        self.sun_elevation = sun_elevation
        self.sun_intensity = sun_intensity
        self.time_of_day = time_of_day
        self.precipitation_type = precipitation_type
        self.precipitation_intensity = precipitation_intensity


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
                precipitation_element = weather_element.find("Precipitation")
                if precipitation_element is not None:
                    self.precipitation_type = precipitation_element.get("precipitationType")
                    self.precipitation_intensity = precipitation_element.get("intensity")
                fog_element = weather_element.find("Fog")
                if fog_element is not None:
                    self.fog_visualRange = fog_element.get("visualRange")
                time_of_day_element = environment_element.find("TimeOfDay")
                if time_of_day_element is not None:
                    # Datum in ein datetime-Objekt konvertieren
                    date_object = datetime.strptime(time_of_day_element.get("dateTime"), '%Y-%m-%dT%H:%M:%S')
                    date_object = date_object.strftime('%H:%M:%S')
                    print("tod")
                    print(date_object)
                    #self.time_of_day = date_object.strftime('%H:%M:%S')
                    date_object = time_to_decimal(date_object)
                    print(date_object)

                    # Nur die Uhrzeit extrahieren



def extract_weather_info(xml_file_path):
    tree = ET.parse(xml_file_path)
    root = tree.getroot()
    weather = Weather()
    weather.fillWeather(root)
    return weather

def main():
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
    print(f"Precipitation_type {weather.precipitation_type}")
    print(f"Precipitation_intensity {weather.precipitation_intensity}")
    print(f"Fog_visualRange {weather.fog_visualRange}")

if __name__ == "__main__":
    main()