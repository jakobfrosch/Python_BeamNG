import os
import shutil
import zipfile
import logging
def remove_last_character(filepath):
    with open(filepath, 'r+') as file:
        file.seek(0, os.SEEK_END)
        file.seek(file.tell() - 1, os.SEEK_SET)
        file.truncate()
def replace_weatherfile_in_zip(zipname, filename, new_content):
    # Temporären Ordner erstellen
    temp_dir = 'temp_unzip_folder'
    os.makedirs(temp_dir, exist_ok=True)

    # Archiv entpacken
    with zipfile.ZipFile(zipname, 'r') as zip_ref:
        zip_ref.extractall(temp_dir)

    # Datei im temporären Ordner ersetzen
    filepath = os.path.join(temp_dir, filename)
    with open(filepath, 'w') as file:
        file.write(new_content)

    # Neues Zip-Archiv erstellen
    with zipfile.ZipFile(zipname, 'w') as zip_ref:
        # Dateien aus dem temporären Ordner zum neuen Archiv hinzufügen
        for root, dirs, files in os.walk(temp_dir):
            for file in files:
                zip_ref.write(os.path.join(root, file), os.path.relpath(os.path.join(root, file), temp_dir))

    # Temporären Ordner löschen
    shutil.rmtree(temp_dir)
def extend_file_in_zip(zipname, filename, additional_data, removeLastChar):
    # Temporären Ordner erstellen
    temp_dir = 'temp_unzip_folder'
    os.makedirs(temp_dir, exist_ok=True)

    # Archiv entpacken
    with zipfile.ZipFile(zipname, 'r') as zip_ref:
        zip_ref.extractall(temp_dir)

    # Datei im temporären Ordner öffnen und letztes Zeichen entfernen
    filepath = os.path.join(temp_dir, filename)
    if removeLastChar:
        remove_last_character(filepath)
    # Datei im temporären Ordner erweitern
    with open(filepath, 'a') as file:
        file.write(additional_data)

    # Neues Zip-Archiv erstellen
    with zipfile.ZipFile(zipname, 'w') as zip_ref:
        # Dateien aus dem temporären Ordner zum neuen Archiv hinzufügen
        for root, dirs, files in os.walk(temp_dir):
            for file_name in files:
                file_path = os.path.join(root, file_name)
                zip_ref.write(file_path, os.path.relpath(file_path, temp_dir))

    # Temporären Ordner löschen
    shutil.rmtree(temp_dir)
def fillWeatherInXML(weather, BEAMNG_HOME):
    zipname = BEAMNG_HOME + '\\gameengine.zip'
    cloudy = ["cloudy", "overcast", "rainy"]
    time_value=weather.time_of_day
    name = "xml_weather"
    maxRaindrops = 10000
    num_drops_value = "0"
    maxFogDensity = 20000
    fogDensity = 0
    if weather.cloud_state in cloudy:
        cloudLayerCoverage = "1"
    else:
        cloudLayerCoverage = "0"
    num_drops_value = None

    if weather is not None and hasattr(weather, 'precipitation_type') and hasattr(weather, 'precipitation_intensity'):
        if weather.precipitation_type == "rain":
            try:
                num_drops_value = str(int(float(weather.precipitation_intensity) * maxRaindrops))
            except (ValueError, TypeError):
                logging.info("Fehler bei der Berechnung von num_drops_value.")
    else:
        logging.info("Wetterdaten sind unvollständig oder weather ist None.")

    fogDensity = None

    if hasattr(weather, 'fog_visualRange') and weather.fog_visualRange:
        try:
            fogDensity = str(maxFogDensity / float(weather.fog_visualRange))
        except (ValueError, ZeroDivisionError):
            logging.info("Fehler bei der Berechnung von fogDensity")

    additional_data = f'''
"{name}": {{
    "TimeOfDay": {{
      "time": {time_value}
    }},
    "Precipitation": {{
      "numDrops": {num_drops_value}
    }},
    "CloudLayer": {{
      "coverage": {cloudLayerCoverage}
    }},
    "LevelInfo": {{
      "fogDensity": {fogDensity}
    }},
    "ScatterSky": {{
      "shadowSoftness": 1,
      "colorize": [0.427451, 0.427451, 0.427451, 1],
      "sunScale": [0.686275, 0.686275, 0.686275, 1],
      "ambientScale": [0.545098, 0.545098, 0.54902, 1],
      "fogScale": [0.756863, 0.760784, 0.760784, 1]
    }},
    "ForestWindEmitter": {{
      "strength": 1.5
    }}
  }}
}}'''
    new_content = f'''{{
        "{name}": {{
            "TimeOfDay": {{
                "time": {time_value}
            }},
            "Precipitation": {{
                "numDrops": {num_drops_value}
            }},
            "CloudLayer": {{
                "coverage": {cloudLayerCoverage}
            }},
            "LevelInfo": {{
                "fogDensity": {fogDensity}
            }},
            "ScatterSky": {{
                "shadowSoftness": 1,
                "colorize": [0.427451, 0.427451, 0.427451, 1],
                "sunScale": [0.686275, 0.686275, 0.686275, 1],
                "ambientScale": [0.545098, 0.545098, 0.54902, 1],
                "fogScale": [0.756863, 0.760784, 0.760784, 1]
            }},
            "ForestWindEmitter": {{
                "strength": 1.5
            }}
        }}
    }}'''
    replace_weatherfile_in_zip(zipname,'art\\weather\\defaults.json', new_content)

def remove_line_from_file_in_zip(zipname, filename, line_to_remove):
    # Temporären Ordner erstellen
    temp_dir = 'temp_unzip_folder'
    os.makedirs(temp_dir, exist_ok=True)

    # Archiv entpacken
    with zipfile.ZipFile(zipname, 'r') as zip_ref:
        zip_ref.extractall(temp_dir)

    # Datei im temporären Ordner öffnen und Zeile entfernen
    filepath = os.path.join(temp_dir, filename)
    with open(filepath, 'r') as file:
        lines = file.readlines()
    with open(filepath, 'w') as file:
        for line in lines:
            if line.strip() != line_to_remove.strip():
                file.write(line)

    # Neues Zip-Archiv erstellen
    with zipfile.ZipFile(zipname, 'w') as zip_ref:
        # Dateien aus dem temporären Ordner zum neuen Archiv hinzufügen
        for root, dirs, files in os.walk(temp_dir):
            for file in files:
                zip_ref.write(os.path.join(root, file), os.path.relpath(os.path.join(root, file), temp_dir))

    # Temporären Ordner löschen
    shutil.rmtree(temp_dir)


def removeWaypointInXML(line_to_remove,BEAMNG_HOME):
    zipname=BEAMNG_HOME+'\\content\\levels\\west_coast_usa.zip'
    remove_line_from_file_in_zip(zipname, 'levels/west_coast_usa/main/MissionGroup/AIWaypointsGroup/items.level.json', line_to_remove)
    print(line_to_remove + "removed")
def fillWaypointInXML(name, pos,BEAMNG_HOME):
    content = f'''\n{{"name":"{name}","class":"BeamNGWaypoint","persistentId":"ffbf30ea-c0a9-4904-a91c-86d5be267d99","__parent":"AIWaypointsGroup","position":{pos},"scale":[3.62683105,3.62683105,3.62683105]}}'''
    #extend_file_in_zip('C:\\Users\\stefan\\Downloads\\BeamNG.tech.v0.31.3.0\\BeamNG.tech.v0.31.3.0\\content\\levels\\west_coast_usa.zip','\\levels\\west_coast_usa\\main\\MissionGroup\\AIWaypointsGroup\\items.level.json', content, False)
    zipname=BEAMNG_HOME+'\\content\\levels\\west_coast_usa.zip'
    extend_file_in_zip(zipname, 'levels/west_coast_usa/main/MissionGroup/AIWaypointsGroup/items.level.json', content, False)
    #'C:\\Users\\stefan\\Downloads\\BeamNG.tech.v0.31.3.0\\BeamNG.tech.v0.31.3.0\\content\\levels\\west_coast_usa.zip', 'levels/west_coast_usa/main/MissionGroup/AIWaypointsGroup/items.level.json'
    #C:\Users\stefan\Downloads\BeamNG.tech.v0.31.3.0\BeamNG.tech.v0.31.3.0\content\levels\west_coast_usa.zip\levels\west_coast_usa\main\MissionGroup\AIWaypointsGroup
    #extend_file_in_zip('C:\\Users\\stefan\\Downloads\\BeamNG.tech.v0.31.3.0\\BeamNG.tech.v0.31.3.0\\gameengine.zip', 'art\\weather\\defaults.json', content, False)
    return content
