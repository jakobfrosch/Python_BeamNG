import os
import shutil
import zipfile
from Weather import Weather
def remove_last_character(filepath):
    with open(filepath, 'r+') as file:
        file.seek(0, os.SEEK_END)
        file.seek(file.tell() - 1, os.SEEK_SET)
        file.truncate()
def replace_file_in_zip(zipname, filename, new_content):
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
def extend_file_in_zip(zipname, filename, additional_data):
    # Temporären Ordner erstellen
    temp_dir = 'temp_unzip_folder'
    os.makedirs(temp_dir, exist_ok=True)

    # Archiv entpacken
    with zipfile.ZipFile(zipname, 'r') as zip_ref:
        zip_ref.extractall(temp_dir)

    # Datei im temporären Ordner öffnen und letztes Zeichen entfernen
    filepath = os.path.join(temp_dir, filename)
    remove_last_character(filepath)

    # Datei im temporären Ordner erweitern
    with open(filepath, 'a') as file:
        file.write(additional_data)

    # Neues Zip-Archiv erstellen
    with zipfile.ZipFile(zipname, 'w') as zip_ref:
        # Dateien aus dem temporären Ordner zum neuen Archiv hinzufügen
        for root, dirs, files in os.walk(temp_dir):
            for file in files:
                zip_ref.write(os.path.join(root, file), os.path.relpath(os.path.join(root, file), temp_dir))

    # Temporären Ordner löschen
    shutil.rmtree(temp_dir)
def fillWeatherInXML(weather):
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
    if weather.precipitation_type == "rain":
        num_drops_value = str(int((float(weather.precipitation_intensity)*maxRaindrops)))
    fogDensity = str(maxFogDensity/float(weather.fog_visualRange))

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
    #extend_file_in_zip('C:\\Users\\stefan\\Downloads\\BeamNG.tech.v0.31.3.0\\BeamNG.tech.v0.31.3.0\\gameengine.zip',
    #                   'art\\weather\\defaults.json', additional_data)
    replace_file_in_zip('C:\\Users\\stefan\\Downloads\\BeamNG.tech.v0.31.3.0\\BeamNG.tech.v0.31.3.0\\gameengine.zip',
                       'art\\weather\\defaults.json', new_content)

