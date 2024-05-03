import os
import shutil
import zipfile
def remove_last_character(filepath):
    with open(filepath, 'r+') as file:
        file.seek(0, os.SEEK_END)
        file.seek(file.tell() - 1, os.SEEK_SET)
        file.truncate()

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

# Beispielaufruf
name = 'rainy4'
time_value = 0.80
num_drops_value = 700

additional_data = f'''
"{name}": {{
    "TimeOfDay": {{
      "time": {time_value}
    }},
    "Precipitation": {{
      "numDrops": {num_drops_value}
    }},
    "CloudLayer": {{
      "coverage": 1
    }},
    "LevelInfo": {{
      "fogDensity": 0.005
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
extend_file_in_zip('C:\\Users\\stefan\\Downloads\\BeamNG.tech.v0.31.3.0\\BeamNG.tech.v0.31.3.0\\gameengine.zip', 'art\\weather\\defaults.json', additional_data)
