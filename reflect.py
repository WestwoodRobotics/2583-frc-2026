import json
import os
import glob

Y_REFLECTION_LINE = 4.021328
Y_TOTAL = 2 * Y_REFLECTION_LINE

def reflect_y(y):
    return Y_TOTAL - y

def process_paths():
    path_files = glob.glob('src/main/deploy/pathplanner/paths/Right*.path')
    for file in path_files:
        with open(file, 'r') as f:
            data = json.load(f)
        
        if 'folder' in data and data['folder'] is not None:
            data['folder'] = data['folder'].replace('Right', 'Left')
            
        for wp in data.get('waypoints', []):
            if 'anchor' in wp and wp['anchor'] is not None:
                wp['anchor']['y'] = reflect_y(wp['anchor']['y'])
            if 'prevControl' in wp and wp['prevControl'] is not None:
                wp['prevControl']['y'] = reflect_y(wp['prevControl']['y'])
            if 'nextControl' in wp and wp['nextControl'] is not None:
                wp['nextControl']['y'] = reflect_y(wp['nextControl']['y'])
            if 'linkedName' in wp and wp['linkedName'] is not None:
                wp['linkedName'] = wp['linkedName'].replace('Right', 'Left')
                
        for rt in data.get('rotationTargets', []):
            if 'rotationDegrees' in rt and rt['rotationDegrees'] is not None:
                rt['rotationDegrees'] = -rt['rotationDegrees']
                
        if 'goalEndState' in data and data['goalEndState'] is not None:
            if 'rotation' in data['goalEndState'] and data['goalEndState']['rotation'] is not None:
                data['goalEndState']['rotation'] = -data['goalEndState']['rotation']
                
        if 'idealStartingState' in data and data['idealStartingState'] is not None:
            if 'rotation' in data['idealStartingState'] and data['idealStartingState']['rotation'] is not None:
                data['idealStartingState']['rotation'] = -data['idealStartingState']['rotation']
                
        new_filename = os.path.basename(file).replace('Right', 'Left')
        new_path = os.path.join(os.path.dirname(file), new_filename)
        with open(new_path, 'w') as f:
            json.dump(data, f, indent=2)

def process_commands(cmd):
    if cmd is None: return
    if cmd.get('type') == 'path':
        if 'pathName' in cmd.get('data', {}):
            cmd['data']['pathName'] = cmd['data']['pathName'].replace('Right', 'Left')
    elif cmd.get('type') in ('sequential', 'parallel', 'race', 'deadline'):
        for c in cmd.get('data', {}).get('commands', []):
            process_commands(c)

def process_autos():
    auto_files = glob.glob('src/main/deploy/pathplanner/autos/Right*.auto')
    for file in auto_files:
        with open(file, 'r') as f:
            data = json.load(f)
            
        if 'folder' in data and data['folder'] is not None:
            data['folder'] = data['folder'].replace('Right', 'Left')
            
        process_commands(data.get('command'))
        
        new_filename = os.path.basename(file).replace('Right', 'Left')
        new_path = os.path.join(os.path.dirname(file), new_filename)
        with open(new_path, 'w') as f:
            json.dump(data, f, indent=2)

def update_settings():
    settings_file = 'src/main/deploy/pathplanner/settings.json'
    with open(settings_file, 'r') as f:
        data = json.load(f)
    if 'pathFolders' in data:
        new_folders = []
        for folder in data['pathFolders']:
            if folder.startswith('Right'):
                left_folder = folder.replace('Right', 'Left')
                if left_folder not in data['pathFolders'] and left_folder not in new_folders:
                    new_folders.append(left_folder)
        data['pathFolders'].extend(new_folders)
    if 'autoFolders' in data:
        new_folders = []
        for folder in data['autoFolders']:
            if folder.startswith('Right'):
                left_folder = folder.replace('Right', 'Left')
                if left_folder not in data['autoFolders'] and left_folder not in new_folders:
                    new_folders.append(left_folder)
        data['autoFolders'].extend(new_folders)
        
    with open(settings_file, 'w') as f:
        json.dump(data, f, indent=2)

if __name__ == '__main__':
    process_paths()
    process_autos()
    update_settings()
