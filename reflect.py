import json
import os
import glob

# Mirror Left* paths/autos into Right* copies (change SRC/DST to flip direction)
SRC = 'Left'
DST = 'Right'

Y_REFLECTION_LINE = 4.021328
Y_TOTAL = 2 * Y_REFLECTION_LINE


def reflect_y(y):
    return Y_TOTAL - y


def rename(s):
    return s.replace(SRC, DST)


def save_new(file, data):
    new_filename = rename(os.path.basename(file))
    new_path = os.path.join(os.path.dirname(file), new_filename)
    if os.path.exists(new_path):
        print(f'Skipping {new_path}, already exists')
        return
    with open(new_path, 'w') as f:
        json.dump(data, f, indent=2)
    print(f'Wrote {new_path}')


def process_paths():
    path_files = glob.glob(f'src/main/deploy/pathplanner/paths/{SRC}*.path')
    for file in path_files:
        with open(file, 'r') as f:
            data = json.load(f)

        if data.get('folder') is not None:
            data['folder'] = rename(data['folder'])

        for wp in data.get('waypoints', []):
            for key in ('anchor', 'prevControl', 'nextControl'):
                if wp.get(key) is not None:
                    wp[key]['y'] = reflect_y(wp[key]['y'])
            if wp.get('linkedName') is not None:
                wp['linkedName'] = rename(wp['linkedName'])

        for rt in data.get('rotationTargets', []):
            if rt.get('rotationDegrees') is not None:
                rt['rotationDegrees'] = -rt['rotationDegrees']

        for key in ('goalEndState', 'idealStartingState'):
            state = data.get(key)
            if state is not None and state.get('rotation') is not None:
                state['rotation'] = -state['rotation']

        save_new(file, data)


def process_commands(cmd):
    if cmd is None:
        return
    if cmd.get('type') == 'path':
        if 'pathName' in cmd.get('data', {}):
            cmd['data']['pathName'] = rename(cmd['data']['pathName'])
    elif cmd.get('type') in ('sequential', 'parallel', 'race', 'deadline'):
        for c in cmd.get('data', {}).get('commands', []):
            process_commands(c)


def process_autos():
    auto_files = glob.glob(f'src/main/deploy/pathplanner/autos/{SRC}*.auto')
    for file in auto_files:
        with open(file, 'r') as f:
            data = json.load(f)

        if data.get('folder') is not None:
            data['folder'] = rename(data['folder'])

        process_commands(data.get('command'))
        save_new(file, data)


def update_settings():
    settings_file = 'src/main/deploy/pathplanner/settings.json'
    with open(settings_file, 'r') as f:
        data = json.load(f)
    for key in ('pathFolders', 'autoFolders'):
        if key in data:
            new_folders = []
            for folder in data[key]:
                if folder.startswith(SRC):
                    dst_folder = rename(folder)
                    if dst_folder not in data[key] and dst_folder not in new_folders:
                        new_folders.append(dst_folder)
            data[key].extend(new_folders)
    with open(settings_file, 'w') as f:
        json.dump(data, f, indent=2)


if __name__ == '__main__':
    if __name__ == '__main__':
        print('cwd:', os.getcwd())
        print('paths:', glob.glob(f'src/main/deploy/pathplanner/paths/{SRC}*.path'))
        print('autos:', glob.glob(f'src/main/deploy/pathplanner/autos/{SRC}*.auto'))
        process_paths()
        process_autos()
        update_settings()
    process_paths()
    process_autos()
    update_settings()