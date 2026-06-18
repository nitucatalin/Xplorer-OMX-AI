#!/usr/bin/env python3
"""
reset_objects.py — repune obiectele la pozitiile initiale din scena.

Echivalentul pasului din RUNBOOK_J5: "intre rulari, repozitioneaza obiectul
la POI". Ruleaza intre rularile campaniei (orchestratorul are pauza intre
runuri) ca obiectul sa fie din nou la POI pentru urmatoarea rulare.

Utilizare (in container, cu simularea pornita):
  ros2 run xplorer_omx_sim reset_objects
"""
import math
import os
import subprocess
import sys


def load_scene():
    import yaml
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('xplorer_omx_sim'),
                            'config', 'pois.yaml')
    except Exception:
        path = os.path.join(os.path.dirname(__file__), '..',
                            'config', 'pois.yaml')
    with open(path) as f:
        return yaml.safe_load(f), path


def reset_object(index, poi):
    """Teleporteaza obiectul POI-ului inapoi la pozitia initiala.
    Returneaza True la succes. Folosit si de go_collect intre episoadele
    unei campanii."""
    name = f'obj_{index}'
    z = {'cub': 0.0175, 'prisma_drept': 0.015}.get(poi['object'], 0.0)
    req = (f'name: "{name}", position: {{x: {poi["obj_x"]}, '
           f'y: {poi["obj_y"]}, z: {z + 0.02}}}, '
           f'orientation: {{w: 1.0}}')
    cmd = ['gz', 'service', '-s', '/world/lab_world/set_pose',
           '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
           '--timeout', '2000', '--req', req]
    r = subprocess.run(cmd, capture_output=True, text=True)
    return 'data: true' in (r.stdout or '')


def main():
    scene, path = load_scene()
    print(f'Scena: {path}')
    n_ok = 0
    for i, poi in enumerate(scene['pois']):
        ok = reset_object(i, poi)
        n_ok += ok
        print(f"  obj_{i}_{poi['object']} -> "
              f"({poi['obj_x']}, {poi['obj_y']}): {'OK' if ok else 'ESEC'}")
    print(f'{n_ok}/{len(scene["pois"])} obiecte repozitionate')
    sys.exit(0 if n_ok else 1)


if __name__ == '__main__':
    main()
