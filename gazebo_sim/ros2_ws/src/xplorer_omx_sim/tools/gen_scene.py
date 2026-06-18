#!/usr/bin/env python3
"""
gen_scene.py — genereaza scena cu obiecte RANDOM si lantul de POI-uri.

Plaseaza N obiecte (cub / prisma triunghiulara / prisma dreptunghiulara) in
pozitii aleatoare in spatiul liber al hartii si calculeaza pentru fiecare un
POI (B, C, D, ...) = poza la care trebuie sa opreasca robotul ca obiectul sa
fie exact in punctul de pick al bratului (FK pe lantul CAD: tip la
(-0.366, -0.057) in frame base_link — bratul e montat cu fata spre spate).

Scrie:
  worlds/lab_world.sdf   — world-ul complet (pereti/obstacole fixe + obiecte)
  config/pois.yaml       — POI A (start) + lista B, C, D... pentru
                           mission_multi_poi
  meshes/prism_tri.stl   — mesh-ul prismei triunghiulare (generat o data)

Utilizare:
  python3 gen_scene.py [--n 4] [--seed 42] [--pkg <cale_pachet>]
Dupa regenerare ruleaza din nou `colcon build` in container.
"""
import argparse
import math
import random
import struct
from pathlib import Path

# punctul de pick in frame-ul robotului = CENTRUL REAL DINTRE CLESTI la
# poza PICK (FK pe geometria CAD a degetelor). Robotul PARCHEAZA CU
# SPATELE la obiect: POI = pozitia robotului astfel incat obiectul sa
# pice exact intre lamelele gripperului
PICK_OFFSET = (-0.3302, -0.0643)

# geometria spatiului liber (identica cu harta lab_map)
ROOM = (-1.5, 6.5, -2.5, 3.5)          # x0, x1, y0, y1 interior
RECT_OBS = [                            # (cx, cy, hx, hy)
    (4.5, -1.2, 0.4, 0.4),
    (0.5, 2.0, 0.3, 0.6),
]
CIRC_OBS = [(4.0, 2.2, 0.3)]            # (cx, cy, r)
POI_A = (1.0, 0.0, 0.0)

WORLD_HEADER = '''<?xml version="1.0" ?>
<!-- GENERAT de tools/gen_scene.py — world laborator + obiecte random.
     Obiectele si POI-urile (config/pois.yaml) sunt generate impreuna. -->
<sdf version="1.8">
  <world name="lab_world">

    <physics name="default_physics" type="ignored">
      <!-- 4 ms: injumatateste sarcina CPU a fizicii (fara GPU in Docker,
           pasul de 2 ms ducea simularea sub timp real si Nav2 sufoca) -->
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system"
            name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.3 0.2 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <surface><friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material><ambient>0.75 0.75 0.75 1</ambient><diffuse>0.75 0.75 0.75 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- pereti laborator: interior x in [-1.5, 6.5], y in [-2.5, 3.5] -->
    <model name="walls">
      <static>true</static>
      <link name="link">
        <collision name="north_c"><pose>2.5 3.575 0.5 0 0 0</pose>
          <geometry><box><size>8.3 0.15 1.0</size></box></geometry></collision>
        <visual name="north_v"><pose>2.5 3.575 0.5 0 0 0</pose>
          <geometry><box><size>8.3 0.15 1.0</size></box></geometry>
          <material><ambient>0.85 0.82 0.75 1</ambient><diffuse>0.85 0.82 0.75 1</diffuse></material></visual>
        <collision name="south_c"><pose>2.5 -2.575 0.5 0 0 0</pose>
          <geometry><box><size>8.3 0.15 1.0</size></box></geometry></collision>
        <visual name="south_v"><pose>2.5 -2.575 0.5 0 0 0</pose>
          <geometry><box><size>8.3 0.15 1.0</size></box></geometry>
          <material><ambient>0.85 0.82 0.75 1</ambient><diffuse>0.85 0.82 0.75 1</diffuse></material></visual>
        <collision name="east_c"><pose>6.575 0.5 0.5 0 0 0</pose>
          <geometry><box><size>0.15 6.0 1.0</size></box></geometry></collision>
        <visual name="east_v"><pose>6.575 0.5 0.5 0 0 0</pose>
          <geometry><box><size>0.15 6.0 1.0</size></box></geometry>
          <material><ambient>0.85 0.82 0.75 1</ambient><diffuse>0.85 0.82 0.75 1</diffuse></material></visual>
        <collision name="west_c"><pose>-1.575 0.5 0.5 0 0 0</pose>
          <geometry><box><size>0.15 6.0 1.0</size></box></geometry></collision>
        <visual name="west_v"><pose>-1.575 0.5 0.5 0 0 0</pose>
          <geometry><box><size>0.15 6.0 1.0</size></box></geometry>
          <material><ambient>0.85 0.82 0.75 1</ambient><diffuse>0.85 0.82 0.75 1</diffuse></material></visual>
      </link>
    </model>

    <model name="box_a">
      <static>true</static>
      <pose>4.5 -1.2 0.4 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>0.8 0.8 0.8</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>0.8 0.8 0.8</size></box></geometry>
          <material><ambient>0.5 0.35 0.2 1</ambient><diffuse>0.5 0.35 0.2 1</diffuse></material></visual>
      </link>
    </model>
    <model name="box_b">
      <static>true</static>
      <pose>0.5 2.0 0.4 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>0.6 1.2 0.8</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>0.6 1.2 0.8</size></box></geometry>
          <material><ambient>0.5 0.35 0.2 1</ambient><diffuse>0.5 0.35 0.2 1</diffuse></material></visual>
      </link>
    </model>
    <model name="pillar">
      <static>true</static>
      <pose>4.0 2.2 0.4 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><cylinder><radius>0.3</radius><length>0.8</length></cylinder></geometry></collision>
        <visual name="v"><geometry><cylinder><radius>0.3</radius><length>0.8</length></cylinder></geometry>
          <material><ambient>0.6 0.6 0.65 1</ambient><diffuse>0.6 0.6 0.65 1</diffuse></material></visual>
      </link>
    </model>
'''

WORLD_FOOTER = '''
  </world>
</sdf>
'''

COLORS = [
    ('rosu', '0.85 0.10 0.10 1'),
    ('verde', '0.10 0.65 0.20 1'),
    ('albastru', '0.10 0.30 0.75 1'),
    ('galben', '0.85 0.75 0.10 1'),
    ('mov', '0.55 0.20 0.65 1'),
    ('portocaliu', '0.90 0.50 0.10 1'),
]


def write_prism_tri_stl(path, side=1.0, height=1.0):
    """Prisma triunghiulara (sectiune triunghi echilateral, unitate, scalabila
    din SDF). Baza centrata in origine, inaltimea pe z."""
    s = side
    h = height
    r = s / math.sqrt(3.0)          # raza circumscrisa
    pts = [(r * math.cos(a), r * math.sin(a))
           for a in (math.pi / 2, math.pi / 2 + 2 * math.pi / 3,
                     math.pi / 2 + 4 * math.pi / 3)]
    lo = [(x, y, 0.0) for x, y in pts]
    hi = [(x, y, h) for x, y in pts]
    tris = [
        (lo[0], lo[2], lo[1]),               # baza (normala -z)
        (hi[0], hi[1], hi[2]),               # capac (+z)
    ]
    for i in range(3):                        # fete laterale (2 tri fiecare)
        j = (i + 1) % 3
        tris.append((lo[i], lo[j], hi[j]))
        tris.append((lo[i], hi[j], hi[i]))
    with open(path, 'wb') as f:
        f.write(b'\0' * 80)
        f.write(struct.pack('<I', len(tris)))
        for a, b, c in tris:
            u = [b[k] - a[k] for k in range(3)]
            v = [c[k] - a[k] for k in range(3)]
            n = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2],
                 u[0] * v[1] - u[1] * v[0])
            ln = math.sqrt(sum(x * x for x in n)) or 1.0
            f.write(struct.pack('<3f', *(x / ln for x in n)))
            for p in (a, b, c):
                f.write(struct.pack('<3f', *p))
            f.write(struct.pack('<H', 0))


def free(x, y, margin):
    x0, x1, y0, y1 = ROOM
    if not (x0 + margin < x < x1 - margin and y0 + margin < y < y1 - margin):
        return False
    for cx, cy, hx, hy in RECT_OBS:
        if abs(x - cx) < hx + margin and abs(y - cy) < hy + margin:
            return False
    for cx, cy, r in CIRC_OBS:
        if (x - cx) ** 2 + (y - cy) ** 2 < (r + margin) ** 2:
            return False
    return True


def object_sdf(idx, kind, x, y, yaw, color):
    # nume UNIFORM obj_<i> (tipul ramane in pois.yaml): topicele de
    # ground-truth si bridge-urile sunt pre-declarate pe aceste nume
    name = f'obj_{idx}'
    mat = (f'<material><ambient>{color}</ambient>'
           f'<diffuse>{color}</diffuse></material>')
    inert = ('<inertial><mass>0.05</mass>'
             '<inertia><ixx>1e-5</ixx><iyy>1e-5</iyy><izz>1e-5</izz>'
             '<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>')
    fric = ('<surface><friction><ode><mu>1.2</mu><mu2>1.2</mu2>'
            '</ode></friction></surface>')
    # dimensiuni APUCABILE: deschiderea lamelelor gripperului e 3.3 cm
    if kind == 'cub':
        geom = '<box><size>0.028 0.028 0.028</size></box>'
        z = 0.014
    elif kind == 'prisma_drept':
        geom = '<box><size>0.05 0.026 0.026</size></box>'
        z = 0.013
    else:  # prisma_tri
        geom = ('<mesh><uri>package://xplorer_omx_sim/meshes/prism_tri.stl'
                '</uri><scale>0.03 0.03 0.035</scale></mesh>')
        z = 0.0
    return (f'''
    <model name="{name}">
      <pose>{x:.3f} {y:.3f} {z} 0 0 {yaw:.3f}</pose>
      <link name="link">
        {inert}
        <collision name="c"><geometry>{geom}</geometry>{fric}</collision>
        <visual name="v"><geometry>{geom}</geometry>{mat}</visual>
      </link>
      <plugin filename="gz-sim-odometry-publisher-system"
              name="gz::sim::systems::OdometryPublisher">
        <odom_topic>/model/{name}/ground_truth</odom_topic>
        <odom_frame>gt_world</odom_frame>
        <robot_base_frame>{name}_link</robot_base_frame>
        <odom_publish_frequency>10</odom_publish_frequency>
      </plugin>
    </model>''', name)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--n', type=int, default=1,
                    help='numar de obiecte / POI-uri (default 1: scenariul '
                         'de baza POI-A -> POI-B -> POI-A)')
    ap.add_argument('--seed', type=int, default=42)
    ap.add_argument('--pkg', type=str,
                    default=str(Path(__file__).resolve().parents[1]))
    args = ap.parse_args()
    rng = random.Random(args.seed)
    pkg = Path(args.pkg)

    write_prism_tri_stl(pkg / 'meshes' / 'prism_tri.stl')

    # 1. pozitii random pentru obiecte; tipurile sunt impartite round-robin
    #    (garantat cel putin un cub / prisma triunghiulara / prisma
    #    dreptunghiulara cand n >= 3), ordinea e amestecata de seed
    kinds_cycle = ['cub', 'prisma_tri', 'prisma_drept']
    rng.shuffle(kinds_cycle)
    objs = []
    tries = 0
    while len(objs) < args.n and tries < 5000:
        tries += 1
        x = rng.uniform(ROOM[0] + 0.8, ROOM[1] - 0.8)
        y = rng.uniform(ROOM[2] + 0.8, ROOM[3] - 0.8)
        if not free(x, y, 0.75):
            continue
        if (x - POI_A[0]) ** 2 + (y - POI_A[1]) ** 2 < 1.2 ** 2:
            continue
        if any((x - ox) ** 2 + (y - oy) ** 2 < 1.0 ** 2 for ox, oy, *_ in objs):
            continue
        kind = kinds_cycle[len(objs) % 3]
        objs.append((x, y, kind, rng.uniform(0, math.pi),
                     rng.choice(COLORS)))
    if len(objs) < args.n:
        raise SystemExit('Nu am gasit destule pozitii libere — scade --n')

    # 2. ordonare in lant nearest-neighbor pornind din POI A
    ordered = []
    cur = (POI_A[0], POI_A[1])
    rest = list(objs)
    while rest:
        rest.sort(key=lambda o: (o[0] - cur[0]) ** 2 + (o[1] - cur[1]) ** 2)
        o = rest.pop(0)
        ordered.append(o)
        cur = (o[0], o[1])

    # 3. POI per obiect: robotul opreste cu obiectul in punctul de pick
    #    (spatele robotului). yaw ales spre punctul anterior => Nav2 se
    #    roteste la goal si bratul ramane cu fata la obiect.
    pois = []
    prev = (POI_A[0], POI_A[1])
    for i, (ox, oy, kind, oyaw, color) in enumerate(ordered):
        base_psi = math.atan2(prev[1] - oy, prev[0] - ox)
        placed = False
        for dpsi in (0, 0.5, -0.5, 1.0, -1.0, 1.6, -1.6, 2.4, -2.4, 3.14):
            psi = base_psi + dpsi
            px = ox - (PICK_OFFSET[0] * math.cos(psi)
                       - PICK_OFFSET[1] * math.sin(psi))
            py = oy - (PICK_OFFSET[0] * math.sin(psi)
                       + PICK_OFFSET[1] * math.cos(psi))
            if free(px, py, 0.45):
                pois.append(dict(name=chr(ord('B') + i), object=kind,
                                 obj_x=round(ox, 3), obj_y=round(oy, 3),
                                 x=round(px, 3), y=round(py, 3),
                                 yaw=round(psi, 4)))
                placed = True
                break
        if not placed:
            raise SystemExit(f'POI imposibil pentru obiectul {i} la '
                             f'({ox:.2f},{oy:.2f}) — alt --seed')
        prev = (pois[-1]['x'], pois[-1]['y'])

    # 4. scrie world-ul
    parts = [WORLD_HEADER]
    parts.append('\n    <!-- ===== OBIECTE DE COLECTAT (generate random, '
                 f'seed={args.seed}) ===== -->')
    for i, (ox, oy, kind, oyaw, (cname, rgba)) in enumerate(ordered):
        sdf, name = object_sdf(i, kind, ox, oy, oyaw, rgba)
        parts.append(sdf)
    parts.append(WORLD_FOOTER)
    world_path = pkg / 'worlds' / 'lab_world.sdf'
    world_path.write_text(''.join(parts))

    # 5. scrie pois.yaml
    y = ['# GENERAT de tools/gen_scene.py — POI A (start) + lantul de',
         '# obiecte. Folosit de mission_multi_poi.',
         f'# seed: {args.seed}',
         'poi_a:',
         f'  x: {POI_A[0]}',
         f'  y: {POI_A[1]}',
         f'  yaw: {POI_A[2]}',
         'pois:']
    for p in pois:
        y.append(f"  - name: {p['name']}")
        for k in ('x', 'y', 'yaw', 'object', 'obj_x', 'obj_y'):
            y.append(f'    {k}: {p[k]}')
    (pkg / 'config' / 'pois.yaml').write_text('\n'.join(y) + '\n')

    print(f'world: {world_path}')
    print(f'pois:  {pkg / "config" / "pois.yaml"}')
    for p in pois:
        print(f"  POI {p['name']}: robot ({p['x']}, {p['y']}, "
              f"yaw {p['yaw']}) -> {p['object']} la "
              f"({p['obj_x']}, {p['obj_y']})")


if __name__ == '__main__':
    main()
