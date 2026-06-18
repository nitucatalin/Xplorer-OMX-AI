#!/usr/bin/env python3
"""
flatten_onshape_urdf.py — genereaza urdf/xplorer_omx_real.urdf din exportul
CAD Onshape al ansamblului Xplorer-A + OMX-AI-F (amr_2ac_robot_22_01_26).

Exportul Onshape are 505 link-uri cu mase CAD nerealiste si nu poate fi
simulat direct. Scriptul:
  1. calculeaza transformata globala a fiecarui link (toate joint-urile la 0);
  2. partitioneaza link-urile in segmente functionale:
       base | 4 roti | shoulder(pan) | upper_arm(lift) | forearm(elbow)
       | gripper_base(wrist_flex) | finger_a | finger_b
     dupa lantul de joint-uri mobile dof_joint1..6 / dof_base_*_wheel;
  3. reorienteaza modelul: înainte = +Y_cad -> +X, origine = centrul rotilor;
  4. emite un URDF nou: vizuale = mesh-urile CAD reale (compuse pe segment),
     coliziuni = primitive simple, fizica = diff-drive 4 roti + controllere
     de pozitie pe articulatiile bratului (aceleasi topice /arm/*).

Utilizare:  python3 flatten_onshape_urdf.py <cale_amr_2ac_robot_22_01_26> <cale_pachet>
"""
import json
import math
import struct
import sys
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

# ── joint-urile mobile pastrate din export (continuous din perechi) ──
ARM_JOINTS_MAP = {
    'dof_joint1_2': 'shoulder_pan',
    'dof_joint2_0': 'shoulder_lift',
    'dof_joint3_2': 'elbow_flex',
    'dof_joint4_2': 'wrist_flex',
    'dof_joint5_2': 'gripper_left',
    'dof_joint6_2': 'gripper_right',
}
WHEEL_JOINTS_MAP = {
    'dof_base_left_1_wheel_joint_2': 'wheel_rear_left',
    'dof_base_left_1_wheel_joint_5': 'wheel_rear_right',
    'dof_base_left_2_wheel_joint_2': 'wheel_front_right',
    'dof_base_right_2_wheel_joint_2': 'wheel_front_left',
}
KEPT = {**ARM_JOINTS_MAP, **WHEEL_JOINTS_MAP}

# parintele cinematic al fiecarui segment nou
SEG_PARENT = {
    'shoulder_pan': 'base_link', 'shoulder_lift': 'shoulder_pan',
    'elbow_flex': 'shoulder_lift', 'wrist_flex': 'elbow_flex',
    'gripper_left': 'wrist_flex', 'gripper_right': 'wrist_flex',
    'wheel_rear_left': 'base_link', 'wheel_rear_right': 'base_link',
    'wheel_front_left': 'base_link', 'wheel_front_right': 'base_link',
}
SEG_LINKNAME = {  # numele link-ului nou per segment
    'shoulder_pan': 'shoulder_link', 'shoulder_lift': 'upper_arm_link',
    'elbow_flex': 'forearm_link', 'wrist_flex': 'gripper_base_link',
    'gripper_left': 'finger_left', 'gripper_right': 'finger_right',
    'wheel_rear_left': 'wheel_rl', 'wheel_rear_right': 'wheel_rr',
    'wheel_front_left': 'wheel_fl', 'wheel_front_right': 'wheel_fr',
}
ARM_LIMITS = {
    'shoulder_pan': (-3.1415, 3.1415, 10, 3.0),
    'shoulder_lift': (-1.9, 1.9, 12, 3.0),
    'elbow_flex': (-1.9, 1.9, 10, 3.0),
    'wrist_flex': (-1.9, 1.9, 8, 3.0),
    'gripper_left': (-1.1, 1.1, 5, 3.0),
    'gripper_right': (-1.1, 1.1, 5, 3.0),
}
ARM_MASS = {
    'shoulder_link': 0.20, 'upper_arm_link': 0.18, 'forearm_link': 0.15,
    'gripper_base_link': 0.10, 'finger_left': 0.02, 'finger_right': 0.02,
}
PID = {  # p, i, d, imax, cmdmax
    'shoulder_pan': (15, 0.2, 1.0, 1, 10), 'shoulder_lift': (25, 0.5, 1.5, 2, 12),
    'elbow_flex': (20, 0.4, 1.2, 2, 10), 'wrist_flex': (10, 0.2, 0.6, 1, 8),
    # degete: strangere ferma (comanda merge dincolo de contact, PID-ul
    # tine apasat pe obiect pe tot transportul)
    'gripper_left': (12, 0.2, 0.6, 2, 8), 'gripper_right': (12, 0.2, 0.6, 2, 8),
}


def rpy_to_mat(rpy):
    R, P, Y = rpy
    cr, sr = math.cos(R), math.sin(R)
    cp, sp = math.cos(P), math.sin(P)
    cy, sy = math.cos(Y), math.sin(Y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])


def mat_to_rpy(R):
    sy = -R[2, 0]
    sy = max(-1.0, min(1.0, sy))
    p = math.asin(sy)
    if abs(abs(sy) - 1.0) < 1e-9:
        r = math.atan2(R[0, 1], R[1, 1]); y = 0.0
    else:
        r = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(R[1, 0], R[0, 0])
    return r, p, y


def T_of(origin):
    if origin is None:
        return np.eye(4)
    xyz = [float(v) for v in (origin.get('xyz') or '0 0 0').split()]
    rpy = [float(v) for v in (origin.get('rpy') or '0 0 0').split()]
    T = np.eye(4)
    T[:3, :3] = rpy_to_mat(rpy)
    T[:3, 3] = xyz
    return T


def stl_vertices(path, step=9):
    """Citeste varfurile dintr-un STL binar (subsample pentru bbox)."""
    data = Path(path).read_bytes()
    if data[:5] == b'solid' and b'facet' in data[:200]:
        # ASCII STL (rar) — parse simplu
        verts = []
        for line in data.decode(errors='ignore').splitlines():
            line = line.strip()
            if line.startswith('vertex'):
                verts.append([float(v) for v in line.split()[1:4]])
        return np.array(verts[::max(1, step // 3)] or [[0, 0, 0]])
    n = struct.unpack('<I', data[80:84])[0]
    arr = np.frombuffer(data[84:84 + n * 50],
                        dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'),
                                        ('attr', '<u2')]), count=n)
    v = arr['v'].reshape(-1, 3)
    return v[::step] if len(v) > step else v


def is_degenerate(path):
    """Mesh degenerat (punct/linie, fara volum) — exclus din vizuale/bbox."""
    v = stl_vertices(path, step=1)
    ext = v.max(axis=0) - v.min(axis=0)
    return int(np.sum(ext < 1e-3)) >= 2


def main():
    src = Path(sys.argv[1]) if len(sys.argv) > 1 else \
        Path(__file__).resolve().parents[4].parent / 'amr_2ac_robot_22_01_26'
    pkg = Path(sys.argv[2]) if len(sys.argv) > 2 else \
        Path(__file__).resolve().parents[1]
    urdf_in = src / 'urdf' / 'amr_2ac_robot_22_01_26.urdf'
    meshes_in = src / 'meshes'
    meshes_out = pkg / 'meshes'
    meshes_out.mkdir(exist_ok=True)

    root = ET.parse(urdf_in).getroot()
    joints = root.findall('joint')
    links = {l.get('name'): l for l in root.findall('link')}

    jinfo, parent_joint = {}, {}
    for j in joints:
        n = j.get('name')
        p = j.find('parent').get('link')
        c = j.find('child').get('link')
        ax = j.find('axis')
        jinfo[n] = dict(parent=p, child=c, T=T_of(j.find('origin')),
                        axis=np.array([float(v) for v in
                                       (ax.get('xyz') if ax is not None
                                        else '0 0 1').split()]))
        parent_joint[c] = n

    # 1. transformata globala per link (toate joint-urile la zero)
    gT = {'root': np.eye(4)}

    def global_T(link):
        if link in gT:
            return gT[link]
        ji = jinfo[parent_joint[link]]
        gT[link] = global_T(ji['parent']) @ ji['T']
        return gT[link]
    for l in links:
        global_T(l)

    # 2. segmentul fiecarui link = primul joint mobil pastrat de deasupra
    def segment_of(link):
        while link != 'root':
            jn = parent_joint[link]
            if jn in KEPT:
                return KEPT[jn]
            link = jinfo[jn]['parent']
        return 'base'
    seg_links = {}
    for l in links:
        seg_links.setdefault(segment_of(l), []).append(l)

    # 3. frame-ul nou: origine = centrul rotilor, inainte (+X) = -Y_cad.
    #    ANSAMBLUL E IDENTIC CU CAD-UL: bratul ramane montat in coltul lui
    #    original, neatins fata de sasiu. Alegerea "inainte = -Y_cad" face
    #    ca acel colt sa fie in SPATELE directiei de mers: bratul sta cu
    #    fata spre spatele platformei, culege obiectele de pe sol din
    #    spatele robotului si le depune in cutia de colectare de pe sasiu.
    wheel_pos = []
    for jn in WHEEL_JOINTS_MAP:
        ji = jinfo[jn]
        wheel_pos.append((global_T(ji['parent']) @ ji['T'])[:3, 3])
    wheel_pos = np.array(wheel_pos)
    center = wheel_pos.mean(axis=0)          # include z-ul axelor
    Rz = rpy_to_mat((0, 0, math.pi / 2))     # -Y_cad -> +X_nou
    T_base = np.eye(4)
    T_base[:3, :3] = Rz
    T_base[:3, 3] = -Rz @ center
    # T_nou(link) = T_base @ T_cad(link); baza la inaltimea axelor rotilor

    # Bratul ramane in coltul lui original din CAD, dar este rotit la 180
    # de grade DIN PROPRIA BAZA (in jurul axei joint-ului shoulder_pan):
    # gripperul impachetat ajunge in EXTERIOR, peste marginea din spate,
    # nu peste interiorul platformei (ca pe robotul real).
    ji_pan = jinfo['dof_joint1_2']
    T_pan0 = T_base @ global_T(ji_pan['parent']) @ ji_pan['T']
    a = ji_pan['axis'] / np.linalg.norm(ji_pan['axis'])
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    A = np.eye(4)
    A[:3, :3] = np.eye(3) + 2.0 * (K @ K)    # rot(axa_pan, pi)
    ARM_FLIP = T_pan0 @ A @ np.linalg.inv(T_pan0)

    def newT(link):
        return T_base @ gT[link]

    # frame global (nou) al fiecarui joint pastrat; lantul bratului primeste
    # ARM_FLIP (mutare la 180). Vizualele bratului primesc acelasi premul,
    # deci transformata relativa vizual<->segment ramane cea din CAD.
    arm_segs = set(ARM_JOINTS_MAP.values())
    seg_premul = {seg: (ARM_FLIP if seg in arm_segs else np.eye(4))
                  for seg in list(KEPT.values()) + ['base']}
    segT = {'base': np.eye(4)}
    seg_axis = {}
    for jn, seg in KEPT.items():
        ji = jinfo[jn]
        Tj = seg_premul[seg] @ T_base @ global_T(ji['parent']) @ ji['T']
        segT[seg] = Tj
        seg_axis[seg] = Tj[:3, :3] @ ji['axis']  # axa in frame global nou

    # redenumeste rotile dupa pozitia REALA in frame-ul nou (etichetele din
    # exportul Onshape nu corespund pozitiilor dupa reorientare)
    rename = {}
    for seg in list(WHEEL_JOINTS_MAP.values()):
        x, y = segT[seg][0, 3], segT[seg][1, 3]
        rename[seg] = ('wheel_'
                       + ('front' if x > 0 else 'rear') + '_'
                       + ('left' if y > 0 else 'right'))
    assert len(set(rename.values())) == 4, rename
    # rename in doua faze (etichetele vechi/noi se pot suprapune intre roti)
    tmp_links = {old: seg_links.pop(old, []) for old in rename}
    tmp_T = {old: segT.pop(old) for old in rename}
    tmp_ax = {old: seg_axis.pop(old) for old in rename}
    for old, new in rename.items():
        seg_links[new] = tmp_links[old]
        segT[new] = tmp_T[old]
        seg_axis[new] = tmp_ax[old]
    for jn in list(WHEEL_JOINTS_MAP):
        WHEEL_JOINTS_MAP[jn] = rename[WHEEL_JOINTS_MAP[jn]]
    SEG_LINKNAME.update({
        'wheel_front_left': 'wheel_fl', 'wheel_front_right': 'wheel_fr',
        'wheel_rear_left': 'wheel_rl', 'wheel_rear_right': 'wheel_rr'})
    print('roti:', {jn.split("dof_base_")[1]: seg
                    for jn, seg in WHEEL_JOINTS_MAP.items()})

    # 4. raza rotii din bbox-ul complet al mesh-ului de cauciuc (tire)
    tire_r = []
    for l, lk in links.items():
        for vis in lk.findall('visual'):
            mesh = vis.find('geometry/mesh')
            if mesh is None or 'Tire' not in mesh.get('filename'):
                continue
            fname = mesh.get('filename').split('/')[-1]
            fpath = meshes_in / fname
            if not fpath.exists():
                continue
            v = stl_vertices(fpath, step=1)
            Tv = newT(l) @ T_of(vis.find('origin'))
            vg = (Tv[:3, :3] @ v.T).T + Tv[:3, 3]
            tire_r.append((vg[:, 2].max() - vg[:, 2].min()) / 2)
    wheel_radius = float(np.mean(tire_r)) if tire_r else 0.055
    print(f'raza roata (din mesh cauciuc): {wheel_radius:.4f} m '
          f'({len(tire_r)} cauciucuri)')

    # cache mesh-uri degenerate (puncte/linii din export)
    degenerate = set()
    for f in meshes_in.iterdir():
        if f.suffix.lower() == '.stl' and is_degenerate(f):
            degenerate.add(f.name)
    print('mesh-uri degenerate excluse:', sorted(degenerate))

    # 5. bbox-ul segmentului base (pentru coliziunea sasiului)
    pts = []
    for l in seg_links['base']:
        for vis in links[l].findall('visual'):
            mesh = vis.find('geometry/mesh')
            if mesh is None:
                continue
            fname = mesh.get('filename').split('/')[-1]
            fpath = meshes_in / fname
            if not fpath.exists() or fname in degenerate:
                continue
            v = stl_vertices(fpath, step=30)
            Tv = newT(l) @ T_of(vis.find('origin'))
            pts.append((Tv[:3, :3] @ v.T).T + Tv[:3, 3])
    P = np.vstack(pts)
    # doar corpul de deasupra axelor (sub axe sunt cauciucuri/suporti)
    body = P[P[:, 2] > 0.005]
    bmin, bmax = body.min(axis=0), body.max(axis=0)
    print('bbox sasiu (nou):', bmin.round(3), bmax.round(3))

    # 6. emite URDF-ul
    out = []
    w = out.append
    w('<?xml version="1.0"?>')
    w('<!-- GENERAT de tools/flatten_onshape_urdf.py din exportul Onshape')
    w('     amr_2ac_robot_22_01_26 (Xplorer-A + OMX-AI-F). NU edita manual -->')
    w('<robot name="xplorer_omx">')

    def fmt(v):
        return ' '.join(f'{x:.6g}' for x in v)

    def visuals_xml(seg, frame_inv, indent='    '):
        """vizualele tuturor link-urilor CAD ale segmentului, in frame-ul nou"""
        premul = seg_premul.get(seg, np.eye(4))
        s = []
        for l in sorted(seg_links.get(seg, [])):
            for vis in links[l].findall('visual'):
                mesh = vis.find('geometry/mesh')
                if mesh is None:
                    continue
                fname = mesh.get('filename').split('/')[-1]
                if not (meshes_in / fname).exists() or fname in degenerate:
                    continue
                Tv = frame_inv @ premul @ newT(l) @ T_of(vis.find('origin'))
                rpy = mat_to_rpy(Tv[:3, :3])
                col = vis.find('material/color')
                rgba = col.get('rgba') if col is not None else '0.6 0.6 0.6 1'
                s.append(f'{indent}<visual>')
                s.append(f'{indent}  <origin xyz="{fmt(Tv[:3,3])}" rpy="{fmt(rpy)}"/>')
                s.append(f'{indent}  <geometry><mesh filename="package://xplorer_omx_sim/meshes/{fname}"/></geometry>')
                s.append(f'{indent}  <material name="m_{abs(hash(rgba))%99999}"><color rgba="{rgba}"/></material>')
                s.append(f'{indent}</visual>')
        return s

    # ---- base_link ----
    w('  <link name="base_link">')
    w('    <inertial>')
    w('      <mass value="9.0"/>')
    c = (bmin + bmax) / 2
    w(f'      <origin xyz="{fmt(c)}"/>')
    w('      <inertia ixx="0.09" ixy="0" ixz="0" iyy="0.15" iyz="0" izz="0.21"/>')
    w('    </inertial>')
    out.extend(visuals_xml('base', np.eye(4)))
    size = bmax - bmin
    w('    <collision>')
    w(f'      <origin xyz="{fmt(c)}"/>')
    w(f'      <geometry><box size="{fmt(size)}"/></geometry>')
    w('    </collision>')

    # ---- CUTIA DE COLECTARE montata pe sasiu: PATRATA si ASEZATA pe
    #      puntea sasiului (inaltimea puntii e masurata LOCAL, sub
    #      amprenta cutiei — nu varful global al sasiului/catargului) ----
    # LANGA brat (asa a fost antrenat modelul real: cutia alaturi de brat,
    # nu in spatele lui); bratul e la (-0.153,-0.07), cutia in stanga lui,
    # pe punte libera (z~0.08)
    box_cx, box_cy = -0.12, 0.10
    box_l, box_w_, box_h, box_t = 0.16, 0.16, 0.05, 0.01
    under = body[(np.abs(body[:, 0] - box_cx) < box_l / 2)
                 & (np.abs(body[:, 1] - box_cy) < box_w_ / 2)]
    deck = under[(under[:, 2] > 0.04) & (under[:, 2] < 0.12)]
    # nivelul puntii = mediana punctelor placii; cutia sta PE placa
    box_z0 = float(np.median(deck[:, 2])) + 0.005 if len(deck) > 50 \
        else (float(under[:, 2].max()) if len(under) else float(bmax[2]))
    zmax_under = float(under[:, 2].max()) if len(under) else 0.0
    if zmax_under > box_z0 + 0.02:
        print(f'AVERTISMENT: componente pana la z={zmax_under:.3f} sub cutie')
    box_parts = [
        # (cx, cy, cz, sx, sy, sz)
        (box_cx, box_cy, box_z0 + box_t / 2, box_l, box_w_, box_t),
        (box_cx, box_cy + box_w_ / 2 - box_t / 2, box_z0 + box_t + (box_h - box_t) / 2 + 0, box_l, box_t, box_h),
        (box_cx, box_cy - box_w_ / 2 + box_t / 2, box_z0 + box_t + (box_h - box_t) / 2 + 0, box_l, box_t, box_h),
        (box_cx + box_l / 2 - box_t / 2, box_cy, box_z0 + box_t + (box_h - box_t) / 2 + 0, box_t, box_w_, box_h),
        (box_cx - box_l / 2 + box_t / 2, box_cy, box_z0 + box_t + (box_h - box_t) / 2 + 0, box_t, box_w_, box_h),
    ]
    for i, (cx, cy, cz, sx, sy, sz) in enumerate(box_parts):
        w('    <visual>')
        w(f'      <origin xyz="{cx:.4f} {cy:.4f} {cz:.4f}"/>')
        w(f'      <geometry><box size="{sx:.4f} {sy:.4f} {sz:.4f}"/></geometry>')
        w('      <material name="collect_box"><color rgba="0.8 0.6 0.2 1"/></material>')
        w('    </visual>')
        w('    <collision>')
        w(f'      <origin xyz="{cx:.4f} {cy:.4f} {cz:.4f}"/>')
        w(f'      <geometry><box size="{sx:.4f} {sy:.4f} {sz:.4f}"/></geometry>')
        w('    </collision>')
    box_top = box_z0 + box_h + box_t
    print(f'cutie colectare: centru ({box_cx},{box_cy}), '
          f'baza z={box_z0:.3f}, top z={box_top:.3f}')
    w('  </link>')

    # ---- roti ----
    for jn, seg in WHEEL_JOINTS_MAP.items():
        ln = SEG_LINKNAME[seg]
        Tj = segT[seg]
        Tinv = np.linalg.inv(Tj)
        # CRITIC: axele rotilor din exportul CAD au semne INCONSISTENTE
        # (stanga -Y, dreapta +Y global) -> diff-drive-ul inversa virajul
        # cu mersul drept (robotul nu putea vira, doar inainte/inapoi).
        # Normalizam TOATE axele la +Y global (conventia standard:
        # viteza pozitiva a jointului = rulare inainte, pe ambele parti).
        ax_local = Tinv[:3, :3] @ np.array([0.0, 1.0, 0.0])
        w(f'  <link name="{ln}">')
        w('    <inertial>')
        w('      <mass value="0.6"/>')
        w('      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.0019"/>')
        w('    </inertial>')
        out.extend(visuals_xml(seg, Tinv))
        # coliziune cilindru aliniat pe axa locala a rotii
        rpy_c = (math.pi / 2, 0, 0) if abs(ax_local[1]) > 0.9 else \
                ((0, math.pi / 2, 0) if abs(ax_local[0]) > 0.9 else (0, 0, 0))
        w('    <collision>')
        w(f'      <origin xyz="0 0 0" rpy="{fmt(rpy_c)}"/>')
        w(f'      <geometry><cylinder radius="{wheel_radius:.4f}" length="0.045"/></geometry>')
        w('    </collision>')
        w('  </link>')
        rpy_j = mat_to_rpy(Tj[:3, :3])
        w(f'  <joint name="{seg}_joint" type="continuous">')
        w('    <parent link="base_link"/>')
        w(f'    <child link="{ln}"/>')
        w(f'    <origin xyz="{fmt(Tj[:3,3])}" rpy="{fmt(rpy_j)}"/>')
        w(f'    <axis xyz="{fmt(ax_local)}"/>')
        w('    <dynamics damping="0.05" friction="0.05"/>')
        w('  </joint>')

    # ---- brat ----
    seg_col_center, seg_col_size = {}, {}
    for jn, seg in ARM_JOINTS_MAP.items():
        ln = SEG_LINKNAME[seg]
        Tj = segT[seg]
        Tinv = np.linalg.inv(Tj)
        ax_local = Tinv[:3, :3] @ seg_axis[seg]
        parent_seg = SEG_PARENT[seg]
        Tp = np.eye(4) if parent_seg == 'base_link' else segT[parent_seg]
        Trel = np.linalg.inv(Tp) @ Tj
        # bbox segment pentru coliziune simpla
        pts = []
        for l in seg_links.get(seg, []):
            for vis in links[l].findall('visual'):
                mesh = vis.find('geometry/mesh')
                if mesh is None:
                    continue
                fname = mesh.get('filename').split('/')[-1]
                fpath = meshes_in / fname
                if not fpath.exists() or fname in degenerate:
                    continue
                v = stl_vertices(fpath, step=30)
                Tv = Tinv @ seg_premul.get(seg, np.eye(4)) \
                    @ newT(l) @ T_of(vis.find('origin'))
                pts.append((Tv[:3, :3] @ v.T).T + Tv[:3, 3])
        if pts:
            Q = np.vstack(pts)
            qmin, qmax = Q.min(axis=0), Q.max(axis=0)
        else:
            qmin, qmax = np.array([-0.02] * 3), np.array([0.02] * 3)
        qc, qs = (qmin + qmax) / 2, np.maximum(qmax - qmin, 0.01)
        # degetele: bbox-ul ghearei curbe e mult prea gros SI prea lung —
        # la inchidere colturile lui matura sub sol si fizica blocheaza
        # strangerea. Coliziunea devine LAMELA DISTALA reala a ghearei:
        # 12 mm grosime (x local) si 45 mm lungime, ancorata la VARF
        # (capatul departat de incheietura, +z local).
        if seg in ('gripper_left', 'gripper_right'):
            qc, qs = qc.copy(), qs.copy()
            tip_z = qc[2] + qs[2] / 2
            qs[0] = 0.012
            qs[2] = min(qs[2], 0.045)
            qc[2] = tip_z - qs[2] / 2
        seg_col_center[seg] = qc.tolist()
        seg_col_size[seg] = qs.tolist()
        m = ARM_MASS[ln]
        w(f'  <link name="{ln}">')
        w('    <inertial>')
        w(f'      <mass value="{m}"/>')
        w(f'      <origin xyz="{fmt(qc)}"/>')
        ix = m * (qs[1]**2 + qs[2]**2) / 12 + 1e-5
        iy = m * (qs[0]**2 + qs[2]**2) / 12 + 1e-5
        iz = m * (qs[0]**2 + qs[1]**2) / 12 + 1e-5
        w(f'      <inertia ixx="{ix:.6g}" ixy="0" ixz="0" iyy="{iy:.6g}" iyz="0" izz="{iz:.6g}"/>')
        w('    </inertial>')
        out.extend(visuals_xml(seg, Tinv))
        w('    <collision>')
        w(f'      <origin xyz="{fmt(qc)}"/>')
        w(f'      <geometry><box size="{fmt(qs)}"/></geometry>')
        w('    </collision>')
        w('  </link>')
        lo, hi, eff, vel = ARM_LIMITS[seg]
        # axa in frame-ul joint-ului (= frame-ul segmentului nou)
        ax_rel = ax_local
        w(f'  <joint name="{seg}" type="revolute">')
        w(f'    <parent link="{parent_seg if parent_seg=="base_link" else SEG_LINKNAME[parent_seg]}"/>')
        w(f'    <child link="{ln}"/>')
        w(f'    <origin xyz="{fmt(Trel[:3,3])}" rpy="{fmt(mat_to_rpy(Trel[:3,:3]))}"/>')
        w(f'    <axis xyz="{fmt(ax_rel)}"/>')
        w(f'    <limit lower="{lo}" upper="{hi}" effort="{eff}" velocity="{vel}"/>')
        w('    <dynamics damping="0.1" friction="0.05"/>')
        w('  </joint>')

    # ---- frecari roti: SKID-STEER (4 roti fixe) ----
    # virajul cere alunecare laterala; mu2 mare blocheaza rotirea si
    # robotul se taraste. mu1 mare (tractiune longitudinala) + mu2 mic
    # (alunecare laterala libera) = reteta standard pentru skid-steer.
    # fdir1 fixeaza directia lui mu1 pe directia de rulare a rotii.
    for seg in WHEEL_JOINTS_MAP.values():
        w(f'  <gazebo reference="{SEG_LINKNAME[seg]}">')
        w('    <mu1>1.1</mu1><mu2>0.18</mu2>')
        w('    <fdir1>1 0 0</fdir1>')
        w('  </gazebo>')

    # ---- frecare mare pe degete (strangerea obiectului) ----
    for ln in ('finger_left', 'finger_right'):
        w(f'  <gazebo reference="{ln}">')
        w('    <mu1>2.5</mu1><mu2>2.5</mu2>')
        w('  </gazebo>')

    # ---- lidar (pe sasiu, pozitie din CAD-ul LD19 daca exista) ----
    w('  <link name="lidar_link">')
    w('    <inertial><mass value="0.05"/>')
    w('      <inertia ixx="1e-5" ixy="0" ixz="0" iyy="1e-5" iyz="0" izz="1e-5"/></inertial>')
    w('  </link>')
    # pozitia LD19 din CAD
    lidar_xyz = None
    for l in seg_links['base']:
        for vis in links[l].findall('visual'):
            mesh = vis.find('geometry/mesh')
            if mesh is not None and 'LD19' in mesh.get('filename'):
                Tv = newT(l) @ T_of(vis.find('origin'))
                lidar_xyz = Tv[:3, 3] + np.array([0, 0, 0.03])
                break
        if lidar_xyz is not None:
            break
    if lidar_xyz is None:
        lidar_xyz = np.array([0.15, 0.0, bmax[2] + 0.02])
    # planul de scanare trebuie sa fie DEASUPRA sasiului, altfel lidar-ul
    # isi vede propriul corp si orbeste navigarea; pastram x,y din CAD dar
    # ridicam z peste cel mai inalt punct al sasiului (exclus stalpul
    # lidar-ului insusi, < 8 cm in jurul lui)
    horiz = np.linalg.norm(body[:, :2] - lidar_xyz[:2], axis=1)
    chassis_top = body[horiz > 0.08][:, 2].max() if (horiz > 0.08).any() \
        else bmax[2]
    # peste sasiu SI peste cutia de colectare (altfel cutia umbreste scanul)
    lidar_xyz[2] = max(lidar_xyz[2], chassis_top + 0.025, box_top + 0.02)
    print('lidar la:', lidar_xyz.round(3),
          f'(top sasiu {chassis_top:.3f}, top cutie {box_top:.3f})')
    w('  <joint name="lidar_joint" type="fixed">')
    w('    <parent link="base_link"/>')
    w('    <child link="lidar_link"/>')
    w(f'    <origin xyz="{fmt(lidar_xyz)}"/>')
    w('  </joint>')
    w('  <gazebo reference="lidar_link">')
    w('''    <sensor name="ld19" type="gpu_lidar">
      <topic>/scan_gz</topic>
      <update_rate>10</update_rate>
      <gz_frame_id>lidar_link</gz_frame_id>
      <lidar>
        <!-- sectorul din spate (+/-30 grade) e MASCAT: acolo lucreaza
             bratul (pick in spatele robotului) si altfel lidar-ul si-ar
             vedea propriul brat ca obstacol-fantoma fixat de robot,
             otravind AMCL si costmap-urile (LD19 real se configureaza
             identic, cu unghi limitat) -->
        <scan><horizontal>
          <samples>300</samples><resolution>1</resolution>
          <min_angle>-2.61799</min_angle><max_angle>2.61799</max_angle>
        </horizontal></scan>
        <range><min>0.30</min><max>12.0</max><resolution>0.01</resolution></range>
        <noise><type>gaussian</type><mean>0.0</mean><stddev>0.01</stddev></noise>
      </lidar>
      <always_on>1</always_on>
      <visualize>true</visualize>
    </sensor>''')
    w('  </gazebo>')

    # ---- pluginuri gz ----
    w('  <gazebo>')
    w('''    <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
      <left_joint>wheel_front_left_joint</left_joint>
      <left_joint>wheel_rear_left_joint</left_joint>
      <right_joint>wheel_front_right_joint</right_joint>
      <right_joint>wheel_rear_right_joint</right_joint>
      <wheel_separation>0.31</wheel_separation>
      <wheel_radius>''' + f'{wheel_radius:.4f}' + '''</wheel_radius>
      <topic>/model/xplorer_omx/cmd_vel</topic>
      <odom_topic>/model/xplorer_omx/odometry</odom_topic>
      <tf_topic>/model/xplorer_omx/tf</tf_topic>
      <frame_id>odom</frame_id>
      <child_frame_id>base_link</child_frame_id>
      <odom_publish_frequency>30</odom_publish_frequency>
      <max_linear_acceleration>1.0</max_linear_acceleration>
      <max_angular_acceleration>2.0</max_angular_acceleration>
    </plugin>
    <plugin filename="gz-sim-joint-state-publisher-system"
            name="gz::sim::systems::JointStatePublisher">
      <topic>/model/xplorer_omx/joint_states</topic>
    </plugin>
    <!-- ground-truth-ul robotului pe topic cu nume FIX (dynamic_pose/info
         din Harmonic vine fara numele entitatilor si e inutilizabil) -->
    <plugin filename="gz-sim-odometry-publisher-system"
            name="gz::sim::systems::OdometryPublisher">
      <odom_topic>/model/xplorer_omx/ground_truth</odom_topic>
      <odom_frame>gt_world</odom_frame>
      <robot_base_frame>base_link</robot_base_frame>
      <odom_publish_frequency>20</odom_publish_frequency>
    </plugin>''')
    # ---- DetachableJoint per obiect: la strangere, obiectul se "sudeaza"
    #      de gripper (joint fix creat la pozitia relativa CURENTA — nu se
    #      teleporteaza nimic) si nu mai aluneca; la eliberare joint-ul se
    #      desface si obiectul cade in cutie. Solutia standard Gazebo
    #      pentru pick-and-place stabil (frecarea pe contacte mici e
    #      instabila). Pluginul ataseaza automat la pornire — nodul de
    #      manipulare publica DETACH pe toate topicurile imediat la start.
    for obj in [f'obj_{i}' for i in range(6)] + ['obj_spawn']:
        w(f'''    <plugin filename="gz-sim-detachable-joint-system"
            name="gz::sim::systems::DetachableJoint">
      <parent_link>gripper_base_link</parent_link>
      <child_model>{obj}</child_model>
      <child_link>link</child_link>
      <topic>/gripper/detach_{obj}</topic>
      <attach_topic>/gripper/attach_{obj}</attach_topic>
      <suppress_child_warning>true</suppress_child_warning>
    </plugin>''')
    for seg in ARM_JOINTS_MAP.values():
        p, i, d, imax, cmax = PID[seg]
        w(f'''    <plugin filename="gz-sim-joint-position-controller-system"
            name="gz::sim::systems::JointPositionController">
      <joint_name>{seg}</joint_name>
      <topic>/arm/{seg}/cmd_pos</topic>
      <p_gain>{p}</p_gain><i_gain>{i}</i_gain><d_gain>{d}</d_gain>
      <i_max>{imax}</i_max><i_min>-{imax}</i_min>
      <cmd_max>{cmax}</cmd_max><cmd_min>-{cmax}</cmd_min>
    </plugin>''')
    w('  </gazebo>')
    w('</robot>')

    urdf_out = pkg / 'urdf' / 'xplorer_omx_real.urdf'
    urdf_out.write_text('\n'.join(out))
    print('scris:', urdf_out)

    # 7. copiaza mesh-urile folosite
    used = set()
    for l in links.values():
        for vis in l.findall('visual'):
            mesh = vis.find('geometry/mesh')
            if mesh is not None:
                used.add(mesh.get('filename').split('/')[-1])
    n = 0
    for f in used:
        srcf = meshes_in / f
        if srcf.exists():
            shutil.copy2(srcf, meshes_out / f)
            n += 1
    print(f'mesh-uri copiate: {n}/{len(used)}')

    # 8. dump date FK pentru calibrarea pozelor (folosit de compute_poses.py)
    fk = {
        'wheel_radius': wheel_radius,
        'segments': {},
    }
    for jn, seg in ARM_JOINTS_MAP.items():
        parent_seg = SEG_PARENT[seg]
        Tp = np.eye(4) if parent_seg == 'base_link' else segT[parent_seg]
        Trel = np.linalg.inv(Tp) @ segT[seg]
        Tinv = np.linalg.inv(segT[seg])
        fk['segments'][seg] = {
            'parent': parent_seg,
            'T_rel': Trel.tolist(),
            'axis_local': (Tinv[:3, :3] @ seg_axis[seg]).tolist(),
            # centrul si gabaritul coliziunii segmentului (frame propriu) —
            # folosite la calculul centrului dintre clesti (punctul de grasp)
            'collision_center': seg_col_center.get(seg),
            'collision_size': seg_col_size.get(seg),
        }
    (pkg / 'tools' / 'fk_data.json').write_text(json.dumps(fk, indent=1))
    # si in config/ — instalat in share, folosit de IK-ul adaptiv din
    # manipulation_infer_node_sim la runtime
    (pkg / 'config' / 'fk_data.json').write_text(json.dumps(fk, indent=1))
    print('scris: tools/fk_data.json + config/fk_data.json')


if __name__ == '__main__':
    main()
