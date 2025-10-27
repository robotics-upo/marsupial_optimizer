#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import math
from typing import Dict, List, Tuple, Any

try:
    import yaml  # pip install pyyaml
except ImportError:
    print("Falta PyYAML. Instala con: pip install pyyaml")
    sys.exit(1)


Vec3 = Tuple[float, float, float]


def euclid(a: Vec3, b: Vec3) -> float:
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)


def dist_origin(p: Vec3) -> float:
    return math.sqrt(p[0]**2 + p[1]**2 + p[2]**2)


def extract_poses(block: Dict[str, Any], prefix: str = "poses") -> List[Dict[str, Any]]:
    """
    Extrae y ordena los elementos posesX por el sufijo numérico.
    Devuelve una lista de dicts con al menos:
      - time
      - pose.position.{x,y,z}
    """
    items = []
    for k, v in block.items():
        if k.startswith(prefix):
            # obtener sufijo numérico (e.g., 'poses12' -> 12)
            try:
                idx = int(k[len(prefix):])
            except ValueError:
                continue
            items.append((idx, v))
    items.sort(key=lambda t: t[0])
    return [v for _, v in items]


def pose_to_xyz(pose_entry: Dict[str, Any]) -> Vec3:
    pos = pose_entry["pose"]["position"]
    return (float(pos["x"]), float(pos["y"]), float(pos["z"]))


def safe_get_time(pose_entry: Dict[str, Any]) -> float:
    # Algunos bloques usan "time: 0", otros podrían no tenerlo
    return float(pose_entry.get("time", 0.0))


def main(path_yaml: str):
    with open(path_yaml, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    ugv_block = data.get("marsupial_ugv", {})
    uav_block = data.get("marsupial_uav", {})
    tether_block = data.get("tether", {})

    ugv_poses = extract_poses(ugv_block)
    uav_poses = extract_poses(uav_block)

    # Convertimos a listas de (t, xyz)
    ugv_seq: List[Tuple[float, Vec3]] = [(safe_get_time(p), pose_to_xyz(p)) for p in ugv_poses]
    uav_seq: List[Tuple[float, Vec3]] = [(safe_get_time(p), pose_to_xyz(p)) for p in uav_poses]

    # Alineación simple por índice (asumiendo mismo número de muestras o mínima común)
    n = min(len(ugv_seq), len(uav_seq))

    # --- Distancia UGV <-> UAV por instante ---
    print("\n=== Distancia UGV ↔ UAV por pose (euclídea 3D) ===")
    print(f"{'idx':>3} | {'t [s]':>10} | {'UGV(x,y,z)':>24} | {'UAV(x,y,z)':>24} | {'dist':>10}")
    dist_ugv_uav: List[float] = []
    for i in range(n):
        t_u, p_u = ugv_seq[i]
        t_a, p_a = uav_seq[i]
        # si tiempos difieren, usamos el de UGV (o podrías promediar)
        d = euclid(p_u, p_a)
        dist_ugv_uav.append(d)
        print(f"{i:>3} | {t_u:>10.6f} | {str(p_u):>24} | {str(p_a):>24} | {d:>10.6f}")

    # --- Distancias sucesivas y longitud total (UGV y UAV) ---
    def successive_and_total(seq: List[Tuple[float, Vec3]]) -> Tuple[List[float], float]:
        dsteps: List[float] = []
        for i in range(1, len(seq)):
            dsteps.append(euclid(seq[i-1][1], seq[i][1]))
        return dsteps, sum(dsteps)

    ugv_steps, ugv_total = successive_and_total(ugv_seq)
    uav_steps, uav_total = successive_and_total(uav_seq)

    print("\n=== Distancias sucesivas UGV (paso a paso) ===")
    for i, d in enumerate(ugv_steps, start=1):
        print(f"UGV {i-1}->{i}: {d:.6f}")
    print(f"Longitud total trayecto UGV: {ugv_total:.6f}\n")

    print("=== Distancias sucesivas UAV (paso a paso) ===")
    for i, d in enumerate(uav_steps, start=1):
        print(f"UAV {i-1}->{i}: {d:.6f}")
    print(f"Longitud total trayecto UAV: {uav_total:.6f}\n")

    # --- Distancia al origen (opcional) ---
    print("=== Distancia al origen por pose (UGV/UAV) ===")
    print(f"{'idx':>3} | {'t [s]':>10} | {'|UGV|':>10} | {'|UAV|':>10}")
    for i in range(n):
        t_u, p_u = ugv_seq[i]
        t_a, p_a = uav_seq[i]
        print(f"{i:>3} | {t_u:>10.6f} | {dist_origin(p_u):>10.6f} | {dist_origin(p_a):>10.6f}")

    # --- Comparación con tether.lengthX si existe (opcional) ---
    # Extraemos lengthX ordenados
    tether_lengths: List[Tuple[int, float]] = []
    for k, v in tether_block.items():
        if isinstance(v, dict) and k.startswith("length"):
            try:
                idx = int(k[len("length"):])
            except ValueError:
                continue
            tether_lengths.append((idx, float(v.get("length", float("nan")))))
    tether_lengths.sort(key=lambda t: t[0])

    if tether_lengths:
        print("\n=== Comparación con tether.lengthX (si corresponde por índice) ===")
        m = min(len(dist_ugv_uav), len(tether_lengths))
        print(f"{'idx':>3} | {'dist(UGV-UAV)':>14} | {'tether.length':>14} | {'Δ (abs)':>10}")
        for i in range(m):
            d = dist_ugv_uav[i]
            _, L = tether_lengths[i]
            delta = abs(d - L)
            print(f"{i:>3} | {d:>14.6f} | {L:>14.6f} | {delta:>10.6f}")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Uso: python distancias_marsupial.py <ruta_al_yaml>")
        print("Ejemplo: python distancias_marsupial.py datos.yaml")
        sys.exit(1)
    main(sys.argv[1])
