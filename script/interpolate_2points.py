#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import argparse

def linspace(a: float, b: float, n: int):
    """Genera n valores equiespaciados entre a y b (incluye ambos extremos)."""
    if n == 1:
        return [a]
    step = (b - a) / (n - 1)
    return [a + i * step for i in range(n)]

def interpolate_points(p1, p2, n, include_endpoints=True):
    """
    Interpola n puntos entre p1 y p2.
    - Si include_endpoints=True, incluye p1 y p2 (n >= 2).
    - Si include_endpoints=False, devuelve n puntos estrictamente internos (n >= 1).
    """
    if include_endpoints:
        if n < 2:
            raise ValueError("Con extremos incluidos, X (n) debe ser >= 2.")
        xs = linspace(p1[0], p2[0], n)
        ys = linspace(p1[1], p2[1], n)
        zs = linspace(p1[2], p2[2], n)
        return list(zip(xs, ys, zs))
    else:
        if n < 1:
            raise ValueError("Sin extremos, X (n) debe ser >= 1.")
        # Genera n+2 puntos con extremos y descarta primero y último
        xs = linspace(p1[0], p2[0], n + 2)[1:-1]
        ys = linspace(p1[1], p2[1], n + 2)[1:-1]
        zs = linspace(p1[2], p2[2], n + 2)[1:-1]
        return list(zip(xs, ys, zs))

def main():
    parser = argparse.ArgumentParser(
        description="Interpola X puntos entre p1(x,y,z) y p2(x,y,z)."
    )
    parser.add_argument("X", type=int, help="Número de puntos a generar (ver notas de extremos).")
    parser.add_argument("x1", type=float)
    parser.add_argument("y1", type=float)
    parser.add_argument("z1", type=float)
    parser.add_argument("x2", type=float)
    parser.add_argument("y2", type=float)
    parser.add_argument("z2", type=float)
    parser.add_argument("--exclude-endpoints", action="store_true",
                        help="Generar solo puntos internos (excluye p1 y p2).")
    parser.add_argument("--precision", type=int, default=6,
                        help="Dígitos decimales en la salida (por defecto 6).")
    parser.add_argument("--format", choices=["csv", "txt"], default="csv",
                        help="Formato de salida (csv|txt). Por defecto csv.")
    args = parser.parse_args()

    p1 = (args.x1, args.y1, args.z1)
    p2 = (args.x2, args.y2, args.z2)

    try:
        pts = interpolate_points(p1, p2, args.X, include_endpoints=not args.exclude_endpoints)
    except ValueError as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)

    fmt = f"{{:.{args.precision}f}}"
    if args.format == "csv":
        # cabecera
        print("i,x,y,z")
        for i, (x, y, z) in enumerate(pts):
            print(f"{i},{fmt.format(x)},{fmt.format(y)},{fmt.format(z)}")
    else:  # txt
        for i, (x, y, z) in enumerate(pts):
            print(f"{i}: x={fmt.format(x)}, y={fmt.format(y)}, z={fmt.format(z)}")

if __name__ == "__main__":
    main()
