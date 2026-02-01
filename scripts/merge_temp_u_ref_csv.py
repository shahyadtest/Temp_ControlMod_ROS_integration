#!/usr/bin/env python3
import csv
from pathlib import Path

base = Path("/home/cristian/bags/export_csv")
temp_path = base / "temp.csv"
u_path    = base / "u.csv"
ref_path  = base / "ref.csv"
out_path  = base / "temp_u_ref.csv"

def read_series(p):
    t, v = [], []
    with p.open() as f:
        r = csv.DictReader(f)
        for row in r:
            t.append(float(row["t"]))
            v.append(float(row["value"]))
    return t, v

tT, temp = read_series(temp_path)
tU, u    = read_series(u_path)
tR, ref  = read_series(ref_path)

# En la práctica, tus tres listas deberían tener largo similar.
n = min(len(tT), len(tU), len(tR))

with out_path.open("w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["t", "temp", "u", "ref"])
    for i in range(n):
        # usamos el tiempo de temp como referencia
        w.writerow([f"{tT[i]:.9f}", temp[i], u[i], ref[i]])

print(f"OK: wrote {n} rows -> {out_path}")
