import csv
import math
import sys

WINDOW = 80.0  # sekundi od prvog uzorka


def stats(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    t0 = float(rows[0]['t'])
    sel = [r for r in rows if float(r['t']) - t0 <= WINDOW]
    e = [float(r['e_pos']) for r in sel]
    y = [abs(float(r['e_yaw_deg'])) for r in sel]
    n = len(e)
    return n, len(rows), math.sqrt(sum(v * v for v in e) / n), max(e), sum(e) / n, max(y)


print(f"{'fajl':<20}{'n':>6}{'ukupno':>8}{'RMS cm':>9}{'max cm':>9}"
      f"{'sred cm':>9}{'max yaw':>9}")
for p in sys.argv[1:]:
    n, n_all, rms, mx, mean, my = stats(p)
    print(f'{p.split("/")[-1]:<20}{n:>6}{n_all:>8}{rms*100:>9.2f}{mx*100:>9.2f}'
          f'{mean*100:>9.2f}{my:>9.2f}')