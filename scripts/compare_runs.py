import csv
import math
import sys


def stats(path):
    e, y = [], []
    with open(path) as f:
        for row in csv.DictReader(f):
            e.append(float(row['e_pos']))
            y.append(abs(float(row['e_yaw_deg'])))
    n = len(e)
    return n, math.sqrt(sum(v * v for v in e) / n), max(e), sum(e) / n, max(y)


print(f"{'fajl':<26}{'n':>6}{'RMS cm':>10}{'max cm':>10}{'sred cm':>10}{'max yaw':>10}")
for p in sys.argv[1:]:
    n, rms, mx, mean, my = stats(p)
    print(f'{p.split("/")[-1]:<26}{n:>6}{rms*100:>10.2f}{mx*100:>10.2f}'
          f'{mean*100:>10.2f}{my:>10.2f}')
