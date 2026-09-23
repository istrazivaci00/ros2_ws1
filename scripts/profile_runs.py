import csv
import sys

BIN = 10.0


def profile(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    t0 = float(rows[0]['t'])
    bins = {}
    for r in rows:
        k = int((float(r['t']) - t0) // BIN)
        bins.setdefault(k, []).append(float(r['e_pos']))
    return {k: sum(v) / len(v) * 100.0 for k, v in bins.items()}


profs = {p.split('/')[-1]: profile(p) for p in sys.argv[1:]}
kmax = max(max(p) for p in profs.values())

print('sekunde'.ljust(12) + ''.join(n[:10].rjust(12) for n in profs))
for k in range(kmax + 1):
    line = f'{int(k * BIN):>4}-{int((k + 1) * BIN):<7}'
    for n in profs:
        v = profs[n].get(k)
        line += '           -' if v is None else f'{v:>12.2f}'
    print(line)