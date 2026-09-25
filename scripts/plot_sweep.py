"""Kriva APE po udelu grubih gresaka, po robusnom jezgru."""

import csv
import statistics as st
import sys
from collections import defaultdict

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

COLOR = {'none': '#0072B2', 'huber': '#D55E00', 'cauchy': '#009E73'}
NAME = {'none': 'bez jezgra', 'huber': 'Huber', 'cauchy': 'Cauchy'}
INK, MUTED, GRID = '#1a1a1a', '#5b5b5b', '#dcdcdc'
OFFSET = {'none': 0, 'huber': -9, 'cauchy': 9}

src = sys.argv[1] if len(sys.argv) > 1 else 'results/mc.csv'
out = sys.argv[2] if len(sys.argv) > 2 else 'results/mc.png'

data = defaultdict(list)
with open(src) as f:
    for row in csv.DictReader(f):
        data[(row['robust'], float(row['frac']))].append(float(row['ape_cm']))

kernels = sorted({k for k, _ in data}, key=lambda k: list(COLOR).index(k))
fracs = sorted({v for _, v in data})

fig, ax = plt.subplots(figsize=(7.2, 4.6))
for k in kernels:
    m = [st.mean(data[(k, f)]) for f in fracs]
    s = [st.stdev(data[(k, f)]) if len(data[(k, f)]) > 1 else 0 for f in fracs]
    x = [f * 100 for f in fracs]
    ax.errorbar(x, m, yerr=s, color=COLOR[k], lw=2, marker='o',
                markersize=7, capsize=3, label=NAME[k], zorder=3)
    ax.annotate(NAME[k], (x[-1], m[-1]), xytext=(8, OFFSET[k]),
                textcoords='offset points', color=INK,
                fontsize=10, va='center')

ax.set_yscale('log')
ax.set_xlabel('udeo pogrešnih zatvaranja petlje  [%]', color=MUTED)
ax.set_ylabel('APE RMS  [cm]  (log)', color=MUTED)
ax.set_title('Tačnost pose-grafa u zavisnosti od kontaminacije',
             color=INK, fontsize=12, loc='left', pad=12)
ax.grid(True, which='both', color=GRID, lw=0.7, zorder=0)
ax.set_axisbelow(True)
for sp in ('top', 'right'):
    ax.spines[sp].set_visible(False)
for sp in ('left', 'bottom'):
    ax.spines[sp].set_color(GRID)
ax.tick_params(colors=MUTED)
ax.legend(frameon=False, labelcolor=INK, loc='upper left')
ax.set_xlim(-2, max(x) + 9)

fig.tight_layout()
fig.savefig(out, dpi=150)
print(f'upisano: {out}')