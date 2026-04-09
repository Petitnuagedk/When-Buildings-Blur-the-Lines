import csv, os, numpy as np

base = r'C:\Users\hugol\Documents\Firenze\B\j1\SASB-data'
for scenario, src in [('SA', 'UrbanRaCompDir-SA'), ('SB', 'UrbanRaCompDir-v2')]:
    src_path = os.path.join(base, src)
    if not os.path.isdir(src_path):
        continue
    epoch = sorted(d for d in os.listdir(src_path) if d.startswith('Epoch_'))[0]
    vals = {}
    for algo in ['aodv', 'olsr', 'dsdv']:
        for lm in ['Friis']:
            nd = os.path.join(src_path, epoch, lm, algo, 'numNodes')
            if not os.path.isdir(nd):
                continue
            for n in sorted(os.listdir(nd), key=int):
                f = os.path.join(nd, n, 'Network_traffic_mapping.csv')
                if not os.path.isfile(f):
                    continue
                with open(f) as fp:
                    h = [x.strip() for x in fp.readline().split(',')]
                    row = dict(zip(h, [x.strip() for x in fp.readline().split(',')]))
                v = float(row.get('RouteSignalizationPacketsSent', 0))
                vals.setdefault(algo, []).append((int(n), v))
    print(f'\n=== {scenario} (Friis) ===')
    for algo, pts in vals.items():
        mn = min(v for _, v in pts)
        mx = max(v for _, v in pts)
        ratio = mx / mn if mn > 0 else float('inf')
        print(f'  {algo:4s}  min={mn:>12,.0f}  max={mx:>12,.0f}  ratio={ratio:.1f}x')
    all_vals = [v for pts in vals.values() for _, v in pts]
    print(f'  ALL   min={min(all_vals):>12,.0f}  max={max(all_vals):>12,.0f}')
