#!/usr/bin/env python3
import sys, csv, math, os

if len(sys.argv) < 2:
    print("usage: eval.py <path/to/gen_trace.csv>")
    sys.exit(2)

path = sys.argv[1]
if not os.path.isfile(path):
    print(f"gen_trace.csv not found: {path}")
    sys.exit(2)

with open(path, newline='') as f:
    rows = list(csv.DictReader(f))

if not rows:
    print("empty gen_trace.csv")
    sys.exit(3)

def f(x): 
    try: return float(x)
    except: return float('inf')

dg = [f(r.get('dist_goal', 'inf')) for r in rows]
if any(math.isinf(x) or math.isnan(x) for x in dg):
    print("dist_goal contains invalid values")
    sys.exit(4)

start = dg[0]
last  = dg[-1]
m     = min(dg)
imprAbs = start - last
imprRel = (imprAbs / max(start,1e-9)) if start>0 else 0.0

print(f"start={start:.6f}  min={m:.6f}  last={last:.6f}  "
      f"imprAbs={imprAbs:.6f}  imprRel={imprRel:.6f}")

# スモーク基準：前進している or 最終誤差が 2cm 未満なら合格
if (imprAbs > 0.0) or (last < 0.02):
    sys.exit(0)
sys.exit(1)
