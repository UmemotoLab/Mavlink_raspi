#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Estimate fixed AR-marker-local -> Motive rigid-body pose offsets from AR-hook CSV."""
import argparse, csv, json, math
from pathlib import Path
import numpy as np

def q_to_R(q):
    x,y,z,w = np.asarray(q, dtype=float)
    n = np.linalg.norm([x,y,z,w])
    if not np.isfinite(n) or n < 1e-12: raise ValueError("invalid quaternion")
    x,y,z,w = np.array([x,y,z,w])/n
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]
    ])

def R_to_q(R):
    # stable branch-based conversion
    tr = float(np.trace(R))
    if tr > 0:
        s=math.sqrt(tr+1.0)*2
        q=[(R[2,1]-R[1,2])/s,(R[0,2]-R[2,0])/s,(R[1,0]-R[0,1])/s,0.25*s]
    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s=math.sqrt(1+R[0,0]-R[1,1]-R[2,2])*2
        q=[0.25*s,(R[0,1]+R[1,0])/s,(R[0,2]+R[2,0])/s,(R[2,1]-R[1,2])/s]
    elif R[1,1] > R[2,2]:
        s=math.sqrt(1+R[1,1]-R[0,0]-R[2,2])*2
        q=[(R[0,1]+R[1,0])/s,0.25*s,(R[1,2]+R[2,1])/s,(R[0,2]-R[2,0])/s]
    else:
        s=math.sqrt(1+R[2,2]-R[0,0]-R[1,1])*2
        q=[(R[0,2]+R[2,0])/s,(R[1,2]+R[2,1])/s,0.25*s,(R[1,0]-R[0,1])/s]
    q=np.asarray(q,dtype=float); q/=np.linalg.norm(q)
    return q

def mean_quaternion(qs):
    qs=np.asarray(qs,dtype=float)
    ref=qs[0]
    qs=np.array([-q if np.dot(q,ref)<0 else q for q in qs])
    A=qs.T@qs
    vals,vecs=np.linalg.eigh(A)
    q=vecs[:,np.argmax(vals)]
    if q[3] < 0: q=-q
    return q/np.linalg.norm(q)

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("csv", help="AR-hook CSV file")
    ap.add_argument("--max-time-diff", type=float, default=0.08,
                    help="maximum AR/Motive pose timestamp difference (s)")
    ap.add_argument("--min-samples", type=int, default=20)
    ap.add_argument("--output", default=None, help="output JSON (default: beside this script)")
    args=ap.parse_args()
    rows=list(csv.DictReader(open(args.csv,newline="",encoding="utf-8-sig")))
    output={}
    for name, rb in (("hand",1),("cargo",2)):
        prefix="AR_Hand" if name=="hand" else "AR_Cargo"
        ar_pos_cols=[f"{prefix}_PreOffset_{axis}_NED_m" for axis in "XYZ"]
        ar_q_cols=[f"{prefix}_PreOffset_{axis}_NED" for axis in ("QX","QY","QZ","QW")]
        # Each CSV row is one logging instant. Do not compare the timestamp numbers
        # directly: AR frame time and Motive sender time may use different clock bases.
        # Received flags ensure both poses exist in the same logged row.
        ar_t=f"AR_{'Hand' if name=='hand' else 'Cargo'}_Pose_Time_s"
        mo_t=f"Motive_RB{rb}_Pose_Time_s"
        mo_p=[f"Motive_RB{rb}_{axis}_NED_m" for axis in "XYZ"]
        mo_q=[f"Motive_RB{rb}_Pose_{axis}" for axis in ("QX","QY","QZ","QW")]
        trans=[]; quats=[]; errs=[]
        for row in rows:
            try:
                if int(float(row[f"AR_{'Hand' if name=='hand' else 'Cargo'}_Pose_Received"])) != 1: continue
                if int(float(row[f"Motive_RB{rb}_Pose_Received"])) != 1: continue
                pa=np.array([float(row[c]) for c in ar_pos_cols])
                qa=np.array([float(row[c]) for c in ar_q_cols])
                pm=np.array([float(row[c]) for c in mo_p])
                qm=np.array([float(row[c]) for c in mo_q])
                if not np.all(np.isfinite(np.r_[pa,qa,pm,qm])): continue
                Ra=q_to_R(qa); Rm=q_to_R(qm)
                # T_offset = inverse(T_AR) * T_Motive; corrected AR pose = T_AR * T_offset.
                Roff=Ra.T@Rm
                toff=Ra.T@(pm-pa)
                trans.append(toff); quats.append(R_to_q(Roff))
            except (KeyError,ValueError,TypeError,ZeroDivisionError):
                continue
        if len(trans)<args.min_samples:
            print(f"{name}: usable pairs {len(trans)} < {args.min_samples}; no offset generated")
            continue
        t=np.median(np.asarray(trans),axis=0)
        q=mean_quaternion(quats)
        output[name]={"position_local_m":[float(v) for v in t],
                      "quaternion_local_xyzw":[float(v) for v in q],
                      "sample_count":len(trans),
                      "pairing_method":"same CSV row; no direct cross-clock timestamp comparison"}
        print(f"{name}: {len(trans)} pairs; local translation {t.round(5)} m")
    if not output:
        raise SystemExit("No offsets generated. Check CSV columns, time bases, and --max-time-diff.")
    out=Path(args.output) if args.output else Path(__file__).with_name("world_pose_offsets.json")
    out.write_text(json.dumps(output,indent=2),encoding="utf-8")
    print(f"Saved: {out}")
if __name__=="__main__": main()
