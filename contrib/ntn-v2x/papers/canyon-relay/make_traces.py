#!/usr/bin/env python3
"""SUMO stop-and-go traces of the street: a 1.2 km ring per direction, two lanes, a signal every 240 m (cycle 90 s,
45 s green, 3 s yellow, offset drawn per start), urban speed limit 11.2 m/s. One trace per start and density:
    traces/n<N>_r<rep>.csv   first line dt; then per vehicle (ns-3 index i, lane i % 4): x0,dir,s(0),s(dt),...
where s is the distance driven since t = 0 after a 120 s warm-up. Lanes do not change. Resumable."""
import gzip, os, random, subprocess, sys, tempfile, xml.etree.ElementTree as ET
HERE = os.path.dirname(os.path.abspath(__file__))
TR = os.path.join(HERE, "traces")
L, NSIG, DT, WARM, SIM = 1200.0, 5, 1.0, 120, 305
V = 11.2


def ring(td, rep):
    import math
    rad = L / (2 * math.pi)
    with open(f"{td}/n.nod.xml", "w") as f:
        f.write("<nodes>\n")
        for k in range(NSIG):
            a = 2 * math.pi * k / NSIG
            f.write(f'<node id="j{k}" x="{rad*math.cos(a):.2f}" y="{rad*math.sin(a):.2f}" type="traffic_light" tl="t{k}"/>\n')
        f.write("</nodes>\n")
    with open(f"{td}/n.edg.xml", "w") as f:
        f.write("<edges>\n")
        for k in range(NSIG):
            # a curved shape so that the length of each edge is close to one block
            pts = []
            for q in range(13):
                a = 2 * math.pi * (k + q / 12) / NSIG
                pts.append(f"{rad*math.cos(a):.2f},{rad*math.sin(a):.2f}")
            f.write(f'<edge id="e{k}" from="j{k}" to="j{(k+1)%NSIG}" numLanes="2" speed="{V}" shape="{" ".join(pts)}"/>\n')
        f.write("</edges>\n")
    subprocess.run(["netconvert", "-n", f"{td}/n.nod.xml", "-e", f"{td}/n.edg.xml", "-o", f"{td}/n.net.xml",
                    "--no-turnarounds", "true", "--junctions.corner-detail", "0", "--no-internal-links", "true"],
                   check=True, capture_output=True)
    rng = random.Random(9000 + rep)
    with open(f"{td}/tls.add.xml", "w") as f:
        f.write("<additional>\n")
        for k in range(NSIG):
            f.write(f'<tlLogic id="t{k}" type="static" programID="p" offset="{rng.randint(0, 89)}">'
                    '<phase duration="45" state="GG"/><phase duration="3" state="yy"/><phase duration="42" state="rr"/></tlLogic>\n')
        f.write("</additional>\n")


def one_dir(td, rep, n, d):
    """Ring simulation of n vehicles of one direction. Returns per vehicle (lane, s0, [s(t)]) in ring meters."""
    rng = random.Random(100 * rep + 7 * n + (d + 1))
    rou = f"{td}/r{d}.rou.xml"
    with open(rou, "w") as f:
        f.write('<routes>\n<vType id="car" length="4.5" minGap="2.0" accel="2.6" decel="4.5" sigma="0.5" tau="1.0" '
                'speedFactor="normc(1.0,0.15,0.6,1.3)" lcStrategic="-1" lcCooperative="0" lcSpeedGain="0" lcKeepRight="0"/>\n')
        veh = []
        for i in range(n):
            veh.append((rng.randrange(NSIG), i % 2))
        counts = {}
        for i, (e, ln) in enumerate(veh):
            order = [f"e{(e + q) % NSIG}" for q in range(NSIG)]
            f.write(f'<vehicle id="v{i}" type="car" depart="0" departLane="{ln}" departPos="random_free" departSpeed="0">'
                    f'<route edges="{" ".join(order)}" repeat="40"/></vehicle>\n')
        f.write("</routes>\n")
    fcd = f"{td}/f{d}.xml"
    subprocess.run(["nice", "-n", "10", "sumo", "-n", f"{td}/n.net.xml", "-r", rou, "-a", f"{td}/tls.add.xml",
                    "--begin", "0", "--end", str(WARM + SIM), "--step-length", "0.5", "--seed", str(5000 + 11 * rep + 2 * (d + 1) + n),
                    "--fcd-output", fcd, "--fcd-output.attributes", "speed,lane,pos", "--device.fcd.period", str(DT),
                    "--no-step-log", "true", "--no-warnings", "true", "--collision.action", "none",
                    "--default.speeddev", "0"], check=True, capture_output=True)
    vs = {}
    for _, el in ET.iterparse(fcd, events=("end",)):
        if el.tag == "timestep":
            t = float(el.get("time"))
            if t >= WARM and abs(t / DT - round(t / DT)) < 1e-6:
                for v in el:
                    vs.setdefault(v.get("id"), []).append((t, v.get("lane"), float(v.get("pos", v.get("lanePosition", 0))), float(v.get("speed"))))
            el.clear()
    os.unlink(fcd)
    out = []
    for i in range(n):
        rows = vs[f"v{i}"]
        lane = int(rows[0][1].rsplit("_", 1)[1])
        e = int(rows[0][1].split("_")[0][1:])
        s0 = e * L / NSIG + rows[0][2]
        s = [0.0]
        for a, b in zip(rows[:-1], rows[1:]):
            s.append(s[-1] + 0.5 * (a[3] + b[3]) * DT)     # trapezoid of the recorded speed
        out.append((i % 2, lane, s0, s))
    return out


def trace(rep, n):
    f = os.path.join(TR, f"n{n}_r{rep}.csv")
    if os.path.exists(f):
        return
    with tempfile.TemporaryDirectory(dir=HERE, prefix="tmp_sumo_") as td:
        ring(td, rep)
        # ns-3 lanes i % 4 in (0, 1) drive +x, in (2, 3) drive -x
        nplus = sum(1 for i in range(n) if i % 4 < 2)
        P = one_dir(td, rep, nplus, +1)
        M = one_dir(td, rep, n - nplus, -1)
    lines = [f"{DT}"]
    ip = im = 0
    for i in range(n):
        if i % 4 < 2:
            _, _, s0, s = P[ip]; ip += 1; x0, d = s0, 1
        else:
            _, _, s0, s = M[im]; im += 1; x0, d = (L - s0) % L, -1
        lines.append(f"{x0:.1f},{d}," + ",".join(f"{x:.1f}" for x in s))
    open(f + ".tmp", "w").write("\n".join(lines) + "\n")
    os.replace(f + ".tmp", f)


if __name__ == "__main__":
    os.makedirs(TR, exist_ok=True)
    for n, reps in ((60, range(20)), (240, range(10, 20))):
        for rep in reps:
            trace(rep, n)
            print(n, rep, flush=True)
