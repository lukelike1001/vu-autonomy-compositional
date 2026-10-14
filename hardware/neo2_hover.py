#!/usr/bin/env python3
"""The simplest Neo 2 flight: take off, climb to 1.4 m, hover, land. No truck, no tags, no closed loop.

It runs the lab's own transport host, ``tools/neo2_fly.py --profile hover``, with the vu-lab limits from
``vu-autonomy-neo2/docs/RUNBOOK-NEO2.md`` ("On verivital-a51"): ``--alt 1.4 --max-alt 1.9``. It runs ON the flight box
(Taylor's Alienware), started by the operator, who stands where the aircraft can be caught.

    python3 hardware/neo2_hover.py            # Wi-Fi, ground check, wait (20 s), "hover" to fly, summary
    python3 hardware/neo2_hover.py --ground   # Wi-Fi and the ground check only: the motors never start

Steps, stopping at the first one that is not right:

1. nobody else is flying from this box (no flight host is running);
2. the box joins the aircraft's Wi-Fi (``DJI-NEO2-CDF6`` on ``wlp5s0``) and 192.168.2.1 routes through it;
3. GROUND CHECK, ``--preflight-only``: the motors never start. It must end with exit code 0 (the aircraft accepts
   our stick stream) and the battery above the 30 % arm floor;
4. ``--wait`` seconds after power-on, 20 by default (the runbook says 170: a younger pack may refuse the launch with
   NO RC STATE, or hop to 0.5 m and land itself; not a fault, run it again; ``--wait 170`` for the runbook's rule);
5. THE FLIGHT, only after the operator types ``hover``. Its output is shown live;
6. the box leaves the aircraft's Wi-Fi, and the run is summarised from its ``flight.jsonl``.

LANDING. The hover lands by itself after ``--hover-secs``. To land sooner: ONE Ctrl-C in this terminal (neo2_fly
lands on Ctrl-C, SIGTERM and SIGHUP), or a hand underneath the aircraft. This runner then waits for the landing.

Records: the aircraft's run in ``~/tello-runs/hover-<date>/<run_id>/`` (a rundb root), and this session's notes in
``~/neo2-sessions/<date>/hover-*.json``.
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import os
import re
import shlex
import signal
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(os.environ.get("NEO2_REPO", "~/ansr-final/vu-autonomy-neo2")).expanduser()
PY = os.environ.get("NEO2_PY", str(Path("~/venvs/neo2-flight/bin/python").expanduser()))
NMCLI = os.environ.get("NEO2_NMCLI", "sudo -n nmcli")
IP = os.environ.get("NEO2_IP", "ip")
LOGS = Path(os.environ.get("NEO2_SESSION_LOGS", "~/neo2-sessions")).expanduser()
WIFI, IFACE, AIRCRAFT = "DJI-NEO2-CDF6", "wlp5s0", "192.168.2.1"
ALT, MAX_ALT = 1.4, 1.9                 # vu-lab: --alt <= 1.6 and --max-alt 1.9 (the survey tops out at 2.05 m)
FLIGHT_HOSTS = ("tools/neo2_closedloop.py", "tools/neo2_fly.py", "tools/fly_scenario.sh")


class Stop(Exception):
    pass


def sh(cmd: str, timeout: float = 60) -> tuple[int, str]:
    p = subprocess.run(["bash", "-c", cmd], capture_output=True, text=True, timeout=timeout)
    return p.returncode, (p.stdout + p.stderr).rstrip("\n")


def step(title: str):
    print(f"\n== {title}", flush=True)


def ask(prompt: str) -> str:
    try:
        return input(f"?? {prompt} ").strip()
    except EOFError:
        raise Stop("no answer (end of input)")


def flight_host_running() -> list[str]:
    """Processes that RUN a flight host: an interpreter (python, or bash for the launcher) whose script argument is
    one. A process that only mentions the path (an editor, less, a grep, a shell command quoting it) does not count."""
    rc, out = sh("ps -eo args=", timeout=20)
    busy = []
    for line in out.splitlines():
        argv = line.split()
        if not argv or not re.match(r"^(python[0-9.]*|bash|sh)$", Path(argv[0]).name):
            continue
        script = next((t for t in argv[1:] if not t.startswith("-")), "")
        if any(script.endswith(h) for h in FLIGHT_HOSTS):
            busy.append(line)
    return busy


def stream(argv: list[str], log: Path) -> tuple[int, str]:
    """Run the host with its output shown live and kept. A Ctrl-C reaches the host (same process group), which lands;
    this runner then keeps waiting for it to finish the landing instead of leaving it."""
    out = []
    with open(log, "w") as f:
        p = subprocess.Popen(argv, cwd=REPO, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        while True:
            try:
                for line in p.stdout:
                    print(f"   | {line}", end="", flush=True)
                    f.write(line)
                    out.append(line)
                p.wait()
                break
            except KeyboardInterrupt:
                print("\n   >> Ctrl-C reached the host: it lands now. Waiting for it (do not press Ctrl-C again).",
                      flush=True)
    return p.returncode, "".join(out)


def run_dir_of(text: str) -> Path | None:
    m = re.findall(r"(?:recording run ->|PREFLIGHT run:)\s*(\S+)", text)
    return Path(m[-1]) if m else None


def wifi_up():
    step(f"Wi-Fi: join {WIFI} on {IFACE}")
    rc, out = sh(f"{NMCLI} connection up {WIFI} ifname {IFACE}", timeout=60)
    print(f"   | {out.splitlines()[-1] if out else ''}")
    rc, con = sh(f"{NMCLI} -g GENERAL.CONNECTION device show {IFACE}")
    rc, route = sh(f"{IP} route get {AIRCRAFT}")
    rc, default = sh(f"{IP} route show default")
    if con.strip() != WIFI or f"dev {IFACE}" not in route:
        raise Stop(f"NOT READY: {IFACE} is on '{con.strip()}', and {AIRCRAFT} routes '{route.strip()}'. Is the "
                   "aircraft on? Wait half a minute and run this again.")
    if IFACE in default:
        raise Stop(f"the default route goes through {IFACE}: the drone network must never be the default. Run "
                   f"'{NMCLI} connection down {WIFI}' and tell Ben or TTJ.")
    print("   ROUTE-OK")


def wifi_down():
    rc, out = sh(f"{NMCLI} connection down {WIFI}", timeout=60)
    print(f"\n== Left the aircraft's Wi-Fi ({'ok' if rc == 0 else out.splitlines()[-1] if out else rc})")


CONTROL_WINDOW_S = 8.0   # the warm replay holds no_rc_state=0 for ~9 s after connect (RUNBOOK-NEO2.md, 2026-09-17)


def control_window_flip(run_dir: Path | None) -> str | None:
    """A ground check whose ONLY fault is the known control-window flip, else None.

    The flip (RUNBOOK-NEO2.md "Take off inside the drone's control window"; HANDOFF-2026-09-18: "sticks accepted,
    then no_rc_state=1 after ~11 s"): no_rc_state is 0 from the start, then turns 1 ONCE, at least
    CONTROL_WINDOW_S after the first telemetry, and stays 1. The ground check's 10 s hold always outlasts the window,
    while a flight takes off inside it (~7 s), with the in-flight NO RC STATE abort as the backstop. Scattered drops,
    a 1 from the start, a flip back to 0, or a flip inside the window are NOT this, and stop the session."""
    f = run_dir / "flight.jsonl" if run_dir else None
    if not f or not f.exists():
        return None
    rows = []
    for line in f.open():
        if '"state"' in line:
            try:
                r = json.loads(line)
            except ValueError:
                continue
            if r.get("kind") == "state" and r.get("no_rc_state") in (0, 1):
                rows.append((r["wall"], r["no_rc_state"]))
    if not rows or rows[0][1] != 0:
        return None
    first_one = next((i for i, (_, v) in enumerate(rows) if v == 1), None)
    if first_one is None or any(v == 0 for _, v in rows[first_one:]):
        return None
    at = rows[first_one][0] - rows[0][0]
    if at < CONTROL_WINDOW_S:
        return None
    return (f"no_rc_state was 0 for the first {at:.1f} s after connect, then 1 to the end of the hold: the known "
            "control-window flip (RUNBOOK-NEO2.md, 2026-09-17). The flight takes off inside the window (~7 s).")


def summarize(run_dir: Path | None) -> dict:
    s = {"run_dir": str(run_dir) if run_dir else None}
    f = run_dir / "flight.jsonl" if run_dir else None
    if not f or not f.exists():
        s["note"] = "no flight.jsonl"
        return s
    rows = []
    for line in f.open():
        if '"state"' in line:
            try:
                r = json.loads(line)
            except ValueError:
                continue
            if r.get("kind") == "state":
                rows.append(r)
    if not rows:
        s["note"] = "no state rows"
        return s
    hs = [r.get("h") for r in rows if isinstance(r.get("h"), (int, float))]
    up = [r["wall"] for r in rows if (r.get("h") or 0) >= 30]            # above 0.3 m
    s.update({"max_height_m": round(max(hs) / 100, 2) if hs else None,
              "above_0.3m_s": round(up[-1] - up[0], 1) if len(up) > 1 else 0.0,
              "battery_start_pct": rows[0].get("bat"), "battery_end_pct": rows[-1].get("bat")})
    return s


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ground", action="store_true", help="Wi-Fi and the ground check only; the motors never start")
    ap.add_argument("--hover-secs", type=float, default=3.0, help="hover time at altitude (neo2_fly's default 3 s)")
    ap.add_argument("--wait", type=float, default=float(os.environ.get("NEO2_WAIT_S", "20")), metavar="S",
                    help="seconds after power-on before take-off (default 20; the runbook's rule is 170)")
    a = ap.parse_args(argv)
    day = dt.date.today().isoformat()
    runs = Path(f"~/tello-runs/hover-{day}").expanduser()
    LOGS.joinpath(day).mkdir(parents=True, exist_ok=True)
    rec_file = LOGS / day / f"hover-{dt.datetime.now():%H%M%S}.json"
    rec = {"day": day, "repo": str(REPO), "runs": str(runs)}
    joined = False
    try:
        step("Checks")
        if not (REPO / "tools/neo2_fly.py").exists() or not Path(PY).exists():
            raise Stop(f"no flight checkout or venv here ({REPO}, {PY}). This runs on the flight box.")
        rc, head = sh(f"git -C {shlex.quote(str(REPO))} log -1 --format='%h %cs'")
        rec["commit"] = head.strip()
        print(f"   flight code: {REPO} at {rec['commit']}")
        busy = flight_host_running()
        if busy:
            raise Stop("a flight host is already running on this box: " + "; ".join(b[:120] for b in busy))
        print("   nobody else is flying from this box")

        print("\n   Aircraft on the floor in the open, fresh pack, nothing within 1.5 m, operator ready to catch.")
        ans = ask("Power-on time? Enter if it was switched on just now, or HH:MM:")
        now = dt.datetime.now()
        power_on = now if not ans else dt.datetime.combine(now.date(), dt.datetime.strptime(ans, "%H:%M").time())
        rec["power_on"] = power_on.isoformat(timespec="seconds")

        wifi_up()
        joined = True

        step("GROUND CHECK (motors never start)")
        base = [PY, "tools/neo2_fly.py", "--link", "wifi", "--warm", "--profile", "hover", "--run-dir", str(runs)]
        rc, out = stream(base + ["--preflight-only"], LOGS / day / "hover-ground.txt")
        rec["ground"] = {"rc": rc, "run_dir": str(run_dir_of(out))}
        if "would be REFUSED" in out:
            raise Stop("the battery is below the 30 % arm floor: swap the pack, then run this again.")
        if rc != 0:
            flip = control_window_flip(run_dir_of(out)) if "sticks accepted" in out else None
            if not flip:
                raise Stop("the ground check did not pass (exit code {}): read its last lines above. Do not fly."
                           .format(rc))
            rec["ground"]["control_window_flip"] = flip
            print(f"   LINK OK, with the known control-window flip: {flip}")
            print("   If the first flight hops to ~0.5 m and lands (AUTO-ABORT: NO RC STATE), run it again: the "
                  "runbook says the second flight has worked every time.")
        print("   GROUND CHECK PASSED")
        if a.ground:
            print("\n--ground: done. Nothing flew.")
            return 0

        while (left := a.wait - (dt.datetime.now() - power_on).total_seconds()) > 0:
            print(f"   .. {left:.0f} s until {a.wait:g} s after power-on", flush=True)
            time.sleep(min(15, left))

        step(f"THE FLIGHT: up to {ALT} m, hover {a.hover_secs:g} s, land (auto-lands above {MAX_ALT} m)")
        print("   TO LAND SOONER: ONE Ctrl-C here, or a hand underneath it.")
        if ask("Clear around it and ready to catch? Type hover to take off (anything else cancels):") != "hover":
            raise Stop("cancelled: nothing flew.")
        t0 = dt.datetime.now()
        rc, out = stream(base + ["--alt", str(ALT), "--max-alt", str(MAX_ALT), "--hover-secs", str(a.hover_secs)],
                         LOGS / day / f"hover-flight-{t0:%H%M%S}.txt")
        rec["flight"] = {"launch": t0.isoformat(timespec="seconds"), "rc": rc, **summarize(run_dir_of(out))}
        return 0 if rc == 0 else 2
    except Stop as e:
        print(f"\nSTOPPED: {e}", flush=True)
        rec["stopped"] = str(e)
        return 2
    finally:
        if joined:
            wifi_down()
        rec_file.write_text(json.dumps(rec, indent=1) + "\n")
        if "flight" in rec:
            f = rec["flight"]
            print(f"\nFLOWN: exit {f['rc']}, max height {f.get('max_height_m')} m, {f.get('above_0.3m_s')} s above "
                  f"0.3 m, battery {f.get('battery_start_pct')} -> {f.get('battery_end_pct')} %")
            print(f"run: {f.get('run_dir')}  (rundb indexes ~/tello-runs; filing it is TTJ's or Ben's step)")
        print(f"notes: {rec_file}")


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal.default_int_handler)
    sys.exit(main())
