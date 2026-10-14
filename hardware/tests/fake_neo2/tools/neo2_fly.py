"""FAKE neo2_fly.py for testing neo2_hover.py: flies nothing. FAKE_GROUND=fail|lowbat changes the ground check."""
import json, os, sys, time
a = sys.argv[1:]
runs = a[a.index("--run-dir") + 1]
rid = f"neo2-fake-{time.time_ns()}"
d = os.path.join(runs, rid); os.makedirs(d, exist_ok=True)
print(f"recording run -> {d}", flush=True)
if "--preflight-only" in a:
    g = os.environ.get("FAKE_GROUND", "")
    print("pre-flight battery: 87%")
    print(">>> PREFLIGHT ONLY: watching no_rc_state for 10s with sticks streaming")
    if g == "lowbat":
        print("   !! BATTERY: 22% is below the 30% arm floor\n   !! This preflight ran to completion, but an ARMING run would be REFUSED at this reading")
    print(f"   no_rc_state over the hold: 0 in 100/100 -> {'CHECK: see counts' if g == 'fail' else 'GATE WOULD PASS and the stream HOLDS it'}")
    if g.startswith("flip"):
        at = 4.0 if g == "flip_early" else 11.5          # 2026-10-13 on the a51: 0 for 11.5 s, then 1 to the end
        t = time.time()
        with open(os.path.join(d, "flight.jsonl"), "w") as f:
            for i in range(143):
                f.write(json.dumps({"kind": "state", "wall": t + i / 10, "h": 0, "bat": 87,
                                    "no_rc_state": 0 if i / 10 < at else 1}) + "\n")
        print("   sticks: sticks accepted (no_rc_state=0 for 4.2s) (t+11.0s)")
        print("   no_rc_state over the hold: 0 in 72/100, 1 in 28/100, stale/missing 0/100 -> CHECK: see counts")
    print(f"\nPREFLIGHT run: {d}")
    sys.exit(2 if g == "fail" or g.startswith("flip") else 0)
t = time.time()
with open(os.path.join(d, "flight.jsonl"), "w") as f:
    for i, h in enumerate([0, 0, 40, 100, 140, 141, 139, 140, 90, 20, 0]):
        f.write(json.dumps({"kind": "state", "wall": t + i, "h": h, "bat": 87 - (i // 4), "motor_on": 1}) + "\n")
print("takeoff ... climb to 1.4 m ... hover 3 s ... land: confirmed on the ground")
