"""neo2_hover.py against a FAKE neo2_fly.py, nmcli and ip: it flies nothing and touches no network."""
from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
RUNNER = HERE.parent / "neo2_hover.py"


@pytest.fixture
def env(tmp_path):
    e = {k: v for k, v in os.environ.items() if not k.startswith("FAKE_")}
    e.update({"NEO2_REPO": str(HERE / "fake_neo2"), "NEO2_PY": sys.executable, "NEO2_NMCLI": str(HERE / "fake_nmcli"),
              "NEO2_IP": str(HERE / "fake_ip"), "NEO2_SESSION_LOGS": str(tmp_path / "logs"), "HOME": str(tmp_path),
              "NEO2_WAIT_S": "0", "FAKE_LOG": str(tmp_path / "nmcli.log")})
    return e


def run(env, *args, stdin="", **fake):
    e = {**env, **{f"FAKE_{k.upper()}": v for k, v in fake.items()}}
    p = subprocess.run([sys.executable, str(RUNNER), *args], input=stdin, capture_output=True, text=True, env=e,
                       timeout=60)
    return p.returncode, p.stdout + p.stderr


def record(env):
    return json.loads(sorted(Path(env["NEO2_SESSION_LOGS"]).rglob("hover-*.json"))[-1].read_text())


def flights(env):
    return list(Path(env["HOME"], "tello-runs").rglob("flight.jsonl"))


def test_a_whole_hover(env):
    rc, out = run(env, stdin="\nhover\n")
    assert rc == 0, out
    for want in ("ROUTE-OK", "GROUND CHECK PASSED", "FLOWN: exit 0, max height 1.41 m", "Left the aircraft's Wi-Fi"):
        assert want in out, want
    f = record(env)["flight"]
    assert f["max_height_m"] == 1.41 and f["battery_start_pct"] == 87 and f["above_0.3m_s"] == 6.0
    assert "connection down DJI-NEO2-CDF6" in Path(env["FAKE_LOG"]).read_text()


def test_ground_only_never_flies(env):
    rc, out = run(env, "--ground", stdin="\n")
    assert rc == 0 and "Nothing flew" in out and not flights(env)


def test_no_hover_typed_no_flight(env):
    rc, out = run(env, stdin="\nyes\n")
    assert rc == 2 and "cancelled: nothing flew" in out and not flights(env)


@pytest.mark.parametrize("fake, words", [({"ground": "fail"}, "ground check did not pass"),
                                         ({"ground": "lowbat"}, "below the 30 % arm floor"),
                                         ({"wifi": "fail"}, "NOT READY")])
def test_it_stops_before_the_flight(env, fake, words):
    rc, out = run(env, stdin="\nhover\n", **fake)
    assert rc == 2 and words in out, out
    assert not flights(env)


def test_it_always_leaves_the_drone_wifi(env):
    run(env, stdin="\nyes\n")
    assert "connection down DJI-NEO2-CDF6" in Path(env["FAKE_LOG"]).read_text()


def test_only_a_running_host_counts_as_busy(monkeypatch):
    """The first version matched ANY command line naming tools/neo2_fly.py (found when the shell that wrote this file
    quoted it): an editor or a pager on the file must not stop a session; a python running it must."""
    sys.path.insert(0, str(HERE.parent))
    import neo2_hover as H
    ps = "\n".join(["less tools/neo2_fly.py", "vim /home/verivital/ansr-final/vu-autonomy-neo2/tools/neo2_fly.py",
                    "bash -c cat > tools/neo2_fly.py <<EOF", "grep tools/neo2_closedloop.py",
                    "/home/verivital/venvs/neo2-flight/bin/python -u tools/neo2_closedloop.py live --arm",
                    "bash tools/fly_scenario.sh S19 --search"])
    monkeypatch.setattr(H, "sh", lambda cmd, timeout=60: (0, ps))
    busy = H.flight_host_running()
    assert busy == ["/home/verivital/venvs/neo2-flight/bin/python -u tools/neo2_closedloop.py live --arm",
                    "bash tools/fly_scenario.sh S19 --search"]


def test_the_control_window_flip_is_not_a_stop(env):
    """2026-10-13 on the a51: sticks accepted, no_rc_state 0 for 11.5 s after connect, then 1 to the end."""
    rc, out = run(env, "--ground", stdin="\n", ground="flip")
    assert rc == 0, out
    assert "known control-window flip" in out and "GROUND CHECK PASSED" in out
    assert record(env)["ground"]["control_window_flip"]


def test_a_flip_inside_the_window_still_stops(env):
    rc, out = run(env, "--ground", stdin="\n", ground="flip_early")
    assert rc == 2 and "ground check did not pass" in out
