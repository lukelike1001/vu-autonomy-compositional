# Hardware: Neo 2 runners

The runners, run ON the flight box (Taylor's Alienware, account `verivital`), by the operator:

| runner | flies | needs |
|---|---|---|
| `neo2_hover.py` | **the simplest flight**: take off, climb to 1.4 m, hover 3 s, land. No truck, no tags | one charged Neo 2, one person ready to catch |
| `neo2_nsbt.py` | **a BehaVerify tree flying the drone**: an example's NSBT, one cell per tick, fixed heading, on the lab's closed-loop host | one charged Neo 2, two people (operator, catcher) |

## From your laptop: `a51.sh`

```
hardware/a51.sh ground      # the ground check, run on the Alienware, shown here
hardware/a51.sh hover       # the flight (you type "hover")
hardware/a51.sh last        # print the newest saved log, to copy and paste
hardware/a51.sh             # a shell on the Alienware;   hardware/a51.sh sync   copies the runners there
```

Everything it runs is shown live and saved in `hardware/logs/` on the laptop (git-ignored). Flying from the laptop is
fine when you stand by the drone: one Ctrl-C lands it, and a dropped connection lands it too (SIGHUP).

## `neo2_hover.py`: one drone, up and down

```
cd ~/vu-autonomy-compositional
python3 hardware/neo2_hover.py --ground     # first: Wi-Fi + ground check, the motors never start
python3 hardware/neo2_hover.py              # the flight: type "hover" when ready
```

It runs `tools/neo2_fly.py --profile hover` from `~/ansr-final/vu-autonomy-neo2` with `~/venvs/neo2-flight`, with the
vu-lab limits from the Neo 2 runbook (`--alt 1.4 --max-alt 1.9`). In order, stopping at the first problem:

1. it checks that nobody is flying from this box;
2. it joins `DJI-NEO2-CDF6` on `wlp5s0` and checks that 192.168.2.1 routes through it (`ROUTE-OK`);
3. **ground check** (`--preflight-only`, motors never start): exit 0, and the battery above the 30 % arm floor;
4. it waits `--wait` seconds after power-on (20 by default; the runbook says 170, see `--wait 170`);
5. **the flight**, only when you type `hover`;
6. it leaves the drone's Wi-Fi (always, even after a stop) and summarises the run (max height, time above 0.3 m,
   battery).

**To land sooner: ONE Ctrl-C in its terminal** (neo2_fly lands on Ctrl-C, SIGTERM and SIGHUP; the runner then waits
for the landing), or a hand underneath it. The run is recorded in `~/tello-runs/hover-<date>/`, which rundb indexes.
Filing it to rundb is TTJ's or Ben's step. As of 2026-10-13, 78 closed-loop runs have flown from this box but no
`neo2_fly` hover has, so the first `--ground` is also the first test of this host from here.

Tests: `python3 -m pytest -q hardware/tests/test_neo2_hover.py` (a fake `neo2_fly.py`, `nmcli` and `ip`; 8 tests).

## `neo2_nsbt.py`: an NSBT flies the Neo 2

```
cd ~/vu-autonomy-compositional
python3 hardware/neo2_nsbt.py --dry        # first: the host's dry run (aircraft on the pad, motors never start)
python3 hardware/neo2_nsbt.py              # the flight: type "nsbt" when ready
```

Default example: `examples/room1_nn1` (one node, NN1-fly; from the Neo 2 pad, cell (8, 15), `We So So So` to (7, 12), at 1.55 m). How the tree
reaches the aircraft:

1. **The tree runs on the flight box.** BehaVerify (`~/venvs/nsbt-bv`) generates the example's tree and runs it from
   every start cell (`examples/route_table.py`). The table must equal the committed `routes.json`, the one the proofs
   and the simulator runs used, or the runner stops.
2. **The lab's host flies it.** `nsbt_flight.py host` runs `tools/neo2_closedloop.py live --mode square` from
   `~/ansr-final/vu-autonomy-neo2` in `~/venvs/neo2-flight`, both unchanged. Only its two pattern functions are
   replaced, by the route for the cell of the aircraft's first position fix. Everything else is the host's own: warm,
   take-off, the tag localizer, `--yaw-mode fixed` (it never turns), the 0.60 m clearance gate and the tag-visibility
   gate (a refusal lands), the stale-pose hold and abort, stall handling, the recording and the landing. The
   lab-specific options are those of the lab's own S19 line (`--site-id vu-lab`, the v6.2 survey's tag map and
   cloud), with `--alt 1.55 --max-alt 2.2 --max-s 90`.
3. **A fix outside the table** (not on a flyable cell): a one-point pattern at the fix, so it holds there and lands.

Afterwards it leaves the drone's Wi-Fi and lists the cells the aircraft's position passed through, against the tree's
route. **To land sooner: ONE Ctrl-C**, or a hand underneath it. Runs land in `~/tello-runs/nsbt-<date>/`.

Checked before the first flight (2026-10-14): the lab's simulator flies every one of the 22 routes through the host's
own `run_closed_loop` (`nsbt_flight.py sitl`), the host accepts the complete armed command line up to the point of
connecting (a smoke test), and `neo2_nsbt.py` runs on the flight box up to the Wi-Fi join. **Not yet run with the
aircraft:** the dry run and the flight.

Tests: `python3 -m pytest -q hardware/tests/test_nsbt_flight.py` (the table, the substitution, the summary; 6 tests).
