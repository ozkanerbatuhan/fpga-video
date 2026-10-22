# Progress (resume point)

| Phase | State | Evidence |
|---|---|---|
| F0 discovery | camera not reachable/unknown -> synthetic clips (tools/gen_clips.py); camera capture script pending model/IP | clips/*.json |
| F1 golden model | done | model/golden.py, docs/media/*_f100.png |
| F2 RTL + sim | done, 18/18 bit-exact, mutation check OK (gauss rounding, CCL union) | sim/last_run.txt |
| F3 Vivado BD | in progress | hw/ |
| F4 hardware | pending (needs board + camera) | |
| F5 packaging | pending | |

Decisions so far: see docs/DECISIONS.md.
Run all tests: `python sim/run_sim.py`.
