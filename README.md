# RFR27 chassis optimizer

Code for an optimizer that designs the steel tube frame of a Formula SAE car (the RFR27 new-member chassis
project) to be as stiff in twist as possible for every pound of steel, and for the pipeline that builds and
checks each design in SOLIDWORKS.

**How it works, in two pages:** [Optimizer Explained.pdf](Optimizer%20Explained.pdf)

Best design so far, 013b, scored with the team's torsional-rigidity test in SOLIDWORKS Simulation:
2795 lb·ft/deg at 75.37 lb, **37.09 TR per lb**, against 28.45 for the team's Rev 8 frame.

## What it does
- Keeps the fixed parts (roll hoops, bulkheads, suspension pickups, driver templates) and chooses the rest:
  which tubes to add, where some joints sit and what size each tube is, from the sizes the supplier stocks.
- Maximises TR per lb under a 75.5 lb weight cap, with the FSAE rule structure, driver clearance and
  packaging keep-out zones as hard constraints.
- Scores designs with a fast beam model of the frame (milliseconds per design), then builds the best one in
  SOLIDWORKS: weldments trimmed in weld order, native checks, and the team's TR study as the final score.

## What is in here
| Part | Files in `scripts/` |
|---|---|
| Beam model of the frame and the twist test | `beam_model.py` |
| Candidate tubes, conflicts and rule sizes (ground structure) | `ground_structure.py` |
| Tube choice and sizing: adjoint gains, Woodbury updates, tabu search, integer program | `layout_engine.py` |
| Outer search over joint positions, and search strategies | `param_search.py`, `portfolio.py`, `continuation.py`, `skeleton_sweep.py` |
| FSAE rule structure checks, read from the geometry | `rule_structure.py`, `audit-frame-rules.py` |
| Driver templates, keep-out zones and exact clearance | `packaging.py`, `photo_keepouts.py`, `make-driver-checks.py` |
| Design recipes and the weld-order trim list | `frame_geometry.py` |
| SOLIDWORKS pipeline: build, trim as added, checks, TR study | `chassis_pipeline.py`, `run-native.mjs`, `native/` (C#) |
| Simulation result checks | `validate-chassis-simulation.py`, `deck_solve.py`, `joint_connectivity.py` |
| Tests | `test-*.py` |

## What is not in here
The team's CAD (starter part, hoops, driver models), the input geometry, the packaging-zone data, the design
recipes and the project's working notes are not public. The code shows the method, but it does not run end to
end, tests included, without those inputs and SOLIDWORKS 2026 with Simulation.

## Setup
- Python 3.11: `python -m pip install -r requirements.txt` (add `-r requirements-reports.txt` for report figures).
- For the CAD steps: .NET 8 SDK, Node.js 20 or newer, and SOLIDWORKS 2026 with Simulation.
- `dotnet` and `node` are found on `PATH` or in their default install folders; set `RFR_DOTNET` or `RFR_NODE`
  to override (and `RFR_PYTHON` for `run-optimizer.ps1`).
- Build the native helper with `dotnet build scripts/native/RfrEngineering.csproj`, adding
  `-p:SolidWorksDir=<folder>` if SOLIDWORKS is not installed in the default folder.
