# Scout improved DWA

Derived from AMSL dwa_planner commit `6387a9660c2064fa0545ac81d55f4bbe7962651f`.
The original repository at `../dwa_planner` (workspace source sibling) is unchanged.
AMSL copyright notices and BSD-3-Clause LICENSE are retained here.

Changes from the baseline:

- All occupied grid cells are used; no angular ray subsampling for map input.
- Each cell's circumscribed disk includes its area, not just its center.
- Exact unicycle trajectories and intermediate footprint motion bounds cover the sweep.
- A conservative rectangle bounds the configured rectangular Scout footprint.
- Collision candidates are excluded before cost comparison, including final yaw.
- Obstacle cost uses a configured 0.15m preferred clearance, not candidate-set min/max normalization.
- Goal progress and heading have fixed scales; steering/steering-change penalties apply when goal bearing is within 45 degrees.
- Slow candidates are not forced to minimum +/- yaw; zero yaw is retained if reachable.
- Goal lifecycle resets in the derivative, and measured yaw overshoot cannot create a reversed sampling window.

These are local planning improvements, not a global route planner. Preferred clearance,
cost weights and guard braking assumptions are software settings, not measured hardware limits.
Known baseline limitations elsewhere remain; this does not certify collision-free operation.

`launch/improved_planner.launch` launches this executable with the current Scout parameters.
`launch/low_speed_planner.launch` still launches the unchanged original for comparison.
