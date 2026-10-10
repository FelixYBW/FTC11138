# AutoPath

[`AutoPath.java`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/AutoPath.java)
drives the route defined in `path.java`: eight straight segments, each with **tangent**
heading (the robot faces its direction of travel), and a PID stop at every point.
Select it on the Driver Station as **AutoPath** in the Autonomous list.

The same route also runs with Pedro's path follower as **AutoPath_Pedro** and
**AutoPath_Generated**; see [autopath-comparison.md](autopath-comparison.md) for how they differ.

## Route

The robot starts at **(55.4, 0)** facing **90°** (+y). Units are inches; headings are
degrees, counter-clockwise positive. Each heading is the direction from the previous point
to the next, `atan2(dy, dx)`.

| Path | To | Length (in) | Heading | Turn first |
|---|---|---|---|---|
| 1 | (56, 26) | 26.0 | 88.7° | −1° (corrected while driving) |
| 2 | (52, 6) | 20.4 | −101.3° | **170°** in place |
| 3 | (0, 0) | 52.3 | −173.4° | 72° in place |
| 4 | (0, 15) | 15.0 | 90° | 97° in place |
| 5 | (14, 15) | 14.0 | 0° | 90° in place |
| 6 | (14, 118) | 103.0 | 90° | 90° in place |
| 7 | (57, 118) | 43.0 | 0° | 90° in place |
| 8 | (57, 113) | 5.0 | −90° | 90° in place |

To change the route, edit the `POINTS` array and `START_HEADING_DEGREES` at the top of
`AutoPath.java`. Headings are computed from the points, so they stay correct when a point
moves.

## How it runs

- **Turn first, then drive.** If a segment's heading differs from the current heading by
  more than 15° (`TURN_IN_PLACE_RADIANS`), the robot first turns in place, holding its x/y
  position while it turns. Then it drives the segment holding that heading. This keeps it
  facing its direction of travel for the whole segment, as `tangent` asks. Smaller
  differences are corrected while driving.
- **Distance PID with a stop at every point.** Error is the straight-line distance to the
  next point:
  - **P** (`KP` = 1 / 0.5 m): full power until 0.5 m remains, then power drops in
    proportion to the remaining distance.
  - **D** (`KD`): brakes while the robot is closing in quickly, to prevent overshoot.
  - **I** (`KI`): only active in the last 4 in, to push through friction at the end.
  - **F** (`KF` = 0.07): a constant push toward the point while the robot is more than
    0.5 in away. Without it, P alone falls below the power needed to move when the robot is
    about 1 to 2 in out, so it stalls there and waits out the 5-second limit. If the robot
    still stalls short, raise `KF`; if it hunts back and forth around the point, lower it.

  The power is split into x and y parts that point at the target, so sideways drift is
  corrected on the way.
- **When a leg counts as done:** within 0.5 in of the point, nearly stopped (under 1 in/s),
  and within 2° of the heading. A turn ends once the heading has stayed within 2° for 0.1 s
  in a row (`TURN_SETTLE_SECONDS`). If the robot coasts back out of the window, the timer
  restarts, so an overshoot is corrected before the drive starts. If the heading still
  drifts more than 2° at the start of the drive, raise this to 0.2 to 0.25 s.
- **Turn control:** P, plus a small D to prevent overshoot, plus a small constant push
  (`HEADING_KF`) so it doesn't stall a degree short. Turn power is capped at 0.5.
- **Safety limits:** 5 seconds per segment and 3 seconds per turn. If a leg runs out of
  time, the robot moves on to the next one.

## Driver Station display

The position is shown at every stage, from INIT until STOP:

| Stage | Display |
|---|---|
| **INIT** | Live x / y and heading. Push the robot by hand to check the odometry before starting: it should read (55.4, 0) at 90°. |
| **Turning / driving** | Path number and phase (`turn` or `drive`), x / y, heading, distance remaining and power. |
| **After each point** | A line is added and stays on screen for the rest of the run: target, where the robot actually stopped, its heading, and how far off it was. For example `P2 (52, 6) -> (51.8, 6.3) -101.0°, off by 0.4 in`. |
| **Done** | The robot stays stopped and the screen keeps the final position and all the point lines until you press **STOP**. |

During INIT and after the route finishes, "Distance remaining" and "Power" read 0.

## Before running

- **Space:** the route covers x 0–57 and y 0–118 in, about 1.5 m × 3 m. Place the robot at
  (55.4, 0) facing +y, so the whole area is ahead of it and to its left. Coordinates are only
  relative to that start point, so (0, 0) doesn't have to be a field corner.
- **Overshoot on path 6:** it's 103 in, so the robot reaches near top speed (about 87 in/s)
  and has only the last 20 in to slow down. If it overshoots point 6, raise `KD`, or lower
  full power by changing the `1` in `Range.clip(..., 0, 1)` to about `0.7`.
- **Paths 1 and 8:** path 1 starts at 90° but its heading is 88.7°, which is under the 15°
  threshold, so the robot corrects that 1.3° while driving. Path 8 is only 5 in, so it creeps
  at low power after its 90° turn.
- **Time:** the route totals about 279 in of driving plus seven turns, which normally fits
  inside the 30-second autonomous timer. If legs keep hitting their safety limits, it can
  run past 30 seconds (worst case 8 × 5 s + 7 × 3 s = 61 s), and the timer will stop it
  partway. In that case, fix the tuning, or turn the timer off for testing.
- **Hardware config:** motors `frontLeft`, `frontRight`, `backLeft`, `backRight` and odometry
  `pinpoint`, as for every OpMode in this project.
