# Three Ways to Drive from Point to Point

This project has three ways of driving the robot from (fromX, fromY) to (toX, toY):

1. **`path.java`**: auto-generated code at the repo root.
2. **`master`'s competition autos** (`opmode/FarZoneAuto.java`, on the `master` branch),
   reused in this project by **`AutoPathPedro.java`**.
3. **`AutoPath.java`**: a hand-written PID controller.

All three drive the same idea: move along straight lines between points, facing the
direction of travel. They differ in **who steers** and **when a move counts as finished**.

## Summary

| | 1. `path.java` (generated) | 2. `master` / `AutoPathPedro` | 3. `AutoPath` (PID) |
|---|---|---|---|
| Builds in this project | **No**: written for a newer Pedro API | Yes | Yes |
| Who steers | Pedro `Follower` | Pedro `Follower` (tuned constants) | Hand-written distance + heading PID |
| Path shape | One `Path` per move | One `PathChain` per move (`master` also chains several segments) | Straight legs, no path objects |
| Heading | `tangent()` | `setTangentHeadingInterpolation()` | Computed `atan2(dy, dx)` and held by a PID |
| Big heading changes | Turns while driving | Turns while driving | Turns **in place** first, then drives |
| A move ends when | Pedro's full end-of-path check (settled, or about 100 ms after reaching the end) | **Parametric end** (t ≥ 0.91) or timeout | Within 0.5 in **and** stopped **and** within 2°, or timeout |
| Pause at each point | Short | **None**: next move starts on arrival | Long |
| Accuracy at each point | High | Lower: hands off before fully stopping | Highest, when tuned |
| Commands | ivy `sequential(follow, ...)` | ivy `follow(...).raceWith(...)` | Plain `while` loops |

---

## 1. `path.java`: generated

```java
private final Pose start = poseFactory.of(55.4, 0, 90);
private final Pose path1 = poseFactory.of(56, 26, 88.678);
...
public Command autoRoutine() {
    return sequential(
        follow(follower, path1()),
        follow(follower, path2()),
        ...
    );
}
public Path path1() { return Paths.line(start, path1).tangent(); }
```

**How it drives:** each move is its own `Path`, wrapped in its own `follow(...)` and run in
`sequential(...)`. Each `follow` runs to completion, so the robot brakes and settles at
**every** point before the next move starts.

**It doesn't build here.** It targets a newer Pedro Pathing API than the 2.1.2 this project
uses:

| In `path.java` | In Pedro 2.1.2 |
|---|---|
| `com.pedropathing.api.Paths`, `Paths.line(a, b).tangent()` | `new BezierLine(a, b)` + `setTangentHeadingInterpolation()` |
| `PoseFactory.degrees().of(x, y, deg)` | `new Pose(x, y, Math.toRadians(deg))` |
| `com.pedropathing.math.Pose`, `pose().x()` | `com.pedropathing.geometry.Pose`, `getPose().getX()` |
| `Constants.create(hardwareMap)` | `Constants.createFollower(hardwareMap)` |
| `follower.distanceToEndpoint()`, `pathIndex()` | not available |

**Only the start heading is used.** `tangent()` computes the heading from the line, so the
headings on the other points (`88.678`, `-101.3099`, ...) are ignored. They're the
generator's notes of the resulting tangent, and all match `atan2(dy, dx)`.

## 2. `master`: Pedro paths that advance on arrival

From `opmode/FarZoneAuto.java` on the `master` branch:

```java
rowCollect = robot.drivetrain.follower.pathBuilder()
        .addPath(new BezierCurve(point(63.74, 8.235), point(63.74, 35.5), point(40.5, 35.5)))
        .setTangentHeadingInterpolation()
        .addPath(new BezierLine(point(40.5, 35.5), point(11.5, 35.5)))
        .setTangentHeadingInterpolation()
        .build();

private Command followWithTimeout(PathChain path, long timeoutMs, double maxPower) {
    return robot.drivetrain.followPath(path, maxPower)
            .raceWith(waitUntil(() -> robot.drivetrain.follower.atParametricEnd()))
            .raceWith(waitMs(timeoutMs));
}
```

**How it drives:**

1. **Build** a `PathChain` from `BezierLine`s (straight) and `BezierCurve`s (curved), with a
   heading mode per segment: tangent, constant (`setConstantHeadingInterpolation`) or
   reversed (`setReversed`).
2. **Follow** it with Pedro's `Follower`, wrapped as an ivy command.
3. **Update** the follower every loop. On each update, Pedro finds the closest point on the
   path and adds drive, translational, heading and centripetal corrections, using the tuned
   values in `pedroPathing/Constants.java`.
4. **End the move** with a race: the path finishing normally, **`atParametricEnd()`**, or a
   timeout, whichever comes first.

**Why it doesn't pause:** `atParametricEnd()` turns true once the robot is geometrically at
the end of the path (t ≥ `tValueConstraint` = 0.91), even if it's still moving. The next
move starts immediately instead of waiting for the robot to settle. Where `master` wants a
pause, it adds one explicitly, for example `waitMs(ROW_SWEEP_END_WAIT_MS)`.

**Driving through points:** segments inside **one** `PathChain` are driven without
stopping, because Pedro only brakes on the chain's last segment. `master` uses this for
smooth runs such as approach-then-sweep, and separate chains where it wants a hand-off.

### Trade-offs to know

- **Hand-off happens early.** t ≥ 0.91 means 9% of a move's length is still left: about
  9 in on a 103 in move, 0.5 in on a 5 in move. The robot is still moving when the next move
  starts, so it **rounds corners** instead of hitting each point exactly.
- **Turns happen while driving.** A big heading change at the start of a move, such as 170°,
  is made by Pedro's heading PIDF while the robot translates. The path tracks well, but the
  robot spends the start of the move rotating.
- **It relies on tuning.** Quality depends entirely on the `FollowerConstants`,
  `PathConstraints` and predictive braking values in `pedroPathing/Constants.java`.

### `AutoPathPedro.java`

`AutoPathPedro` (Driver Station name **AutoPath_Pedro**) runs `path.java`'s 8 moves in
`master`'s style:

- Each move is a one-segment `PathChain`: a `BezierLine` with tangent heading.
- Each move is followed with the same `followWithTimeout` race (parametric end or 5 s).
- After the last move, the follower keeps holding the final point.
- It's a standalone `LinearOpMode` that only uses the drive and odometry. It needs the ivy
  command library (`com.pedropathing:ivy`), which `build.dependencies.gradle` includes.
- The Driver Station shows live x / y and heading during INIT and while running. After each
  move, a line is added: target, actual position, heading, how far off, `arrived` or
  `TIMEOUT`, and time.

## 3. `AutoPath.java`: hand-written PID

See [autopath.md](autopath.md) for its route and tuning.

**How it drives:**

- For each leg, if the heading must change by more than 15°, it **turns in place** first,
  using a PID with a 0.1 s settle window.
- It then drives the leg with a **distance PID** (P + I + D + static-friction F). Power is
  split into x and y parts pointing at the target, and the heading is held.
- A leg ends only when the robot is **within 0.5 in, nearly stopped, and within 2°**, or after
  5 s.

**Why it pauses:** the end condition demands a full stop on target. Any shortfall (stalling
just outside 0.5 in, a noisy speed reading, a turn stuck 2–3° short) keeps the leg running
until the safety limit runs out. It's also untuned: its gains are estimates, not
measurements.

---

## Which to use

| Goal | Use |
|---|---|
| Fast autonomous that flows from move to move, like the competition autos | **2**: `AutoPathPedro` / `master` style |
| Stop precisely on every point | **1**: `path.java`'s style (one `follow` per move, full settle), translated to Pedro 2.1.2 |
| Learning how PID works, or driving without Pedro's path follower | **3**: `AutoPath` |

To flow through points that don't need a hand-off, put them in **one** `PathChain` with
several `addPath(...)` calls, instead of one chain per move.
