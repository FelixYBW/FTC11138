# Pedro Pathing 3.0 Upgrade (branch `pedro-3`)

This branch moves the project from Pedro Pathing 2.1.2 to **3.0.1**. All four autonomous
OpModes are ported and the project builds. It has **not been run on the robot**, and the new
path-following algorithm (**Foresight**) runs on estimated values until it's tuned. Read
[Before running](#before-running) first.

## What changed

### Dependencies (`build.dependencies.gradle`)

| Before (2.x) | After (3.0) |
|---|---|
| `com.pedropathing:ftc:2.1.2` | `com.pedropathing:revhub:3.0.1` (Pedro's FTC library, renamed in 3.0; pulls in `core:3.0.1`) |
| `com.pedropathing:ivy:1.0.0` | removed: ivy has no Pedro 3 release |
| FTC SDK 11.1.0 | **unchanged** |

The FTC SDK stays at 11.1.0, so **the Driver Station app doesn't need updating**. Pedro's 3.0
Quickstart uses SDK 12.0.0, so I checked compatibility: all 28 SDK methods `revhub` calls for
the mecanum drive and the Pinpoint (motor power, direction and brake mode, and every
`GoBildaPinpointDriver` call) exist in SDK 11.1.

### `Constants.java`

Pedro 3 builds the follower from three parts instead of one builder:

```java
new Follower(
        new PinpointLocalizer(hardwareMap, localizerConfig),   // odometry
        new Mecanum(hardwareMap, drivetrainConfig),            // motors
        new Foresight(foresightConfig));                       // path-following algorithm
```

`createFollower(...)` is renamed `create(...)`, the name Pedro 3's Quickstart and `path.java`
use.

### OpModes

| OpMode | 3.0 changes |
|---|---|
| Drive Forward 1m, AutoPath (PID) | API renames only (table below). The PID logic is unchanged. |
| AutoPath_Pedro | Uses `Paths.line(...).tangent()` and `follower.follow(path)`. The ivy `follow(...).raceWith(...)` hand-off is now a loop with the same rule: a move ends on finish, parametric end, or 5 s. |
| AutoPath_Generated | Now uses **`path.java`'s own code** (`PoseFactory`, `Paths.line(...).tangent()`, `pose().x()`, `distanceToEndpoint()`), since 3.0 is what it was generated for. Only ivy's `sequential(follow(...))` is replaced, by a loop that starts the next path when `!isBusy()`. |

| 2.x | 3.0 |
|---|---|
| `com.pedropathing.geometry.Pose`, `getX()`, `getHeading()` | `com.pedropathing.math.Pose`, `x()`, `heading()` |
| `follower.getPose()`, `follower.getHeading()` | `follower.pose()`, `follower.pose().heading()` |
| `follower.setStartingPose(p)` | `follower.setPose(p)` |
| `follower.startTeleopDrive(true)` | not needed: manual drive brakes by default |
| `follower.setTeleOpDrive(x, y, turn, false)` (field-centric) | `follower.manual(ManualDrive.fieldCentric(x, y, turn, heading))` |
| `pathBuilder().addPath(new BezierLine(a, b)).setTangentHeadingInterpolation().build()` | `Paths.line(a, b).tangent()` |
| `follower.followPath(chain)` | `follower.follow(path)` |

**Conventions are unchanged:** positive strafe is +y (left), positive turn is counter-clockwise.
I checked this in 3.0's `Mecanum` wheel mixing, so the PID heading correction keeps the same
sign.

**`isBusy()` changed meaning slightly.** In 3.0, it stays true until the robot has **settled**
on the end point (within the constraints, or after the timeout), not just until the path ends.
This is why AutoPath_Generated settles at every point, like `path.java`.

## Constant mapping

| 3.0 setting | Value | Source |
|---|---|---|
| Motor names and directions | `frontLeft` … REVERSE / REVERSE / REVERSE / FORWARD | 2.x, unchanged |
| `xPodOffset` / `yPodOffset` | 5.20 / −0.94 in | 2.x `forwardPodY` / `strafePodX` (same `setOffsets(x, y)` call, so odometry is identical) |
| `xPodDirection` / `yPodDirection` | REVERSED / FORWARD | 2.x forward / strafe encoder directions |
| `podType` | 4-bar pod | 2.x |
| `maxAchievableForwardVelocity` / `Strafe` | 86.8 / 54.5 in/s | 2.x `xVelocity` / `yVelocity` |
| `headingFeedback` | `proportional(1.2)` | 2.x heading PIDF P |
| `headingStaticFF` | `staticFeedforward(0.03)` | 2.x heading PIDF F |
| `linearBrakeCoefficients` | `diag(0.061, 0.061)` | forward: 2.x predictive braking. **Strafe: estimate** (reuses forward, which predicts a longer stop) |
| `quadraticBrakeCoefficients` | `diag(0.00195, 0.00195)` | same as above |
| `forwardTranslational` / `strafeTranslational` | `proportional(0.1)` above 2.5 in, `0.3` below | 2.x translational P / secondary P. **Strafe: estimate** (2.x had one gain for both) |
| `parametricTConstraint` | 0.09 | 2.x `tValueConstraint` 0.91 (3.0 ends at t ≥ 1 − value) |
| `velocity` / `translational` / `heading` / `timeout` constraints | 0.3 / 0.3 / 0.02 / 100 | 2.x `PathConstraints` |
| `naturalForwardDeceleration` / `Strafe` | 41.28 / 59.78 | **Estimate**: 2.x never measured these and ran on Pedro's defaults |
| `coast` / `brake` | `proportionalFeedforward(1/86.8)` / `(0.85/86.8)` | **Estimate**: the tuner's formula (kV = 1 ÷ speed per unit power) using `xVelocity` |
| `headingBrakeCoefficients` | `(0.04, 0.005)` | **Estimate**: no 2.x equivalent |

## Before running

1. **Drive Forward 1m and AutoPath only use odometry and motors.** Their PID code doesn't use
   Foresight, so they should behave as they did on 2.x. Test with these first.
2. **AutoPath_Pedro and AutoPath_Generated depend on Foresight**, which runs partly on
   estimates. Expect worse braking and corrections than 2.x until it's tuned. Try them with
   the robot raised, or with plenty of room and a hand on **STOP**.
3. **Tune Foresight** with Pedro 3's tuners: `ForesightTuner`, `MecanumTuner` and
   `PinpointTuner` in the [Pedro Quickstart](https://github.com/Pedro-Pathing/Quickstart).
   They print a ready-made `ForesightConfig` to paste into `Constants.java`.

### The tuning library isn't included

The 3.0 tuners need `com.pedropathing:tuning:1.0.1`, which depends on **Sloth**
(`dev.frozenmilk.sinister`), a runtime code loader that hooks into the FTC app, plus a web
dashboard server. That's a substantial build change, so it's left out of this branch. Adding it
means following the Quickstart's build setup (the `repo.dairy.foundation` repository and its
Sloth configuration).

## What doesn't carry over

- **`path.java` still can't compile as-is.** Everything except its ivy imports is now valid
  Pedro 3 code, but ivy has no Pedro 3 release. AutoPath_Generated is `path.java` with only the
  ivy routine replaced.
- **The 2.x docs** ([pedro-constants.md](pedro-constants.md),
  [pedro-tuning-new-robot.md](pedro-tuning-new-robot.md)) describe 2.x settings and tuners.
  The physical measurements in them (motor directions, pod offsets, velocities, mass) still
  apply; the PIDF, predictive-braking and `PathConstraints` sections don't.
- **`master`'s competition autos** stay on 2.x. They use ivy, so moving them to 3.0 would mean
  replacing the command library.
