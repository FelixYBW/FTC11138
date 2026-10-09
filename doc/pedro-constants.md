# Defining Pedro Pathing Constants

All robot-specific settings for Pedro Pathing (v2.1.2) live in
[`TeamCode/.../pedroPathing/Constants.java`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/Constants.java).
Four objects describe the robot, and `createFollower()` combines them into a `Follower`:

```java
public static Follower createFollower(HardwareMap hardwareMap) {
    return new FollowerBuilder(followerConstants, hardwareMap)
            .mecanumDrivetrain(driveConstants)       // MecanumConstants
            .pinpointLocalizer(localizerConstants)   // PinpointConstants
            .pathConstraints(pathConstraints)        // PathConstraints
            .build();
}
```

| Object | Answers the question | Used by `DriveForwardOneMeter`? |
|---|---|---|
| `MecanumConstants` | Which motors are there, which way do they spin, how fast does the robot go? | **Yes**: every drive command goes through it |
| `PinpointConstants` | Where are the odometry pods and how do I read them? | **Yes**: provides the X / Y / heading the PID uses |
| `FollowerConstants` | How hard should the path follower correct errors? | No, only when following Pedro paths |
| `PathConstraints` | When is a path considered finished? | No, only when following Pedro paths |

`DriveForwardOneMeter` runs its own distance PID and drives through
`follower.setTeleOpDrive(...)`. Pedro's path-following controllers are bypassed, so
`FollowerConstants` and `PathConstraints` have no effect on it. They take effect as soon
as an OpMode calls `follower.followPath(...)`.

Every class uses a builder style: each setter returns `this`, so calls chain. Anything you
don't set keeps its default. The constructor of each class calls `defaults()`, so the
defaults listed here come from `defaults()`; some field initializers in the source differ.

**Units:** distances are in **inches**, velocities in **in/s**, accelerations in
**in/s²**, angles in **radians**, and powers range from **0 to 1**.

---

## 1. Order of definition and tuning

Define and tune the objects in this order. Each step depends on the one before it.

1. **`MecanumConstants`, motors.** Set names and directions so every wheel spins forward when told to drive forward.
2. **`PinpointConstants`, localization.** Set pod offsets and encoder directions so the pose reads correctly when you push the robot by hand.
3. **`MecanumConstants`, velocities.** Measure `xVelocity` and `yVelocity` with the tuners.
4. **`FollowerConstants`.** Set mass, zero-power accelerations, then the PIDF and braking gains.
5. **`PathConstraints`.** Set end-of-path tolerances.

The Pedro tuning OpModes (`Tuning.java`) were deleted from this repo in commit `7a66024`. To
re-tune, restore them with `git show 8a7f7f0:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/Tuning.java`
or copy them from the [Pedro Pathing quickstart](https://github.com/Pedro-Pathing/Quickstart).

---

## 2. `MecanumConstants`: the drivetrain

Package `com.pedropathing.ftc.drivetrains`. This describes the four mecanum motors.

### Current definition

```java
public static MecanumConstants driveConstants = new MecanumConstants()
        .maxPower(1)
        .leftFrontMotorName("frontLeft")
        .leftRearMotorName("backLeft")
        .rightFrontMotorName("frontRight")
        .rightRearMotorName("backRight")
        .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
        .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
        .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
        .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
        .xVelocity(86.8)
        .yVelocity(54.5);
```

### Settings

| Setter | Default | Meaning |
|---|---|---|
| `leftFrontMotorName(String)` | `"leftFront"` | Motor name in the Driver Station robot configuration. Must match exactly. |
| `leftRearMotorName(String)` | `"leftRear"` | Same as above. |
| `rightFrontMotorName(String)` | `"rightFront"` | Same as above. |
| `rightRearMotorName(String)` | `"rightRear"` | Same as above. |
| `leftFrontMotorDirection(Direction)` | `REVERSE` | `FORWARD` or `REVERSE`. Choose so a positive power drives the wheel **forward**. |
| `leftRearMotorDirection(Direction)` | `REVERSE` | Same as above. |
| `rightFrontMotorDirection(Direction)` | `FORWARD` | Same as above. |
| `rightRearMotorDirection(Direction)` | `FORWARD` | Same as above. |
| `xVelocity(double)` | `81.34` | Top **forward** speed at full power, in/s. Measured by the *Forward Velocity Tuner*. |
| `yVelocity(double)` | `65.43` | Top **sideways** (strafe) speed at full power, in/s. Measured by the *Lateral Velocity Tuner*. |
| `maxPower(double)` | `1` | Cap on any wheel power, 0 to 1. Lower it to slow the whole robot down. |
| `useBrakeModeInTeleOp(boolean)` | `false` | Whether motors brake (instead of coast) at zero power in teleop drive. `startTeleopDrive(true)` overrides this. |
| `useVoltageCompensation(boolean)` | `false` | Scales wheel powers by battery voltage so behavior stays consistent as the battery drains. |
| `nominalVoltage(double)` | `12.0` | The voltage that voltage compensation normalizes to. |
| `staticFrictionCoefficient(double)` | `0.1` | Friction term used in the voltage-compensation formula. |
| `motorCachingThreshold(double)` | `0.01` | A motor's power is only re-sent when it changes by more than this. This reduces hardware traffic and speeds up loops. |

### How `xVelocity` and `yVelocity` are used

Mecanum wheels push at an angle, and a robot usually strafes slower than it drives
forward. Pedro turns the two speeds into the direction a front-left wheel actually pushes
(`frontLeftVector = normalize(xVelocity, -yVelocity)`) and uses it to mix wheel powers.
Wrong values make diagonal and sideways motion come out skewed, even when straight-ahead
driving looks fine.

### Getting motor directions right

Run a test that sets each motor to a small positive power, one at a time. Every wheel
should roll the robot **forward**. Flip any wheel that doesn't. Fix directions **before**
tuning anything else.

> **Check this robot:** the right-front motor is `REVERSE` while the right-rear is
> `FORWARD`. Usually both motors on one side share a direction. This can be correct if one
> motor is mounted or geared differently, but confirm it with the one-motor-at-a-time test.

---

## 3. `PinpointConstants`: the odometry

Package `com.pedropathing.ftc.localization.constants`. This describes the goBILDA Pinpoint
computer and its two dead-wheel pods. It is the source of `follower.getPose()`.

### Current definition

```java
public static PinpointConstants localizerConstants = new PinpointConstants()
        .forwardPodY(132.0 / 25.4)      // 132 mm -> inches
        .strafePodX(-24 / 25.4)         // -24 mm -> inches
        .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
        .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
        .distanceUnit(DistanceUnit.INCH)
        .hardwareMapName("pinpoint")
        .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
```

### Settings

| Setter | Default | Meaning |
|---|---|---|
| `hardwareMapName(String)` | `"pinpoint"` | Pinpoint device name in the robot configuration (an I2C device). |
| `distanceUnit(DistanceUnit)` | `INCH` | Unit for the two offsets below. Keep `INCH` to match the rest of Pedro. |
| `forwardPodY(double)` | `1` | **Sideways** offset of the *forward-measuring* pod from the robot's center of rotation. **Left is positive**, right is negative. |
| `strafePodX(double)` | `-2.5` | **Forward/back** offset of the *strafe-measuring* pod from the center of rotation. **Forward is positive**, back is negative. |
| `forwardEncoderDirection(EncoderDirection)` | `REVERSED` | `FORWARD` or `REVERSED`. Choose so pushing the robot forward makes **X increase**. |
| `strafeEncoderDirection(EncoderDirection)` | `FORWARD` | Choose so pushing the robot left makes **Y increase**. |
| `encoderResolution(GoBildaOdometryPods)` | `goBILDA_4_BAR_POD` | Pod type: `goBILDA_4_BAR_POD` or `goBILDA_SWINGARM_POD`. |
| `customEncoderResolution(double)` | *unset* | Ticks per mm, for non-goBILDA pods. Overrides `encoderResolution`. |
| `yawScalar(double)` | *unset* | Overrides the Pinpoint's factory heading calibration. Only set this if heading drifts after many full turns. |

### Offset naming

The names come from what each offset is measured along, which is easy to mix up:

- The **forward pod** rolls when the robot drives forward. Its offset is measured
  **sideways** (Y), hence `forwardPodY`.
- The **strafe pod** rolls when the robot strafes. Its offset is measured
  **forward/back** (X), hence `strafePodX`.

Measure from the robot's **center of rotation** (usually the center of the drive wheels),
not the frame's corner. Wrong offsets make the pose drift sideways whenever the robot turns
in place.

### Verifying localization

Push the robot by hand with the *Localization Test* running:

| Push | Expected |
|---|---|
| Forward | X increases |
| Left | Y increases |
| Rotate counter-clockwise | Heading increases |
| Spin 360° in place | X and Y return near their start |

### Coordinate frame (important for `DriveForwardOneMeter`)

Pedro's frame: **+X is the robot's forward when heading = 0**, **+Y is 90° to the left**,
and **heading is counter-clockwise positive** in radians. A positive `turn` passed to
`setTeleOpDrive` therefore rotates counter-clockwise. `DriveForwardOneMeter` starts with
heading 90° (`Math.toRadians(90)`), so "ahead" is +Y and the target `(0, 39)` is straight
in front of the robot.

---

## 4. `FollowerConstants`: path-following control

Package `com.pedropathing.follower`. These gains control how the `Follower` corrects
itself while following a path. They are **not used** by the teleop-drive PID in
`DriveForwardOneMeter`.

### Current definition

```java
public static FollowerConstants followerConstants = new FollowerConstants()
        .mass(14.7)
        .centripetalScaling(0)
        .headingPIDFCoefficients(new PIDFCoefficients(1.2, 0.0004, 0.06, 0.03))
        .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1.6, 0, 0.14, 0.026))
        .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.13, 0.061, 0.00195));
```

### How the Follower combines corrections

On each `follower.update()`, the Follower adds up to four correction vectors:

1. **Translational**: pushes the robot back onto the path line.
2. **Heading**: rotates the robot toward the target heading.
3. **Drive**: controls speed along the path and slows down toward the end.
4. **Centripetal**: leans into curves so the robot doesn't swing wide.

Each has its own coefficients below.

### Coefficient types

```java
new PIDFCoefficients(p, i, d, f)
new FilteredPIDFCoefficients(p, i, d, t, f)   // t = low-pass filter time constant on D
new PredictiveBrakingCoefficients(p, linearBraking, quadraticFriction)
```

- **P (proportional):** power proportional to the error. This is the main gain.
- **I (integral):** builds up while error persists and removes small steady offsets. Keep it tiny or 0.
- **D (derivative):** reacts to how fast the error changes. It damps overshoot and oscillation.
- **F (feedforward):** a constant push in the direction of the error that overcomes static friction.

### Primary and secondary controllers

Translational, heading and drive each support a **secondary** PIDF that takes over when
the error is small. A strong, damped secondary loop gives accurate final positioning
without making large corrections violent. **Setting a `secondary...Coefficients(...)`
automatically enables it.** The switch point is set by the matching `...PIDFSwitch`.

### Settings

#### Robot model

| Setter | Default | Meaning |
|---|---|---|
| `mass(double)` | `10.65` | Robot mass in **kg**, used only for the centripetal correction. This robot is 14.7 kg. |
| `forwardZeroPowerAcceleration(double)` | `-41.28` | Deceleration in in/s² when power is cut while driving forward. **Must be negative.** Measured by the *Forward Zero Power Acceleration Tuner*. |
| `lateralZeroPowerAcceleration(double)` | `-59.78` | Same, for strafing. **Must be negative.** Measured by the *Lateral Zero Power Acceleration Tuner*. |
| `centripetalScaling(double)` | `0.0005` | Strength of the centripetal correction on curves. This robot sets **0** (off). |

#### Translational (stay on the path line)

| Setter | Default | Meaning |
|---|---|---|
| `translationalPIDFCoefficients(PIDFCoefficients)` | `(0.1, 0, 0, 0)` | Main translational PIDF. |
| `secondaryTranslationalPIDFCoefficients(PIDFCoefficients)` | `(0.3, 0, 0.01, 0.015)`, off | Used when the error is below `translationalPIDFSwitch`. |
| `translationalPIDFSwitch(double)` | `3` | Error in **inches** below which the secondary loop takes over. |
| `translationalIntegral(PIDFCoefficients)` | `(0, 0, 0, 0.015)` | Separate integral term for the translational loop. |

#### Heading (face the right way)

| Setter | Default | Meaning |
|---|---|---|
| `headingPIDFCoefficients(PIDFCoefficients)` | `(1, 0, 0, 0.01)` | Main heading PIDF. Error is in radians. This robot uses `(1.2, 0.0004, 0.06, 0.03)`. |
| `secondaryHeadingPIDFCoefficients(PIDFCoefficients)` | `(5, 0, 0.08, 0.01)`, off | Used when the heading error is below `headingPIDFSwitch`. This robot uses `(1.6, 0, 0.14, 0.026)`. |
| `headingPIDFSwitch(double)` | `π/20` (9°) | Heading error in **radians** below which the secondary loop takes over. |
| `turnHeadingErrorThreshold(double)` | `0.01` | Radians. `follower.turn()` and `turnTo()` count as done inside this error. |

#### Drive (speed along the path and braking)

| Setter | Default | Meaning |
|---|---|---|
| `drivePIDFCoefficients(FilteredPIDFCoefficients)` | `(0.025, 0, 0.00001, 0.6, 0.01)` | Main drive PIDF on the remaining distance. |
| `secondaryDrivePIDFCoefficients(FilteredPIDFCoefficients)` | `(0.02, 0, 0.000005, 0.6, 0.01)`, off | Used when the drive error is below `drivePIDFSwitch`. |
| `drivePIDFSwitch(double)` | `20` | Drive error in **inches** below which the secondary loop takes over. |
| `predictiveBrakingCoefficients(PredictiveBrakingCoefficients)` | `(0.15, 0.1, 0.001)`, off | **Replaces the drive PIDF** with a model that predicts stopping distance from current speed. Setting it enables it. This robot uses `(0.13, 0.061, 0.00195)`. |

`PredictiveBrakingCoefficients(p, linearBraking, quadraticFriction)`:

- `linearBraking` and `quadraticFriction` describe how the robot naturally slows down.
  They are measured automatically by the *Predictive Braking Tuner*.
- `p` controls how hard the robot reacts to the remaining distance after subtracting the
  predicted stopping distance. Tune it with the *Line Test*: as high as possible without
  jitter, typically 0.1 to 0.3.
- `.withMaximumBrakingPower(x)` caps the reverse braking power (default 0.2). Too high
  causes voltage dips that can reboot the Control Hub.

When predictive braking is on, it also drives the translational correction once the robot reaches the end of a path,
and `drivePIDF...` settings are ignored.

#### Holding the end point

| Setter | Default | Meaning |
|---|---|---|
| `automaticHoldEnd(boolean)` | `true` | After a path finishes, keep actively holding the final pose. |
| `holdPointTranslationalScaling(double)` | `0.45` | Scales translational correction while holding a point. |
| `holdPointHeadingScaling(double)` | `0.35` | Scales heading correction while holding a point. |

#### Stuck detection

When the robot isn't moving, partway through a path (for example, pushing against a wall),
the path is ended after a timeout instead of waiting forever.

| Setter | Default | Meaning |
|---|---|---|
| `stuckVelocity(double)` | `1.0` | in/s. Below this, the robot may be stuck. |
| `stuckTValueLow(double)` | `0.1` | Ignore "stuck" before this fraction of the path (still accelerating). |
| `stuckTValueHigh(double)` | `0.8` | Ignore "stuck" after this fraction of the path (decelerating normally). |
| `stuckTimeout(double)` | `500` | Milliseconds stuck before the path is ended. |

#### Advanced

| Setter | Default | Meaning |
|---|---|---|
| `driveKalmanFilterModelCovariance(double)` | `6` | Kalman filter on drive error. Higher means faster response, lower means smoother. |
| `driveKalmanFilterDataCovariance(double)` | `1` | Higher ignores noisy measurements more; lower trusts them more. |

### Tuning order

1. `mass`, then the zero-power accelerations, using their tuners.
2. **Translational** PIDF: push the robot sideways off a straight path and raise P until it returns quickly; add D if it oscillates.
3. **Heading** PIDF: rotate the robot by hand mid-path, same procedure.
4. **Drive**: either run the Predictive Braking Tuner and adjust `p` (what this robot does) or tune `drivePIDFCoefficients` on a long straight line.
5. **Centripetal**: on curves, raise `centripetalScaling` if the robot swings wide; lower it if it cuts inside.
6. Add secondary controllers only if final accuracy needs it.

---

## 5. `PathConstraints`: when is a path done?

Package `com.pedropathing.paths`. These rules decide when `follower.isBusy()` turns false.
They are the default for every path; individual paths can override them.

### Current definition

```java
public static PathConstraints pathConstraints = new PathConstraints(
        0.91,   // tValueConstraint
        0.3,    // velocityConstraint (in/s)
        0.3,    // translationalConstraint (in)
        0.02,   // headingConstraint (rad, ~1.1°)
        100,    // timeoutConstraint (ms)
        1.25,   // brakingStrength
        10,     // BEZIER_CURVE_SEARCH_LIMIT
        1       // brakingStart
);
```

### Constructors

The 8-argument constructor takes positional arguments, so order matters. Keep the
comments.

```java
new PathConstraints(tValue, velocity, translational, heading, timeout, brakingStrength, searchLimit, brakingStart)
new PathConstraints(tValue, timeout, brakingStrength, brakingStart)   // others use defaults
new PathConstraints(tValue, timeout)                                  // brakingStrength = 1, brakingStart = 1
```

### Settings

| Parameter | Default | Meaning |
|---|---|---|
| `tValueConstraint` | `0.995` | The path's progress parameter *t* runs from 0 (start) to 1 (end). Past this value, the path counts as *parametrically at its end*. This robot uses `0.91`, which ends earlier so it settles faster. |
| `velocityConstraint` | `0.1` | in/s. At the end, the robot must be slower than this. |
| `translationalConstraint` | `0.1` | inches. At the end, the robot must be closer than this to the final point. |
| `headingConstraint` | `0.007` | radians (~0.4°). At the end, heading error must be below this. |
| `timeoutConstraint` | `100` | ms. Once at the parametric end, the follower gets this long to meet the constraints above. After that, the path ends anyway. |
| `brakingStrength` | `1` | Multiplier on deceleration at the end of paths. Higher stops harder (risk of overshoot or wheel slip); lower stops gentler but slower. This robot uses `1.25`. |
| `BEZIER_CURVE_SEARCH_LIMIT` | `10` | Iterations when searching for the closest point on a curve. Higher is more accurate but slower. |
| `brakingStart` | `1` | Multiplier for how early deceleration begins. Raise it to start braking sooner. |

A path finishes when it is parametrically at its end **and** it meets the velocity,
translational and heading constraints, **or** the timeout expires.

### Trade-off

Tight constraints give precise stops but slower cycles, and the robot may hunt around
the end point until the timeout. Loose constraints give faster cycles but less precise
stops. This robot loosened the defaults (0.1 → 0.3 in/s, 0.1 → 0.3 in, 0.007 → 0.02 rad)
to cut settle time.

---

## 6. Quick checklist for a new robot

- [ ] Motor names match the robot configuration exactly.
- [ ] Each motor alone drives its wheel forward at positive power.
- [ ] Pinpoint name matches; pod type is set.
- [ ] Pushing forward increases X; pushing left increases Y; CCW increases heading.
- [ ] Spinning 360° in place returns X/Y near the start (offsets are right).
- [ ] `xVelocity` and `yVelocity` are measured, not left at defaults.
- [ ] `mass` and zero-power accelerations are measured (negative).
- [ ] Translational, then heading, then drive or braking gains are tuned.
- [ ] Path constraints are tight enough for accuracy, loose enough for speed.
