# Finding the Pedro Pathing Constants for a New Robot

This guide walks through **every value** in
[`Constants.java`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/Constants.java):
where it comes from, how to measure or calculate it, and what a sane result looks like.
For what each setting *means*, see [pedro-constants.md](pedro-constants.md).

Each value comes from one of four sources:

| Source | Examples |
|---|---|
| **Robot configuration** on the Driver Station | motor names, Pinpoint name |
| **Hardware datasheet** | pod resolution, motor RPM, wheel size |
| **Ruler / scale** on the physical robot | pod offsets, mass |
| **Tuning OpMode** that drives the robot and computes the value | velocities, decelerations, braking, PIDF gains |

Work through the steps **in order**. Each step relies on the values before it: a wrong
motor direction ruins localization, and wrong localization ruins every measurement after it.

---

## 0. Preparation

### Tools

- Tape measure (inches, or mm and divide by 25.4)
- Painter's tape to mark start lines on the floor
- A scale (bathroom scale is fine)
- At least **10 ft (3 m) of clear, flat floor**, ideally field tiles, since carpet changes friction
- **Fully charged batteries** (above 13 V). Speed and deceleration vary with voltage, so measure on a fresh battery.

### Restore the tuning OpModes

The tuners were removed from this repo in commit `7a66024`. Restore them:

```bash
git show 8a7f7f0:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/Tuning.java \
  > TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/Tuning.java
```

They also need the Panels dashboard libraries, which were removed from `build.dependencies.gradle`. Add these back:

```gradle
implementation 'com.pedropathing:telemetry:1.0.0'
implementation 'com.bylazar:fullpanels:1.0.12'
```

`Tuning.java` only calls `Constants.createFollower(hardwareMap)`, so it works with the
current `Constants.java`.

### Running a tuner

1. Deploy, then on the Driver Station pick **TeleOp → Tuning**.
2. Use the gamepad to choose a folder (*Localization*, *Automatic*, *Manual*, *Tests*) and a tuner.
3. Read results on the Driver Station telemetry or in **Panels**: connect a laptop to the
   robot's Wi-Fi and open `http://192.168.43.1:8001`. Panels also draws the robot on a field
   and graphs errors.
4. Most tuners can apply a result **temporarily** (press **A**). Temporary values are lost
   when the robot restarts. **Always copy the final number into `Constants.java`.**

To edit gains live in Panels instead of redeploying each time, temporarily put
`@Configurable` (from `com.bylazar.configurables.annotations`) on the `Constants` class.

---

## 1. `MecanumConstants`

### 1.1 Motor names

**Where:** Driver Station → ⋮ → **Configure Robot** → your config → **Control Hub** (or
Expansion Hub) → **Motors**. Each port has a name.

**How:** set the four names in `Constants.java` to exactly what the configuration says,
including capitalization. This robot uses `frontLeft`, `frontRight`, `backLeft`, `backRight`.

**If wrong:** the OpMode crashes at init with *"Unable to find a hardware device with name …"*.

### 1.2 Motor directions

**Where:** found by testing; no tuner covers this.

**How:**

1. Prop the robot up so the wheels spin freely.
2. Run a throwaway OpMode that powers **one motor at a time** at `+0.3`:
   ```java
   DcMotorEx m = hardwareMap.get(DcMotorEx.class, "frontLeft");
   m.setDirection(DcMotorSimple.Direction.FORWARD);
   m.setPower(0.3);
   ```
3. Watch which way the wheel turns. If it would roll the robot **forward**, use `FORWARD`.
   Otherwise use `REVERSE`.
4. Repeat for all four motors.

**Typical result:** one side `REVERSE` and the other `FORWARD`, because motors on opposite
sides face opposite ways. This robot has right-front `REVERSE` but right-rear `FORWARD`.
That's unusual, so recheck it with this test.

**Confirm:** with the Pedro *Localization Test*, the left stick forward should drive the
robot straight forward, and strafing should move it straight sideways.

### 1.3 `xVelocity`: top forward speed (in/s)

**Where:** the **Automatic → Forward Velocity Tuner**.

**What the tuner does:** it drives forward at full power for `DISTANCE` = 96 in, records
the robot's speed on every loop, and averages the **last 10 samples** (`RECORD_NUMBER`),
taken when the robot is at top speed.

**How:**

1. Place the robot with at least 96 in plus about 3 ft of stopping room ahead of it.
2. Start the tuner and let it run. Press **B** to abort.
3. Read **Forward Velocity** from the telemetry.
4. Repeat **3 times** on a full battery and average.

**Estimate before measuring** (to sanity-check the result):

```
theoretical speed (in/s) = motor RPM ÷ 60 × π × wheel diameter (in)
real speed               ≈ 80–90 % of theoretical (load, friction, battery)
```

Example: goBILDA 435 RPM motors with 104 mm (4.09 in) wheels:
`435 ÷ 60 × π × 4.09 ≈ 93 in/s` theoretical, so expect about 75 to 85 in/s.
This robot measured **86.8**.

**Red flags:** a value far below the estimate means a motor direction is wrong (wheels
fight each other), the wheels are slipping, or the battery is low.

### 1.4 `yVelocity`: top strafe speed (in/s)

**Where:** the **Automatic → Lateral Velocity Tuner**. Same procedure as 1.3, but driving
sideways, so clear the room to the robot's side.

**Expected:** strafing is slower because the mecanum rollers slip. It's usually **65–85 %
of `xVelocity`**. This robot measured **54.5**, which is 63 % of 86.8. Low but plausible
for heavy robots or worn rollers.

### 1.5 `maxPower`

**Where:** your choice. `1` is full speed. Lower it (for example `0.7`) if the robot is too
fast to control or causes wheel slip. Re-measure 1.3 and 1.4 only if you change the gearing
or wheels, not when you change `maxPower`.

### 1.6 Voltage compensation (optional)

- `useVoltageCompensation(true)` scales power so a drained battery behaves like a fresh one.
- `nominalVoltage`: the voltage you tuned at. Use `12.0`, or the typical voltage measured
  during your tuning session.
- `staticFrictionCoefficient`: leave at `0.1` unless you have a reason to change it.

If you enable this, **re-run all tuners** with it on, since it changes how much power the
wheels actually get.

---

## 2. `PinpointConstants`

### 2.1 `hardwareMapName`

**Where:** Driver Station → Configure Robot → Control Hub → **I2C Bus** *n* (the port the
Pinpoint is plugged into) → device type **goBILDA Pinpoint Odometry Computer** → name it.
This robot uses `pinpoint`.

### 2.2 `encoderResolution`: pod type

**Where:** the pod you bought.

| Pod | Wheel | Encoder | Resolution | Setting |
|---|---|---|---|---|
| goBILDA 4-Bar Odometry Pod | 32 mm | 2000 CPR | 19.894 ticks/mm | `goBILDA_4_BAR_POD` |
| goBILDA Swingarm Odometry Pod | 48 mm | 2000 CPR | 13.263 ticks/mm | `goBILDA_SWINGARM_POD` |

**Other pods:** calculate the resolution and use `customEncoderResolution(...)`:

```
ticks per mm = encoder counts per revolution ÷ (π × wheel diameter in mm)
```

Example: REV Through Bore encoder (8192 CPR) on a 35 mm wheel:
`8192 ÷ (π × 35) = 74.5` ticks/mm, so use `.customEncoderResolution(74.5)`.

### 2.3 Encoder directions

**Where:** found by testing with **Localization → Localization Test**.

**How:** push the robot by hand and watch X and Y:

| Push the robot | Must happen | If it's backwards |
|---|---|---|
| Forward | X increases | Flip `forwardEncoderDirection` |
| Left | Y increases | Flip `strafeEncoderDirection` |

Heading comes from the Pinpoint's built-in IMU, so it has no direction setting.
Rotating counter-clockwise must increase heading. If it doesn't, the Pinpoint is mounted
upside down; flip it physically.

### 2.4 Pod offsets: `forwardPodY` and `strafePodX`

The Pinpoint must know where the pods are relative to the robot's **center of rotation**.
When the robot spins in place, a pod that isn't at the center rolls along an arc, and the
Pinpoint subtracts that rolling using the offsets.

#### Method A: measure with a ruler

1. Find the **center of rotation**: on a mecanum robot, the midpoint between all four wheel
   contact patches (cross the diagonals from wheel to wheel).
2. **Forward pod** (the one whose wheel rolls when driving forward): measure its
   **sideways** distance from the center. **Left of center is positive**, right is negative.
   That's `forwardPodY`.
3. **Strafe pod** (the one whose wheel rolls when strafing): measure its
   **forward/back** distance from the center. **Forward of center is positive**, behind is
   negative. That's `strafePodX`.
4. Measure to the **wheel contact point** of each pod, not to the pod's mounting bracket.

This robot: forward pod 132 mm left, strafe pod 24 mm behind:

```java
.forwardPodY(132.0 / 25.4)   //  5.20 in, left    -> positive
.strafePodX(-24 / 25.4)      // -0.94 in, behind -> negative
```

#### Method B: measure with the Offsets Tuner (more accurate)

1. Set **both offsets to 0** in `Constants.java` and deploy.
2. Run **Localization → Offsets Tuner** with the robot at a marked spot.
3. Rotate the robot **exactly 180°** in place by hand. Line an edge up against a wall or a
   tile seam before and after.
4. Read `strafeX` and `forwardY` from the telemetry, and enter them as `strafePodX` and
   `forwardPodY`.

**Why this works:** with zero offsets, the Pinpoint thinks the pods sit at the center. A
pod that's actually *r* inches off center sweeps a half-circle during a 180° turn, so the
reported position drifts by **2r**. The tuner starts at (72, 72) and computes
`offset = (72 − reported) ÷ 2`.

Repeat 2 to 3 times and average. Then **confirm**: spin the robot a few full turns in
the Localization Test; X and Y should stay within about 0.5 in of where they started.

### 2.5 Distance check (Forward Tuner and Lateral Tuner)

The goBILDA pods have exact factory resolutions, so these tuners are a **check**, not a
measurement.

1. Tape a start line and a line 48 in away (`DISTANCE` = 48).
2. Run **Localization → Forward Tuner** and push the robot from line to line, straight.
3. The telemetry **Multiplier** should be about 1.00 (within ±1 %).
4. Repeat sideways with the **Lateral Tuner**.

**If it's off by more than 1 to 2 %:** the pod wheels are slipping (raise spring tension,
clean the wheels), the wrong pod type is selected, or you're using custom pods. For custom
pods, correct the resolution with:

```
new customEncoderResolution = old resolution ÷ multiplier
```

### 2.6 `yawScalar` (rarely needed)

**Where:** the **Localization → Turn Tuner**. The Pinpoint's IMU is factory calibrated,
so only touch this if heading is measurably off.

1. Align an edge of the robot against a wall.
2. Run the tuner and rotate the robot by hand **exactly** one full turn (`ANGLE` = 2π),
   back against the wall. Ten full turns gives a more accurate result; set `ANGLE` to
   `10 × 2π` to match.
3. If the reported **Multiplier** differs from 1 by more than about 0.5 %, set
   `.yawScalar(multiplier)`.

---

## 3. `FollowerConstants`

### 3.1 `mass` (kg)

**Where:** a scale.

**How:** weigh the competition-ready robot, including battery and game-piece mechanisms.
With a bathroom scale, weigh yourself holding the robot, then subtract your own weight.

```
kg = lb × 0.4536
```

This robot: 14.7 kg (about 32.4 lb). It only affects centripetal correction, so ±0.5 kg
doesn't matter.

### 3.2 `forwardZeroPowerAcceleration` (in/s², negative)

How quickly the robot slows down when the motors are cut and it coasts. Pedro uses this to
predict stopping distance and decide when to start braking.

**Where:** the **Automatic → Forward Zero Power Acceleration Tuner**.

**What the tuner does:** it drives forward at full power until the speed exceeds
`VELOCITY`, cuts power (coast mode), then measures `Δspeed ÷ Δtime` on every loop until
the robot stops, and averages them.

> **Set `VELOCITY` below your `xVelocity` first.** The tuner's default is 90 in/s, but this
> robot's top speed is 86.8 in/s, so it would never reach the trigger speed and would drive
> forever. Use about 85 % of `xVelocity`, for example `VELOCITY = 70`.

**How:** give it plenty of room (acceleration plus coasting, often 8 to 10 ft), run it 3
times, and average. The result **must be negative**. Typical values are −25 to −60.

**Check by hand:**

```
deceleration ≈ −(speed when power was cut)² ÷ (2 × coasting distance)
```

Example: cut at 70 in/s and coast 60 in, giving `−70² ÷ (2 × 60) ≈ −41 in/s²`.

### 3.3 `lateralZeroPowerAcceleration` (in/s², negative)

**Where:** the **Automatic → Lateral Zero Power Acceleration Tuner**. Same procedure,
strafing. Its default `VELOCITY` is 55, but this robot's `yVelocity` is 54.5, so lower it to
about 45. Strafing normally stops **faster** (more negative, often −50 to −80) because
the rollers scrub.

### 3.4 Predictive braking (what this robot uses for the drive)

```java
new PredictiveBrakingCoefficients(p, linearBraking, quadraticFriction)
```

#### `linearBraking` and `quadraticFriction`: measured automatically

**Where:** the **Automatic → Predictive Braking Tuner**. It needs **4 to 5 ft** of room in
front of and behind the robot.

**What the tuner does:**

1. Drives for 1 second at power 1, 1, 1, 0.9, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2,
   alternating forward and backward.
2. After each drive, records the speed *v*, applies reverse braking power −0.2, and records
   how far the robot slides, *d*, before stopping.
3. Fits a curve through the 12 (*v*, *d*) points with least squares:

```
stopping distance d = linearBraking × v + quadraticFriction × v²
```

The **v** term is braking force that grows with speed (motor back-EMF, which behaves like
a damper). The **v²** term is wheel slip at high speed. Copy `kLinearBraking` and
`kQuadraticFriction` from the telemetry.

This robot: `linearBraking = 0.061`, `quadraticFriction = 0.00195`. At 80 in/s, the
predicted stopping distance is `0.061 × 80 + 0.00195 × 80² ≈ 17.4 in`.

The tuner brakes with power 0.2, which is the default `maximumBrakingPower`. If you change
`.withMaximumBrakingPower(...)`, the measured coefficients no longer match; re-run the
tuner with the same braking power.

#### `p`: tuned by hand

The controller outputs `power = p × (distance left − predicted stopping distance)`, so
it starts braking exactly when the robot would otherwise coast past the end.

1. Start with `p = 0.1`.
2. Run **Tests → Line**, which drives 40 in forward and back, over and over.
3. Raise `p` in steps of 0.02 while watching the end of each run:
   - Stops short or creeps slowly to the end → raise `p`.
   - Shakes or buzzes at the end → too high; go back one step.
4. Typical final value: **0.1 to 0.3**. This robot uses **0.13**.

### 3.5 Translational, heading and drive PIDF

These are tuned by **feel** using the Manual tuners. The same procedure applies to all
three:

| Gain | Start | How to tune |
|---|---|---|
| **P** | default value | Raise until corrections are quick. Stop when it starts to overshoot or oscillate. |
| **D** | 0 | Raise until the oscillation from P is damped out. Too much makes it jittery. |
| **F** | 0 | Static friction. Find the smallest power that just makes the robot creep (raise `setTeleOpDrive` power in 0.01 steps until it moves). Use about that value. Usually 0.01 to 0.05. |
| **I** | 0 | Leave at 0. Add a tiny amount (0.0001 to 0.001) only if a small steady error never goes away. |

#### Translational

**Where:** the **Manual → Translational Tuner**. Only the translational loop is active.
The robot follows a 40 in line back and forth.

**How:** push the robot **sideways** off the line. It should snap back without
overshooting. Watch *Error X* and *Error Y* in Panels; they should return to 0 smoothly.
This robot runs the default translational gains.

#### Heading

**Where:** the **Manual → Heading Tuner**.

**How:** **twist** the robot by hand. It should rotate back to the target heading
without wobbling. This robot uses `P = 1.2, I = 0.0004, D = 0.06, F = 0.03`.

**Secondary heading** (`secondaryHeadingPIDFCoefficients`): takes over within
`headingPIDFSwitch` (default π/20 = 9°). Make it **stiffer** (higher P, more D) than the
main loop to remove the last degree or two of error. This robot uses
`P = 1.6, D = 0.14, F = 0.026`.

#### Drive (only if *not* using predictive braking)

**Where:** the **Manual → Drive Tuner**, or **Tests → Line**. Tune so the robot reaches
the end of a 40 in line quickly without overshooting. With predictive braking enabled,
these values are ignored, so skip this.

### 3.6 `centripetalScaling`

**Where:** the **Manual → Centripetal Tuner**, which drives a curve 20 in forward and 20 in
left, back and forth.

**How:**

- The robot swings to the **outside** of the curve → raise `centripetalScaling`.
- It cuts to the **inside** → lower it.
- Typical: 0.0001 to 0.001. This robot uses **0** (off), which is fine if its paths are
  mostly straight lines.

### 3.7 Final checks

Run **Tests → Triangle** and **Tests → Circle**. The robot should trace the shape in
Panels' field view and finish close to where it started.

---

## 4. `PathConstraints`

These values aren't measured; you choose them as a trade-off between accuracy and speed.

### Starting point

Begin with the defaults:

```java
new PathConstraints(0.995, 0.1, 0.1, 0.007, 100, 1, 10, 1)
//                  tValue vel trans heading timeout braking search brakingStart
```

### Adjusting by watching the robot

Run your real autonomous paths (or **Tests → Line**) and watch the end of each path:

| What you see | Change |
|---|---|
| Robot hovers and hunts at the end before moving on | Loosen: raise `velocityConstraint`, `translationalConstraint`, `headingConstraint`, or lower `tValueConstraint` |
| Next path starts before the robot really arrived | Tighten the same values |
| Path ends late because the timeout keeps expiring | The constraints are unreachable. Loosen them, or improve the PIDF tuning |
| Overshoots at the end of fast paths | Lower `brakingStrength`, or raise `brakingStart` to begin braking earlier |
| Stops too gently and wastes time | Raise `brakingStrength` |

### Picking tolerances from match needs

Set the tolerances from how precise the task actually needs to be:

- **Scoring or shooting position:** about ±0.5 in and ±1° (`0.5`, `0.017` rad).
- **Driving past waypoints:** ±1 to 2 in is fine.

```
radians = degrees × π ÷ 180         // 1° = 0.0175 rad
```

This robot loosened the defaults to `0.3 in/s`, `0.3 in`, `0.02 rad` (1.1°), `tValue 0.91`
and `brakingStrength 1.25` to cut settle time between cycles.

---

## 5. Summary

| Value | Source | Method | This robot |
|---|---|---|---|
| Motor names | Robot config | Copy from Driver Station | `frontLeft`, … |
| Motor directions | Test | One motor at a time | see `Constants.java` |
| `xVelocity` | Tuner | Forward Velocity Tuner, 3× average | 86.8 in/s |
| `yVelocity` | Tuner | Lateral Velocity Tuner, 3× average | 54.5 in/s |
| Pinpoint name | Robot config | I2C bus device name | `pinpoint` |
| `encoderResolution` | Datasheet | Pod type, or `CPR ÷ (π × wheel mm)` | 4-bar pod |
| Encoder directions | Test | Push robot; X/Y must increase | forward `REVERSED`, strafe `FORWARD` |
| `forwardPodY`, `strafePodX` | Ruler or tuner | Measure from center, or Offsets Tuner 180° spin | 5.20 in, −0.94 in |
| `yawScalar` | Tuner | Turn Tuner, only if needed | unset |
| `mass` | Scale | `lb × 0.4536` | 14.7 kg |
| `forwardZeroPowerAcceleration` | Tuner | Forward ZPA Tuner (lower `VELOCITY` first) | not set (default −41.3) |
| `lateralZeroPowerAcceleration` | Tuner | Lateral ZPA Tuner (lower `VELOCITY` first) | not set (default −59.8) |
| `linearBraking`, `quadraticFriction` | Tuner | Predictive Braking Tuner (curve fit) | 0.061, 0.00195 |
| Braking `p` | Manual | Line test, raise until jitter | 0.13 |
| Heading PIDF | Manual | Heading Tuner, twist the robot | (1.2, 0.0004, 0.06, 0.03) |
| Translational PIDF | Manual | Translational Tuner, push sideways | defaults |
| `centripetalScaling` | Manual | Centripetal Tuner | 0 |
| `PathConstraints` | Choice | Watch path endings | see section 4 |

> **This robot never measured its zero-power accelerations;** they're still Pedro's
> defaults. Predictive braking replaces most of their role in the drive loop, but running
> the two ZPA tuners and recording the values would make the setup complete.
