# Decode-Premier-Ivy

Minimal FTC robot project for a mecanum robot with goBILDA Pinpoint odometry, built on
Pedro Pathing 2.1.2. It contains four autonomous OpModes, selected from the **Autonomous**
list on the Driver Station:

| Driver Station name | File | What it does |
|---|---|---|
| **Drive Forward 1m** | `DriveForwardOneMeter.java` | Drives from (0, 0) to (0, 39 in) with a distance PID and stops |
| **AutoPath** | `AutoPath.java` | Drives the 8-move route with a hand-written PID: turns in place, then a full stop at each point |
| **AutoPath_Pedro** | `AutoPathPedro.java` | Same route with Pedro's path follower, `master`-style: each move hands off on arrival, without a pause |
| **AutoPath_Generated** | `AutoPathGenerated.java` | `path.java` translated to Pedro 2.1.2: same route, settles briefly at each point |

All OpModes are in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`.

`path.java` at the repo root is the generated original of the 8-move route. It targets
Pedro Pathing 3.x and isn't part of the build; `AutoPathGenerated.java` is its buildable
translation.

## Hardware

Robot configuration names: motors `frontLeft`, `frontRight`, `backLeft`, `backRight`, and
the Pinpoint odometry computer `pinpoint`. Drive and localizer tuning lives in
`TeamCode/.../pedroPathing/Constants.java`.

## Docs

- [doc/autopath-comparison.md](doc/autopath-comparison.md): how the three ways of driving
  point to point differ (`path.java`, `master`'s Pedro autos, the hand-written PID), and
  which to use
- [doc/autopath.md](doc/autopath.md): the 8-move route, and how `AutoPath` runs and is tuned
- [doc/pedro-constants.md](doc/pedro-constants.md): what every setting in `Constants.java`
  means
- [doc/pedro-tuning-new-robot.md](doc/pedro-tuning-new-robot.md): how to measure or
  calculate each constant on a new robot

## Building

Open the project in Android Studio, or run `./gradlew :TeamCode:assembleDebug`. The APK is
written to `TeamCode/build/outputs/apk/debug/TeamCode-debug.apk`.
