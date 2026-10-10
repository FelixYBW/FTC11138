package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;

/**
 * Drives the route from path.java: straight lines between the points below, each with tangent
 * heading (the robot faces its direction of travel). Before a segment that needs a large turn,
 * the robot turns in place to the segment's heading, then drives it and PID-stops on the point.
 */
@Autonomous(name = "AutoPath", group = "Autonomous")
public class AutoPath extends LinearOpMode {
    // Inches. The robot starts on the first point facing START_HEADING_DEGREES.
    private static final double[][] POINTS = {
            {55.4, 0},   // start
            {56, 26},    // path1
            {52, 6},     // point2
            {0, 0},      // point3
            {0, 15},     // point4
            {14, 15},    // point5
            {14, 118},   // point6
            {57, 118},   // point7
            {57, 113},   // point8
    };
    private static final double START_HEADING_DEGREES = 90;

    // Distance PID. kP = 1 / 0.5m: full power until 0.5m remains, then proportional slowdown.
    private static final double KP = 1 / (50 / 2.54);
    private static final double KI = 0.002;
    private static final double KD = 0.006;
    // Only integrate near the target so the I term doesn't wind up during the full-power phase.
    private static final double INTEGRAL_ZONE_INCHES = 4;
    // Static-friction push toward the point. Without it, P alone drops below the power needed to move
    // about 1-2 in short, and the robot stalls there until the segment timeout.
    private static final double KF = 0.07;

    // Heading PID, error in radians. KF is a small push that overcomes friction near the target.
    private static final double HEADING_KP = 1.0;
    private static final double HEADING_KD = 0.05;
    private static final double HEADING_KF = 0.04;
    private static final double MAX_TURN = 0.5;
    // Turn in place first when a segment's heading differs from the current one by more than this.
    private static final double TURN_IN_PLACE_RADIANS = Math.toRadians(15);

    private static final double POSITION_TOLERANCE_INCHES = 0.5;
    private static final double HEADING_TOLERANCE_RADIANS = Math.toRadians(2);
    private static final double SPEED_TOLERANCE_INCHES_PER_SEC = 1;
    private static final double SEGMENT_TIMEOUT_SECONDS = 5;
    private static final double TURN_TIMEOUT_SECONDS = 3;
    // A turn ends once the heading has stayed within tolerance this long without leaving it.
    private static final double TURN_SETTLE_SECONDS = 0.1;

    private Follower follower;
    private int pathNumber;
    private String phase = "";
    // Where the robot actually stopped at each point, shown on the Driver Station.
    private final List<String> arrivals = new ArrayList<>();

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(POINTS[0][0], POINTS[0][1], Math.toRadians(START_HEADING_DEGREES)));

        // Show the live position during INIT, so odometry can be checked by pushing the robot.
        while (opModeInInit()) {
            follower.update();
            phase = "init";
            report(0, 0, 0);
        }
        follower.startTeleopDrive(true);

        for (int i = 1; i < POINTS.length && opModeIsActive(); i++) {
            pathNumber = i;
            double[] from = POINTS[i - 1];
            double[] to = POINTS[i];
            double tangentHeading = Math.atan2(to[1] - from[1], to[0] - from[0]);

            String turnResult = "no turn";
            if (Math.abs(AngleUnit.normalizeRadians(tangentHeading - follower.getHeading())) > TURN_IN_PLACE_RADIANS) {
                turnResult = turnTo(tangentHeading);
            }
            String result = driveTo(to[0], to[1], tangentHeading);

            Pose pose = follower.getPose();
            arrivals.add(String.format("P%d (%.0f, %.0f) -> (%.1f, %.1f) %.1f°, off by %.1f in, %s",
                    i, to[0], to[1], pose.getX(), pose.getY(), Math.toDegrees(follower.getHeading()),
                    Math.hypot(to[0] - pose.getX(), to[1] - pose.getY()), result));
            arrivals.add("    turn before P" + i + ": " + turnResult);
        }

        stopDrive();
        // Keep the final position and the per-point results on screen until STOP is pressed.
        phase = "done";
        while (opModeIsActive()) {
            follower.update();
            report(0, 0, 0);
        }
    }

    /**
     * Rotates in place until the heading has settled within tolerance of the target.
     * Returns how the turn ended, for the Driver Station.
     */
    private String turnTo(double targetHeading) {
        phase = "turn";
        ElapsedTime loopTimer = new ElapsedTime();
        ElapsedTime timer = new ElapsedTime();
        Pose hold = follower.getPose();
        double lastError = headingError(targetHeading);
        ElapsedTime settleTimer = new ElapsedTime();
        boolean settling = false;
        double error = lastError;

        while (opModeIsActive() && timer.seconds() < TURN_TIMEOUT_SECONDS) {
            follower.update();
            double dt = loopTimer.seconds();
            loopTimer.reset();

            error = headingError(targetHeading);
            double rate = dt > 0 ? (error - lastError) / dt : 0;
            lastError = error;

            // Restart the settle timer whenever the heading leaves the tolerance window (e.g. overshoot).
            if (Math.abs(error) < HEADING_TOLERANCE_RADIANS) {
                if (!settling) {
                    settling = true;
                    settleTimer.reset();
                } else if (settleTimer.seconds() >= TURN_SETTLE_SECONDS) {
                    return String.format("settled in %.1fs", timer.seconds());
                }
            } else {
                settling = false;
            }

            // Hold position with the drive PID's P term so the turn doesn't wander.
            Pose pose = follower.getPose();
            double driveX = Range.clip(KP * (hold.getX() - pose.getX()), -1, 1);
            double driveY = Range.clip(KP * (hold.getY() - pose.getY()), -1, 1);
            follower.setTeleOpDrive(driveX, driveY, headingCorrection(error, rate), false);
            report(hold.getX() - pose.getX(), hold.getY() - pose.getY(), 0);
        }
        return String.format("TIMEOUT %.1fs: hdg %.1f°", timer.seconds(), Math.toDegrees(error));
    }

    /**
     * Drives to the target point holding the target heading, and PID-stops on it.
     * Returns how the leg ended, for the Driver Station: its time, and on a timeout the unmet values.
     */
    private String driveTo(double targetX, double targetY, double targetHeading) {
        phase = "drive";
        ElapsedTime loopTimer = new ElapsedTime();
        ElapsedTime timer = new ElapsedTime();
        double integral = 0;
        Pose startPose = follower.getPose();
        double lastDistance = Math.hypot(targetX - startPose.getX(), targetY - startPose.getY());
        double lastHeadingError = headingError(targetHeading);
        double distance = lastDistance;
        double derivative = 0;
        double error = lastHeadingError;

        while (opModeIsActive() && timer.seconds() < SEGMENT_TIMEOUT_SECONDS) {
            follower.update();
            double dt = loopTimer.seconds();
            loopTimer.reset();

            Pose pose = follower.getPose();
            double dx = targetX - pose.getX();
            double dy = targetY - pose.getY();
            distance = Math.hypot(dx, dy);
            derivative = dt > 0 ? (distance - lastDistance) / dt : 0;
            lastDistance = distance;

            error = headingError(targetHeading);
            double rate = dt > 0 ? (error - lastHeadingError) / dt : 0;
            lastHeadingError = error;

            if (distance < INTEGRAL_ZONE_INCHES) {
                integral += distance * dt;
            } else {
                integral = 0;
            }

            if (distance < POSITION_TOLERANCE_INCHES
                    && Math.abs(derivative) < SPEED_TOLERANCE_INCHES_PER_SEC
                    && Math.abs(error) < HEADING_TOLERANCE_RADIANS) {
                stopDrive();
                return String.format("reached in %.1fs", timer.seconds());
            }

            double feedforward = distance > POSITION_TOLERANCE_INCHES ? KF : 0;
            double power = Range.clip(KP * distance + KI * integral + KD * derivative + feedforward, 0, 1);
            // Field-centric drive vector pointing from the robot straight at the target.
            double driveX = distance > 0 ? power * dx / distance : 0;
            double driveY = distance > 0 ? power * dy / distance : 0;
            follower.setTeleOpDrive(driveX, driveY, headingCorrection(error, rate), false);
            report(dx, dy, power);
        }

        stopDrive();
        // Last values seen before the timeout; whichever is outside its tolerance kept the leg running.
        return String.format("TIMEOUT %.1fs: dist %.2f in, speed %.1f in/s, hdg %.1f°",
                timer.seconds(), distance, Math.abs(derivative), Math.toDegrees(error));
    }

    private double headingError(double targetHeading) {
        return AngleUnit.normalizeRadians(targetHeading - follower.getHeading());
    }

    /** Positive turn is counter-clockwise (heading increases), so it follows the error's sign. */
    private double headingCorrection(double error, double rate) {
        double feedforward = Math.abs(error) > HEADING_TOLERANCE_RADIANS ? Math.signum(error) * HEADING_KF : 0;
        return Range.clip(HEADING_KP * error + HEADING_KD * rate + feedforward, -MAX_TURN, MAX_TURN);
    }

    private void stopDrive() {
        follower.setTeleOpDrive(0, 0, 0, false);
        follower.update();
    }

    private void report(double dx, double dy, double power) {
        Pose pose = follower.getPose();
        telemetry.addData("Path number", "%d (%s)", pathNumber, phase);
        telemetry.addData("x / y (in)", "%.1f / %.1f", pose.getX(), pose.getY());
        telemetry.addData("heading (deg)", "%.1f", Math.toDegrees(follower.getHeading()));
        telemetry.addData("Distance remaining (in)", "%.1f", Math.hypot(dx, dy));
        telemetry.addData("Power", "%.2f", power);
        for (String arrival : arrivals) {
            telemetry.addLine(arrival);
        }
        telemetry.update();
    }
}
