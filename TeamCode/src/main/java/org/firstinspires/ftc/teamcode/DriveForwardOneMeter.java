package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Drive Forward 1m")
public class DriveForwardOneMeter extends LinearOpMode {
    // Field coordinates in inches. The robot starts at (0, 0) facing +y ("ahead").
    private static final double START_X = 0;
    private static final double START_Y = 0;
    private static final double TARGET_X = 0;
    private static final double TARGET_Y = 39;
    private static final double HEADING = Math.toRadians(90);

    // Distance PID. kP = 1 / 0.5m: output stays clamped at full power until 0.5m remains,
    // then falls off in proportion to the remaining distance.
    private static final double KP = 1 / (50 / 2.54);
    private static final double KI = 0.002;
    private static final double KD = 0.006;
    // Only integrate near the target so the I term doesn't wind up during the full-power phase.
    private static final double INTEGRAL_ZONE_INCHES = 4;

    // Keeps the robot facing +y while it drives.
    private static final double HEADING_KP = 1.0;

    private static final double POSITION_TOLERANCE_INCHES = 0.5;
    private static final double SPEED_TOLERANCE_INCHES_PER_SEC = 1;
    private static final double TIMEOUT_SECONDS = 5;

    @Override
    public void runOpMode() {
        Follower follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(START_X, START_Y, HEADING));

        waitForStart();
        follower.startTeleopDrive(true);

        ElapsedTime loopTimer = new ElapsedTime();
        ElapsedTime runTimer = new ElapsedTime();
        double integral = 0;
        double lastDistance = Math.hypot(TARGET_X - START_X, TARGET_Y - START_Y);

        while (opModeIsActive() && runTimer.seconds() < TIMEOUT_SECONDS) {
            follower.update();
            double dt = loopTimer.seconds();
            loopTimer.reset();

            Pose pose = follower.getPose();
            double dx = TARGET_X - pose.getX();
            double dy = TARGET_Y - pose.getY();
            double distance = Math.hypot(dx, dy);
            double derivative = dt > 0 ? (distance - lastDistance) / dt : 0;
            lastDistance = distance;

            if (distance < INTEGRAL_ZONE_INCHES) {
                integral += distance * dt;
            } else {
                integral = 0;
            }

            if (distance < POSITION_TOLERANCE_INCHES
                    && Math.abs(derivative) < SPEED_TOLERANCE_INCHES_PER_SEC) {
                break;
            }

            double power = Range.clip(KP * distance + KI * integral + KD * derivative, 0, 1);
            // Field-centric drive vector pointing from the robot straight at the target.
            double driveX = distance > 0 ? power * dx / distance : 0;
            double driveY = distance > 0 ? power * dy / distance : 0;

            double headingError = AngleUnit.normalizeRadians(HEADING - follower.getHeading());
            // Positive turn is counter-clockwise (heading increases), so it follows the error's sign.
            double turn = Range.clip(HEADING_KP * headingError, -0.5, 0.5);

            follower.setTeleOpDrive(driveX, driveY, turn, false);

            telemetry.addData("X / Y (in)", "%.1f / %.1f", pose.getX(), pose.getY());
            telemetry.addData("Distance left (in)", distance);
            telemetry.addData("Power", power);
            telemetry.addData("P / I / D", "%.2f / %.2f / %.2f", KP * distance, KI * integral, KD * derivative);
            telemetry.update();
        }

        follower.setTeleOpDrive(0, 0, 0, false);
        follower.update();
    }
}
