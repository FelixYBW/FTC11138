package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Drive Forward 1m")
public class DriveForwardOneMeter extends LinearOpMode {
    private static final double ONE_METER_INCHES = 100 / 2.54;

    @Override
    public void runOpMode() {
        Follower follower = Constants.createFollower(hardwareMap);
        Pose start = new Pose(0, 0, 0);
        Pose end = new Pose(ONE_METER_INCHES, 0, 0);
        follower.setStartingPose(start);

        PathChain forward = follower.pathBuilder()
                .addPath(new BezierLine(start, end))
                .setConstantHeadingInterpolation(0)
                .build();

        waitForStart();
        follower.followPath(forward, true);

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            telemetry.addData("X (in)", follower.getPose().getX());
            telemetry.addData("Y (in)", follower.getPose().getY());
            telemetry.update();
        }
    }
}
