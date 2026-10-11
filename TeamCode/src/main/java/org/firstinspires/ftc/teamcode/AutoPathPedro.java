package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;

/**
 * The 8-move route from path.java, driven the way FarZoneAuto drives on master: each move is a
 * straight Pedro path with tangent heading followed by the Follower, and each move ends as soon
 * as the robot reaches the path's parametric end or LEG_TIMEOUT_MS elapses - it does not sit out
 * the follower's settle before starting the next move.
 */
@Autonomous(name = "AutoPath_Pedro", group = "Autonomous")
public class AutoPathPedro extends LinearOpMode {
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

    private static final long LEG_TIMEOUT_MS = 5000;

    private Follower follower;
    private int leg = 0;
    private String phase = "init";
    private final ElapsedTime legTimer = new ElapsedTime();
    // Where the robot was when each move handed off to the next, shown on the Driver Station.
    private final List<String> arrivals = new ArrayList<>();

    @Override
    public void runOpMode() {
        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(POINTS[0][0], POINTS[0][1], Math.toRadians(START_HEADING_DEGREES)));

        // Show the live position during INIT, so odometry can be checked by pushing the robot.
        while (opModeInInit()) {
            follower.update();
            report();
        }

        phase = "drive";
        startLeg(1);
        while (opModeIsActive()) {
            follower.update();

            // Same race as FarZoneAuto.followWithTimeout: the move ends when the path finishes,
            // reaches its parametric end, or times out, whichever comes first.
            if (leg < POINTS.length
                    && (!follower.isBusy() || follower.atParametricEnd()
                        || legTimer.milliseconds() >= LEG_TIMEOUT_MS)) {
                logArrival(leg);
                if (leg + 1 < POINTS.length) {
                    startLeg(leg + 1);
                } else {
                    // Leave the last path running: the follower finishes it and then holds its end
                    // point, instead of holding wherever the robot was at the hand-off.
                    leg = POINTS.length;
                    phase = "done";
                }
            }
            report();
        }
    }

    private void startLeg(int move) {
        leg = move;
        legTimer.reset();
        follower.follow(line(POINTS[move - 1], POINTS[move]));
    }

    /** A straight move between two points, facing the direction of travel (tangent heading). */
    private Path line(double[] from, double[] to) {
        return Paths.line(new Pose(from[0], from[1]), new Pose(to[0], to[1])).tangent();
    }

    private void logArrival(int move) {
        double[] target = POINTS[move];
        Pose pose = follower.pose();
        String reason = legTimer.milliseconds() >= LEG_TIMEOUT_MS ? "TIMEOUT" : "arrived";
        arrivals.add(String.format("P%d (%.0f, %.0f) -> (%.1f, %.1f) %.1f°, off by %.1f in, %s in %.1fs",
                move, target[0], target[1], pose.x(), pose.y(), Math.toDegrees(pose.heading()),
                Math.hypot(target[0] - pose.x(), target[1] - pose.y()), reason, legTimer.seconds()));
    }

    private void report() {
        Pose pose = follower.pose();
        telemetry.addData("Move", "%d of %d (%s)", Math.min(leg, POINTS.length - 1), POINTS.length - 1, phase);
        telemetry.addData("x / y (in)", "%.1f / %.1f", pose.x(), pose.y());
        telemetry.addData("heading (deg)", "%.1f", Math.toDegrees(pose.heading()));
        telemetry.addData("Busy", follower.isBusy());
        for (String arrival : arrivals) {
            telemetry.addLine(arrival);
        }
        telemetry.update();
    }
}
