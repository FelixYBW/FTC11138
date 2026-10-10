package org.firstinspires.ftc.teamcode.opmode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

/**
 * The 8-move route from path.java, driven the way FarZoneAuto drives: each move is a Pedro
 * PathChain (a straight BezierLine with tangent heading) followed by the tuned Follower, and
 * each move ends as soon as the robot reaches the path's parametric end or LEG_TIMEOUT_MS
 * elapses - it does not sit out the follower's settle before starting the next move.
 */
@Autonomous(name = "AutoPath (Pedro)", group = "Autonomous")
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
    private static final double MAX_POWER = 1.0;

    private Follower follower;
    private int leg = 0;
    private String phase = "init";
    private final ElapsedTime legTimer = new ElapsedTime();
    // Where the robot was when each move handed off to the next, shown on the Driver Station.
    private final List<String> arrivals = new ArrayList<>();

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(POINTS[0][0], POINTS[0][1], Math.toRadians(START_HEADING_DEGREES)));
        Command routine = autoRoutine();

        // Show the live position during INIT, so odometry can be checked by pushing the robot.
        while (opModeInInit()) {
            follower.update();
            report();
        }

        phase = "drive";
        schedule(routine);
        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();
            report();
        }
    }

    private Command autoRoutine() {
        Command[] steps = new Command[POINTS.length];
        for (int i = 1; i < POINTS.length; i++) {
            final int move = i;
            steps[i - 1] = sequential(
                    instant(() -> {
                        leg = move;
                        legTimer.reset();
                    }),
                    followWithTimeout(line(POINTS[i - 1], POINTS[i])),
                    instant(() -> logArrival(move))
            );
        }
        // The follower keeps holding the last point once the routine ends.
        steps[POINTS.length - 1] = instant(() -> phase = "done");
        return sequential(steps);
    }

    /** A straight move between two points, facing the direction of travel (tangent heading). */
    private PathChain line(double[] from, double[] to) {
        return follower.pathBuilder()
                .addPath(new BezierLine(new Pose(from[0], from[1]), new Pose(to[0], to[1])))
                .setTangentHeadingInterpolation()
                .build();
    }

    /**
     * Same as FarZoneAuto.followWithTimeout: follow the path, finishing as soon as the robot
     * reaches the path's parametric end OR the timeout elapses, instead of waiting for the
     * follower to settle on the end point.
     */
    private Command followWithTimeout(PathChain path) {
        return follow(follower, path, MAX_POWER)
                .raceWith(waitUntil(() -> follower.atParametricEnd()))
                .raceWith(waitMs(LEG_TIMEOUT_MS));
    }

    private void logArrival(int move) {
        double[] target = POINTS[move];
        Pose pose = follower.getPose();
        String reason = legTimer.milliseconds() >= LEG_TIMEOUT_MS ? "TIMEOUT" : "arrived";
        arrivals.add(String.format("P%d (%.0f, %.0f) -> (%.1f, %.1f) %.1f°, off by %.1f in, %s in %.1fs",
                move, target[0], target[1], pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()),
                Math.hypot(target[0] - pose.getX(), target[1] - pose.getY()), reason, legTimer.seconds()));
    }

    private void report() {
        Pose pose = follower.getPose();
        telemetry.addData("Move", "%d of %d (%s)", leg, POINTS.length - 1, phase);
        telemetry.addData("x / y (in)", "%.1f / %.1f", pose.getX(), pose.getY());
        telemetry.addData("heading (deg)", "%.1f", Math.toDegrees(pose.getHeading()));
        telemetry.addData("Busy", follower.isBusy());
        for (String arrival : arrivals) {
            telemetry.addLine(arrival);
        }
        telemetry.update();
    }
}
