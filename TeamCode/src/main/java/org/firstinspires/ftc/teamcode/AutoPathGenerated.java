package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

/**
 * path.java translated line-for-line from the Pedro Pathing 3.x API it was generated for to the
 * Pedro 2.1.2 API this project uses. Same poses, same 8 tangent-heading lines, same
 * sequential(follow(...)) routine: each follow runs to completion, so the robot brakes and
 * settles at every point before the next move.
 */
@Autonomous(name = "AutoPath_Generated", group = "Autonomous")
public class AutoPathGenerated extends LinearOpMode {

    private Follower follower;
    private int pathNumber = 0;

    // path.java: PoseFactory.degrees().of(x, y, degrees)
    private static Pose pose(double x, double y, double headingDegrees) {
        return new Pose(x, y, Math.toRadians(headingDegrees));
    }

    private final Pose start = pose(55.4, 0, 90);
    private final Pose path1 = pose(56, 26, 88.678);
    private final Pose point2 = pose(52, 6, -101.3099);
    private final Pose point3 = pose(0, 0, -173.4181);
    private final Pose point4 = pose(0, 15, 90);
    private final Pose point5 = pose(14, 15, 0);
    private final Pose point6 = pose(14, 118, 90);
    private final Pose point7 = pose(57, 118, 0);
    private final Pose point8 = pose(57, 113, -90);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
            move(1, path1()),
            move(2, path2()),
            move(3, path3()),
            move(4, path4()),
            move(5, path5()),
            move(6, path6()),
            move(7, path7()),
            move(8, path8())
        );
    }

    // follow(follower, path) plus a move counter for telemetry. path.java shows
    // follower.pathIndex(), which is always 0 here because each follow runs a single path.
    private Command move(int number, PathChain path) {
        return sequential(
            instant(() -> pathNumber = number),
            follow(follower, path)
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.createFollower(hardwareMap);   // path.java: Constants.create(hardwareMap)
        follower.setStartingPose(start);                     // path.java: follower.setPose(start)
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            Pose pose = follower.getPose();                  // path.java: follower.pose()
            telemetry.addData("x", pose.getX());
            telemetry.addData("y", pose.getY());
            telemetry.addData("heading (deg)", Math.toDegrees(pose.getHeading()));

            if (follower.getCurrentPath() != null) {         // path.java: follower.currentPath()
                // path.java: follower.distanceToEndpoint()
                telemetry.addData("Current path distance remaining",
                        follower.getCurrentPath().endPose().distanceFrom(pose));
                telemetry.addData("Path number", pathNumber);
            }

            telemetry.update();
        }
    }

    // path.java: Paths.line(a, b).tangent()
    private PathChain line(Pose from, Pose to) {
        return follower.pathBuilder()
                .addPath(new BezierLine(from, to))
                .setTangentHeadingInterpolation()
                .build();
    }

    public PathChain path1() {
        return line(start, path1);
    }

    public PathChain path2() {
        return line(path1, point2);
    }

    public PathChain path3() {
        return line(point2, point3);
    }

    public PathChain path4() {
        return line(point3, point4);
    }

    public PathChain path5() {
        return line(point4, point5);
    }

    public PathChain path6() {
        return line(point5, point6);
    }

    public PathChain path7() {
        return line(point6, point7);
    }

    public PathChain path8() {
        return line(point7, point8);
    }
}
