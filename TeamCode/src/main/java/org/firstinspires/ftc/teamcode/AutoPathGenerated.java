package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * path.java on Pedro Pathing 3.0, the version it was generated for. The poses, paths and
 * telemetry are path.java's own code. The only change is the routine: path.java runs
 * sequential(follow(...), ...) from the ivy command library, which has no Pedro 3 release, so
 * this runs the same sequence with a loop. Each path runs until the follower finishes it, so the
 * robot settles at every point before the next move, as in path.java.
 */
@Autonomous(name = "AutoPath_Generated", group = "Autonomous")
public class AutoPathGenerated extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(55.4, 0, 90);
    private final Pose path1 = poseFactory.of(56, 26, 88.678);
    private final Pose point2 = poseFactory.of(52, 6, -101.3099);
    private final Pose point3 = poseFactory.of(0, 0, -173.4181);
    private final Pose point4 = poseFactory.of(0, 15, 90);
    private final Pose point5 = poseFactory.of(14, 15, 0);
    private final Pose point6 = poseFactory.of(14, 118, 90);
    private final Pose point7 = poseFactory.of(57, 118, 0);
    private final Pose point8 = poseFactory.of(57, 113, -90);

    // Autonomous routine: path.java's sequential(follow(follower, path1()), ...).
    private Path[] autoRoutine() {
        return new Path[]{path1(), path2(), path3(), path4(), path5(), path6(), path7(), path8()};
    }

    @Override
    public void runOpMode() {
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();
        Path[] routine = autoRoutine();
        int next = 0;

        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            // Start the next path once the current one has finished (path.java's sequential).
            if (!follower.isBusy() && next < routine.length) {
                follower.follow(routine[next++]);
            }

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                // follower.pathIndex() counts segments within one path; each path here is one segment.
                telemetry.addData("Path number", next);
            }

            telemetry.update();
        }
    }

    public Path path1() {
        return Paths.line(start, path1).tangent();
    }

    public Path path2() {
        return Paths.line(path1, point2).tangent();
    }

    public Path path3() {
        return Paths.line(point2, point3).tangent();
    }

    public Path path4() {
        return Paths.line(point3, point4).tangent();
    }

    public Path path5() {
        return Paths.line(point4, point5).tangent();
    }

    public Path path6() {
        return Paths.line(point5, point6).tangent();
    }

    public Path path7() {
        return Paths.line(point6, point7).tangent();
    }

    public Path path8() {
        return Paths.line(point7, point8).tangent();
    }
}
