package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Pedro Pathing 3.0 constants.
 *
 * Values marked "2.x" are carried over unchanged from the tuned Pedro 2.1.2 setup. Values marked
 * "ESTIMATE" have no 2.x equivalent and must be measured with the Pedro 3 Foresight tuner before
 * relying on path following. See doc/pedro-3-upgrade.md.
 */
public class Constants {
    // Same motors and directions as 2.x.
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeft");
        c.frontRightName.set("frontRight");
        c.backLeftName.set("backLeft");
        c.backRightName.set("backRight");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    // 2.x forwardPodY -> xPodOffset and strafePodX -> yPodOffset: both versions pass them to
    // GoBildaPinpointDriver.setOffsets(x, y) in that order, so localization is unchanged.
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(132.0 / 25.4);
        c.yPodOffset.set(-24 / 25.4);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    // Same structure the Pedro 3 ForesightTuner generates.
    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        // 2.x translational PIDF P (0.1 above 3 in of error, secondary 0.3 below it).
        // ESTIMATE for strafe: 2.x had one translational gain for both directions.
        Controller primaryTranslational = Controller.proportional(0.1);
        Controller secondaryTranslational = Controller.proportional(0.3);
        c.forwardTranslational.set(Controller.piecewise(secondaryTranslational).put(2.5, primaryTranslational));
        c.strafeTranslational.set(Controller.piecewise(secondaryTranslational).put(2.5, primaryTranslational));

        // ESTIMATE: kV = 1 / (top speed per unit power) = 1 / 86.8; the tuner scales brake by 0.85.
        c.coast.set(Controller.proportionalFeedforward(1 / 86.8));
        c.brake.set(Controller.proportionalFeedforward(0.85 / 86.8));

        // 2.x heading PIDF P and F.
        c.headingFeedback.set(Controller.proportional(1.2));
        c.headingStaticFF.set(Controller.staticFeedforward(0.03));
        // ESTIMATE: heading stopping distance = 0.04 * omega + 0.005 * omega^2 (radians).
        c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04, 0.005));

        // 2.x predictive braking (stopping distance = linear * v + quadratic * v^2), forward.
        // ESTIMATE for strafe: reuses the forward values, which predict a longer stop (safer).
        c.linearBrakeCoefficients.set(Matrix.diag(0.061, 0.061));
        c.quadraticBrakeCoefficients.set(Matrix.diag(0.00195, 0.00195));

        // 2.x xVelocity / yVelocity.
        c.maxAchievableForwardVelocity.set(86.8);
        c.maxAchievableStrafeVelocity.set(54.5);
        // ESTIMATE: 2.x never measured these and ran on Pedro's defaults (-41.28 / -59.78).
        c.naturalForwardDeceleration.set(41.28);
        c.naturalStrafeDeceleration.set(59.78);

        // 2.x PathConstraints. 2.x ended a path at t >= 0.91; 3.0 ends it at t >= 1 - this value.
        c.parametricTConstraint.set(0.09);
        c.velocityConstraint.set(0.3);
        c.translationalConstraint.set(0.3);
        c.headingConstraint.set(0.02);
        c.timeoutConstraint.set(100.0);
    });

    public static Follower create(HardwareMap hardwareMap) {
        return new Follower(
                new PinpointLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
