package org.firstinspires.ftc.teamcode.pedro;

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
 * Pedro Pathing 3.x constants, tuned with AutoTune (values carried over from the other repo).
 * Motor names are this robot's config names: FL, FR, BL, BR.
 */
public class Constants {

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("FL");
        c.frontRightName.set("FR");
        c.backLeftName.set("BL");
        c.backRightName.set("BR");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(5.486309021476686);
        c.yPodOffset.set(0.3080375738969938);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.26645342109943193);
                Controller secondaryTranslationalForward = Controller.proportional(0.0984474193297017);
                Controller primaryTranslationalLateral = Controller.proportional(0.2693759849346184);
                Controller secondaryTranslationalLateral = Controller.proportional(0.09952722857445917);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.012468958855124865));
                c.brake.set(Controller.proportionalFeedforward(0.010598615026856136));

                c.headingFeedback.set(Controller.proportional(2.593369189609896));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.048774189997895304, 0.004047612335213798));

                c.linearBrakeCoefficients.set(Matrix.diag(0.10679725127430298, 0.05390872899997885));
                c.quadraticBrakeCoefficients.set(Matrix.diag(7.530792951605766E-4, 0.0017130424412057144));

                c.maxAchievableForwardVelocity.set(81.03909517403349);
                c.maxAchievableStrafeVelocity.set(60.604814439936874);
                c.naturalForwardDeceleration.set(36.95058754694533);
                c.naturalStrafeDeceleration.set(73.25509909745702);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }

}
