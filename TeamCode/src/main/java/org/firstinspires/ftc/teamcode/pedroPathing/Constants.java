package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12.97)
            .forwardZeroPowerAcceleration(-32.44)
           .lateralZeroPowerAcceleration(-70.03)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.06,0,0.005,0.02))
            .headingPIDFCoefficients(new PIDFCoefficients(0.7, 0, 0.03, 0.01))
            //.useSecondaryHeadingPIDF(false)
            //.secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.8, 0, 0, 0.07))

            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.1, 0, 0.0001, 0.6,0.025))
            //    .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.1, 0, 0.000001, 0.6,0.025))
            // .headingPIDFCoefficients(new PIDFCoefficients(0.5, 0, 0.0001, 0.02))

            .centripetalScaling(0.0005);


    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100,  1.35, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();

    }

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(0)
            .strafePodX(3)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);



    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("FR")
            .rightRearMotorName("BR")
            .leftRearMotorName("BL")
            .leftFrontMotorName("FL")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(81.54)
            .yVelocity(57.75);


}

