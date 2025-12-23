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
            /*.mass(8.981)
            .forwardZeroPowerAcceleration(-31.26)
            .lateralZeroPowerAcceleration(-47.85)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.07, 0,0.005 , 0.03))
            .headingPIDFCoefficients(new PIDFCoefficients(-1, 0, -0.02, -0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(.5, 0, -0.005, 0.6,0.025))
            .centripetalScaling(0.001); */
    .mass(8.8)
    .forwardZeroPowerAcceleration(-41.26)
    .lateralZeroPowerAcceleration(-81.87)
    .translationalPIDFCoefficients(new PIDFCoefficients(0.09,0,0.005,0.03))
    .headingPIDFCoefficients(new PIDFCoefficients(0.7, 0, 0.025, 0.03))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(.02, 0, 0.005, 0.6,0.005))
    .centripetalScaling(0.00065);


    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();

    }

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(0)
            .strafePodX(2.5)
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
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(67.624)
            .yVelocity(46.94);

}

