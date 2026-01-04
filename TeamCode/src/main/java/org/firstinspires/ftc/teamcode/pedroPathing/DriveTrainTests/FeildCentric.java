package org.firstinspires.ftc.teamcode.pedroPathing.DriveTrainTests;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@Configurable
@TeleOp(group = "DriveTrain")
public class FeildCentric extends OpMode {
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    private static final double DEADZONE = 0.05;


    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(60, 85))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(138), 0.8))
                .build();
    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
    }

    private double dz(double value) {
        return Math.abs(value) > DEADZONE ? value : 0.0;
    }


    @Override
    public void loop() {
        follower.update();
        telemetryM.update();

        double y  = dz(-gamepad1.left_stick_y);
        double x  = dz(-gamepad1.left_stick_x);
        double rx = dz(-gamepad1.right_stick_x);

        if (!automatedDrive) {
            if (!slowMode) {
                follower.setTeleOpDrive(
                        y,
                        x,
                        rx,
                        false // FIELD-CENTRIC
                );
            } else {
                follower.setTeleOpDrive(
                        y * slowModeMultiplier,
                        x * slowModeMultiplier,
                        rx * slowModeMultiplier,
                        false // FIELD-CENTRIC
                );
            }
        }


        // FIELD-CENTRIC TELEOP DRIVE ONLY
        if (!automatedDrive) {
            if (!slowMode) {
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x,
                        false // FIELD-CENTRIC
                );
            } else {
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y * slowModeMultiplier,
                        -gamepad1.left_stick_x * slowModeMultiplier,
                        -gamepad1.right_stick_x * slowModeMultiplier,
                        false // FIELD-CENTRIC
                );
            }
        }



        // AUTOMATED PATH (A BUTTON)
        if (gamepad1.aWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }

        // STOP AUTO (B BUTTON OR FINISHED)
        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        // SLOW MODE TOGGLE (RIGHT BUMPER)
        if (gamepad1.rightBumperWasPressed()) {
            slowMode = !slowMode;
        }

        // HEADING RESET (X BUTTON)
        if (gamepad1.xWasPressed()) {
            Pose pose = follower.getPose();
            follower.setStartingPose(new Pose(
                    pose.getX(),
                    pose.getY(),
                    0
            ));
        }

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
    }
}