package org.firstinspires.ftc.teamcode.SumoRobot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@TeleOp(name = "Shooter + Pedro Test")
public class shooterAccuracyTest extends OpMode {

    private DcMotorEx shooter1, shooter2;
    private Follower follower;

    static final double TICKS_PER_REV = 537.7;
    static final double MAX_RPM = 5600;
    static final double TARGET_X = 144;
    static final double TARGET_Y = 36;
    static final double VELOCITY_TOLERANCE = 50;

    @Override
    public void init() {

        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");

        shooter1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        follower = Constants.createFollower(hardwareMap);

        telemetry.addLine("Shooter + Pedro Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {

        // Update Pedro odometry
        follower.update();
        Pose pose = follower.getPose();

        double distance = Math.hypot(TARGET_X - pose.getX(), TARGET_Y - pose.getY());

        // Distance → RPM lookup table
        double targetRPM;
        if (distance < 30) targetRPM = 3600;
        else if (distance < 45) targetRPM = 4000;
        else if (distance < 60) targetRPM = 4400;
        else if (distance < 75) targetRPM = 4800;
        else targetRPM = 5200;

        targetRPM = Math.min(targetRPM, MAX_RPM);
        double targetVelocity = targetRPM * TICKS_PER_REV / 60.0;

        // Shooter control
        if (gamepad1.right_bumper) {
            shooter1.setVelocity(targetVelocity);
            shooter2.setVelocity(targetVelocity);

            boolean readyToFire = Math.abs(shooter1.getVelocity() - targetVelocity) < VELOCITY_TOLERANCE &&
                    Math.abs(shooter2.getVelocity() - targetVelocity) < VELOCITY_TOLERANCE;
            telemetry.addData("Ready to Fire", readyToFire);
        } else {
            shooter1.setVelocity(0);
            shooter2.setVelocity(0);
        }

        telemetry.addData("Pose X", pose.getX());
        telemetry.addData("Pose Y", pose.getY());
        telemetry.addData("Distance", distance);
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Shooter1 Velocity", shooter1.getVelocity());
        telemetry.addData("Shooter2 Velocity", shooter2.getVelocity());
        telemetry.update();
    }
}
