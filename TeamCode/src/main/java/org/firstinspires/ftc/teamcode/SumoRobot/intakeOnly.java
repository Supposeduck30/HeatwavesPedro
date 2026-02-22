package org.firstinspires.ftc.teamcode.SumoRobot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp




public class intakeOnly extends OpMode {

    private DcMotor fr, fl, br, bl;
    private DcMotor intake;
    private Follower follower;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(1);
        Pose startPose = new Pose(54.5, 8, Math.toRadians(90));
        follower.setStartingPose(startPose);

        fr = hardwareMap.get(DcMotor.class, "FR");
        fl = hardwareMap.get(DcMotor.class, "FL");
        br = hardwareMap.get(DcMotor.class, "BR");
        bl = hardwareMap.get(DcMotor.class, "BL");

        intake = hardwareMap.get(DcMotor.class, "Intake");

        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);

        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    private boolean firstLoop = true;

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();

        if (firstLoop) {
            firstLoop = false;
            telemetry.addData("Status", "Initializing...");
            telemetry.update();
            return;
        }

        if (gamepad1.triangle) {
            follower.setPose(new Pose(7, 9, Math.toRadians(90)));
        }

        double driveForward = -gamepad1.left_stick_y;
        double driveStrafe  = -gamepad1.left_stick_x;
        double driveTurn    = -gamepad1.right_stick_x;

        follower.setTeleOpDrive(driveForward, driveStrafe, driveTurn, true);

        /* ---------- INTAKE ---------- */
        double intakePower = 0;
        if (gamepad2.cross) {               // X → intake reverse
            intakePower = -0.9;
        } else if (gamepad2.left_bumper) {  // LB → intake forward
            intakePower = 1.0;
        } else if (gamepad2.right_bumper) { // RB → intake forward (shoot)
            intakePower = 1.0;
        }

        intake.setPower(intakePower);

        /* ---------- TELEMETRY ---------- */
        Pose currentPose = follower.getPose();

        telemetry.addData("--- POSE ---", "");
        telemetry.addData("X Position", "%.2f", currentPose.getX());
        telemetry.addData("Y Position", "%.2f", currentPose.getY());
        telemetry.addData("Heading (degrees)", "%.2f", Math.toDegrees(currentPose.getHeading()));
        telemetry.addData("Motor direction of front right", fr.getDirection());
        telemetry.update();
    }
}