package org.firstinspires.ftc.teamcode.SumoRobot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class SUMOTELEOP extends OpMode {

    // Hardware
    private DcMotor fr, fl, br, bl;
    private DcMotorEx shooter1, shooter2;
    private DcMotor intake;
    private Servo kicker;

    private Follower follower;

    // Poses
    private final Pose startPose = new Pose(54, 8, Math.toRadians(90));
    private final Pose parkPose  = new Pose(104.67, 33, Math.toRadians(360));

    // Shooter Constants
    public static final double LOW_VELOCITY  = 1700;
    public static final double HIGH_VELOCITY = 2300;
    private static final double VELOCITY_TOLERANCE = 100;

    // Kicker Positions
    public static final double KICKER_OUT = 0.60;
    public static final double KICKER_IN  = 0.31;

    // Rumble State
    private boolean shooterReadyLast = false;
    private long lastRumbleTime = 0;
    private static final long MIN_RUMBLE_INTERVAL = 250; // ms

    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);

        fr = hardwareMap.get(DcMotor.class, "FR");
        fl = hardwareMap.get(DcMotor.class, "FL");
        br = hardwareMap.get(DcMotor.class, "BR");
        bl = hardwareMap.get(DcMotor.class, "BL");

        shooter1 = hardwareMap.get(DcMotorEx.class, "Shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "Shooter2");

        intake = hardwareMap.get(DcMotor.class, "Intake");
        kicker = hardwareMap.get(Servo.class, "Kicker");

        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);

        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidf = new PIDFCoefficients(100, 0, 0, 15);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        kicker.setPosition(KICKER_IN);
    }

    @Override
    public void loop() {

        // FOLLOWER vs DRIVER CONTROL
        if (gamepad1.triangle) {
            follower.holdPoint(parkPose);
        } else {
            follower.breakFollowing();
            drive();
        }

        // === KICKER (PS5 TRIANGLE) ===
        boolean kickerActive = gamepad2.triangle;
        kicker.setPosition(kickerActive ? KICKER_OUT : KICKER_IN);

        // === SHOOTER PRESETS ===
        boolean lowGoal  = gamepad2.left_bumper;
        boolean highGoal = gamepad2.right_bumper;

        double targetVelocity = 0;

        if (highGoal) {
            targetVelocity = HIGH_VELOCITY;
        } else if (lowGoal) {
            targetVelocity = LOW_VELOCITY;
        }

        // APPLY SHOOTER
        if (targetVelocity > 0) {
            shooter1.setVelocity(targetVelocity);
            shooter2.setVelocity(targetVelocity);
            intake.setPower(kickerActive ? 0.0 : 1.0);
        } else {
            shooter1.setVelocity(0);
            shooter2.setVelocity(0);
            intake.setPower(0);
        }

        // RUMBLE LOGIC
        double currentVelocity =
                (shooter1.getVelocity() + shooter2.getVelocity()) / 2.0;

        boolean shooterReadyNow =
                targetVelocity > 0 &&
                        Math.abs(currentVelocity - targetVelocity) <= VELOCITY_TOLERANCE;

        long now = System.currentTimeMillis();

        if (shooterReadyNow && !shooterReadyLast &&
                now - lastRumbleTime >= MIN_RUMBLE_INTERVAL) {
            gamepad2.rumble(200);
            lastRumbleTime = now;
        }

        shooterReadyLast = shooterReadyNow;

        // === TELEMETRY ===
        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Current Velocity", currentVelocity);
        telemetry.addData("Shooter Ready", shooterReadyNow);
        telemetry.update();
    }

    private void drive() {
        double vertical   = gamepad1.left_stick_y;
        double horizontal = gamepad1.left_stick_x;
        double pivot      = gamepad1.right_stick_x * 0.75;

        fr.setPower(pivot + vertical + horizontal);
        br.setPower(-pivot - vertical + horizontal);
        bl.setPower(pivot - vertical - horizontal);
        fl.setPower(-pivot + vertical - horizontal);
    }
}
