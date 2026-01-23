package org.firstinspires.ftc.teamcode.SumoRobot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
@TeleOp
public class SUMOTELEOP extends OpMode {

    // Drive motors
    private DcMotor fr, fl, br, bl;
    // Shooter motors
    private DcMotorEx shooter1, shooter2;
    // Intake and kicker
    private DcMotor intake;
    private Servo kicker;


    // Kicker pulse state
    private boolean kicking = false;
    private long kickStartTime = 0;
    private boolean kickerButtonLast = false;
    private static final long KICK_TIME = 140; // milliseconds

    // Shooter constants
    public static final double LOW_VELOCITY  = 1700;
    public static final double HIGH_VELOCITY = 2300;
    private static final double VELOCITY_TOLERANCE = 100;

    // Kicker positions
    public static final double KICKER_OUT = 0.52;
    public static final double KICKER_IN  = 0.35;

    @Override
    public void init() {
        // Initialize drive motors

        fr = hardwareMap.get(DcMotor.class, "FR");
        fl = hardwareMap.get(DcMotor.class, "FL");
        br = hardwareMap.get(DcMotor.class, "BR");
        bl = hardwareMap.get(DcMotor.class, "BL");

        // Initialize shooter motors
        shooter1 = hardwareMap.get(DcMotorEx.class, "Shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "Shooter2");

        // Initialize intake and kicker
        intake = hardwareMap.get(DcMotor.class, "Intake");
        kicker = hardwareMap.get(Servo.class, "Kicker");

        // Set drive motor directions
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set brake behavior
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Shooter setup
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidf = new PIDFCoefficients(120, 0, 0, 14);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        // Set kicker to initial position
        kicker.setPosition(KICKER_IN);
    }

    @Override
    public void loop() {

        // Handle kicker pulse
        boolean kickerButtonNow = gamepad2.triangle;

        // Detect button press (false -> true) to start kicker pulse
        if (kickerButtonNow && !kickerButtonLast && !kicking) {
            kicking = true;
            kickStartTime = System.currentTimeMillis();
            kicker.setPosition(KICKER_OUT);
        }

        // Automatically retract kicker after KICK_TIME
        if (kicking && System.currentTimeMillis() - kickStartTime >= KICK_TIME) {
            kicker.setPosition(KICKER_IN);
            kicking = false;
        }

        kickerButtonLast = kickerButtonNow;

        // Shooter presets
        boolean lowGoal  = gamepad2.left_bumper;
        boolean highGoal = gamepad2.right_bumper;

        double targetVelocity = 0;

        if (highGoal) {
            targetVelocity = HIGH_VELOCITY;
        } else if (lowGoal) {
            targetVelocity = LOW_VELOCITY;
        }

        // Apply shooter power
        if (targetVelocity > 0) {
            shooter1.setVelocity(targetVelocity);
            shooter2.setVelocity(targetVelocity);
        } else {
            shooter1.setVelocity(0);
            shooter2.setVelocity(0);
        }

        // Intake always runs when shooter is active
        if (targetVelocity > 0) {
            intake.setPower(1.0);
        } else {
            intake.setPower(0);
        }

        // Telemetry
        double currentVelocity = (shooter1.getVelocity() + shooter2.getVelocity()) / 2.0;
        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Current Velocity", currentVelocity);
        telemetry.update();

        // Drive
        drive();
    }

    private void drive() {
        double vertical   = gamepad1.left_stick_y;
        double horizontal = -gamepad1.left_stick_x;
        double pivot      = gamepad1.right_stick_x * 0.75;

        fr.setPower(pivot + vertical - horizontal);
        br.setPower(pivot +vertical + horizontal);
        bl.setPower(pivot - vertical + horizontal);
        fl.setPower(pivot + -vertical - horizontal);
    }
}