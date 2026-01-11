package org.firstinspires.ftc.teamcode.SumoRobot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class SUMOTELEOP extends OpMode {

    // Drive motors
    private DcMotor fr, fl, br, bl;

    // Mechanisms
    private DcMotorEx shooter1, shooter2;
    private DcMotor intake;
    private Servo kicker;

    // Shooter velocities
    public static final double LOW_VELOCITY  = 1700;
    public static final double HIGH_VELOCITY = 2300;

    // Kicker positions
    public static final double KICKER_OUT = 0.60;
    public static final double KICKER_IN  = 0.31;

    private boolean kickerActive = false;

    @Override
    public void init() {

        // Drive motors
        fr = hardwareMap.get(DcMotor.class, "FR");
        fl = hardwareMap.get(DcMotor.class, "FL");
        br = hardwareMap.get(DcMotor.class, "BR");
        bl = hardwareMap.get(DcMotor.class, "BL");

        // Other motors
        shooter1 = hardwareMap.get(DcMotorEx.class, "Shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "Shooter2");
        intake   = hardwareMap.get(DcMotor.class, "Intake");
        kicker   = hardwareMap.get(Servo.class, "Kicker");

        // Drive directions
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);

        // Brake mode
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Shooter setup
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidf = new PIDFCoefficients(40, 0, 0, 25);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        kicker.setPosition(KICKER_IN);
    }

    @Override
    public void loop() {

        drive();

        // === KICKER CONTROL ===
        kickerActive = gamepad2.dpad_up;
        kicker.setPosition(kickerActive ? KICKER_OUT : KICKER_IN);

        boolean lowGoal  = gamepad2.left_bumper;
        boolean highGoal = gamepad2.right_bumper;

        if (highGoal) {
            setShooterVelocity(HIGH_VELOCITY);
            intake.setPower(kickerActive ? 0.0 : 1.0);

        } else if (lowGoal) {
            setShooterVelocity(LOW_VELOCITY);
            intake.setPower(kickerActive ? 0.0 : 1.0);

        } else if (gamepad2.triangle) {
            setShooterVelocity(LOW_VELOCITY);
            intake.setPower(0.0);

        } else {
            stopShooter();
            intake.setPower(0.0);
        }
    }

    // =========================
    // Helper Methods
    // =========================

    private void setShooterVelocity(double velocity) {
        shooter1.setVelocity(velocity);
        shooter2.setVelocity(velocity);
    }

    private void stopShooter() {
        shooter1.setVelocity(0);
        shooter2.setVelocity(0);
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
