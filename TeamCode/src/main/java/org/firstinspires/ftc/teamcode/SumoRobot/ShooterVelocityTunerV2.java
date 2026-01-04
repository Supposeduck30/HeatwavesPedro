package org.firstinspires.ftc.teamcode.SumoRobot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Shooter Velocity Tuner")
public class ShooterVelocityTunerV2 extends OpMode {

    DcMotorEx shooter1, shooter2;
    DcMotor intake;
    Servo kicker;

    // ---------------- TUNING VALUES ----------------
    double targetVelocity = 4000;   // ticks/sec
    double P = 0.0003;
    double F = 0.00023;              // ~1 / maxVelocity

    double[] stepSizes = {0.00001, 0.00005, 0.0001, 0.0005};
    int stepIndex = 1;

    boolean pidChanged = true;

    @Override
    public void init() {

        shooter1 = hardwareMap.get(DcMotorEx.class, "Shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "Shooter2");
        intake   = hardwareMap.get(DcMotor.class, "Intake");
        kicker   = hardwareMap.get(Servo.class, "Kicker");

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // IMPORTANT: do NOT reverse velocity motors
        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);

        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addLine("Shooter Velocity Tuner Ready");
    }

    @Override
    public void loop() {

        /* ---------------- Intake ---------------- */
        intake.setPower(gamepad1.left_bumper ? 1.0 : 0.0);

        /* ---------------- Kicker ---------------- */
        kicker.setPosition(gamepad1.right_bumper ? 0.6 : 0.3);

        /* ---------------- Target Velocity ---------------- */
        if (gamepad1.y) targetVelocity = 4000;
        if (gamepad1.a) targetVelocity = 2000;

        /* ---------------- Step Size ---------------- */
        if (gamepad1.b) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
            sleep(200);
        }

        double step = stepSizes[stepIndex];

        /* ---------------- PIDF Tuning ---------------- */
        if (gamepad1.dpad_up) {
            P += step;
            pidChanged = true;
        }
        if (gamepad1.dpad_down) {
            P -= step;
            pidChanged = true;
        }
        if (gamepad1.dpad_left) {
            F += step;
            pidChanged = true;
        }
        if (gamepad1.dpad_right) {
            F -= step;
            pidChanged = true;
        }

        /* ---------------- Apply PIDF ONLY if changed ---------------- */
        if (pidChanged) {
            shooter1.setVelocityPIDFCoefficients(P, 0, 0, F);
            shooter2.setVelocityPIDFCoefficients(P, 0, 0, F);
            pidChanged = false;
        }

        /* ---------------- Run Shooter ---------------- */
        shooter1.setVelocity(targetVelocity);
        shooter2.setVelocity(targetVelocity);

        /* ---------------- Telemetry ---------------- */
        double velocity = shooter1.getVelocity();
        double error = targetVelocity - velocity;

        telemetry.addData("Target (ticks/s)", targetVelocity);
        telemetry.addData("Velocity", "%.1f", velocity);
        telemetry.addData("Error", "%.1f", error);
        telemetry.addLine("----------------------");
        telemetry.addData("P", "%.6f", P);
        telemetry.addData("F", "%.6f", F);
        telemetry.addData("Step", "%.6f", step);
        telemetry.update();
    }

    private void sleep(long ms) {
        try { Thread.sleep(ms); } catch (Exception ignored) {}
    }
}
