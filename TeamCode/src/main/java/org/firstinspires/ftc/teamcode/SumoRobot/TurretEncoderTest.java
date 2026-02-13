package org.firstinspires.ftc.teamcode.SumoRobot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name = "TURRET ENCODER TEST", group = "Test")
public class TurretEncoderTest extends OpMode {

    private DcMotorEx turretMotor;

    // Motor constants (same as your main code)
    private static final double COUNTS_PER_MOTOR_REV = 383.6;
    private static final double GEAR_RATIO = 3.92;
    private static final double COUNTS_PER_DEGREE = (COUNTS_PER_MOTOR_REV * GEAR_RATIO) / 360.0;

    // Test modes
    private enum Mode {
        MANUAL,      // Manual power with gamepad
        RUN_TO_POS   // RUN_TO_POSITION test
    }
    private Mode currentMode = Mode.MANUAL;
    private int testTargetTicks = 0;

    // For toggling direction setting
    private boolean directionForward = true;

    @Override
    public void init() {
        turretMotor = hardwareMap.get(DcMotorEx.class, "Turret");

        // Start with clean reset
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Default to FORWARD - we'll test both
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "INITIALIZED - Point turret RIGHT, then press START");
        telemetry.addData("Direction", "FORWARD (press A to toggle)");
        telemetry.addData("Instructions", "");
        telemetry.addData("LEFT STICK", "Manual power (Y axis)");
        telemetry.addData("A", "Toggle FORWARD/REVERSE direction");
        telemetry.addData("B", "Reset encoder to 0");
        telemetry.addData("X", "Test RUN_TO_POSITION mode");
        telemetry.addData("Y", "Return to MANUAL mode");
        telemetry.addData("DPAD UP", "Set target +90° (left)");
        telemetry.addData("DPAD DOWN", "Set target -90° (right)");
        telemetry.update();
    }

    @Override
    public void loop() {
        // ---------- BUTTON HANDLING ----------
        if (gamepad1.a) {
            toggleDirection();
            sleep(250); // Simple debounce
        }

        if (gamepad1.b) {
            resetEncoder();
            sleep(250);
        }

        if (gamepad1.x) {
            currentMode = Mode.RUN_TO_POS;
            telemetry.addData("MODE", "RUN_TO_POSITION");
            sleep(250);
        }

        if (gamepad1.y) {
            currentMode = Mode.MANUAL;
            turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            turretMotor.setPower(0);
            telemetry.addData("MODE", "MANUAL");
            sleep(250);
        }

        if (gamepad1.dpad_up) {
            testTargetTicks = (int)(90 * COUNTS_PER_DEGREE);
            if (!directionForward) testTargetTicks = -testTargetTicks;
            runToPositionTest(testTargetTicks);
            sleep(250);
        }

        if (gamepad1.dpad_down) {
            testTargetTicks = (int)(-90 * COUNTS_PER_DEGREE);
            if (!directionForward) testTargetTicks = -testTargetTicks;
            runToPositionTest(testTargetTicks);
            sleep(250);
        }

        // ---------- MOTOR CONTROL ----------
        if (currentMode == Mode.MANUAL) {
            // Manual power control with left stick Y
            double power = -gamepad1.left_stick_y; // Inverted so up = positive
            turretMotor.setPower(power);
        }

        // ---------- TELEMETRY - RAW VALUES ----------
        int rawTicks = turretMotor.getCurrentPosition();
        double rawAngle = rawTicks / COUNTS_PER_DEGREE;

        // Also calculate what your main code would show in each direction mode
        double mainCodeAngle_FORWARD = rawAngle; // No inversion
        double mainCodeAngle_REVERSE = -rawAngle; // Inverted

        telemetry.addData("========== RAW ENCODER ==========", "");
        telemetry.addData("Raw Encoder Ticks", rawTicks);
        telemetry.addData("Raw Angle (degrees)", "%.2f", rawAngle);
        telemetry.addData("", "");

        telemetry.addData("========== CURRENT SETTINGS ==========", "");
        telemetry.addData("Motor Direction", turretMotor.getDirection() == DcMotorSimple.Direction.FORWARD ? "FORWARD" : "REVERSE");
        telemetry.addData("Motor Mode", turretMotor.getMode());
        telemetry.addData("Test Mode", currentMode);
        telemetry.addData("Motor Power", "%.2f", turretMotor.getPower());
        telemetry.addData("", "");

        telemetry.addData("========== WHAT YOUR CODE WOULD SEE ==========", "");
        telemetry.addData("If Direction = FORWARD:", "Angle = %.2f°", mainCodeAngle_FORWARD);
        telemetry.addData("If Direction = REVERSE:", "Angle = %.2f° (inverted)", mainCodeAngle_REVERSE);
        telemetry.addData("", "");

        telemetry.addData("========== PHYSICAL POSITION ==========", "");
        if (Math.abs(rawAngle) < 10) {
            telemetry.addData("Position", "➡️ POINTING RIGHT (0°)");
        } else if (rawAngle > 80 && rawAngle < 100) {
            telemetry.addData("Position", "⬆️ POINTING FORWARD (90°)");
        } else if (rawAngle > 170 || rawAngle < -170) {
            telemetry.addData("Position", "⬅️ POINTING LEFT (180°)");
        } else if (rawAngle > 10) {
            telemetry.addData("Position", "↖️ LEFT OF CENTER (%.2f°)", rawAngle);
        } else if (rawAngle < -10) {
            telemetry.addData("Position", "↘️ RIGHT OF CENTER (%.2f°)", rawAngle);
        }

        if (currentMode == Mode.RUN_TO_POS) {
            telemetry.addData("", "");
            telemetry.addData("========== RUN_TO_POSITION TEST ==========", "");
            telemetry.addData("Target Ticks", testTargetTicks);
            telemetry.addData("Target Angle", "%.2f°", testTargetTicks / COUNTS_PER_DEGREE);
            telemetry.addData("Error (ticks)", testTargetTicks - rawTicks);
            telemetry.addData("Is Busy", turretMotor.isBusy());
        }

        telemetry.update();
    }

    private void toggleDirection() {
        if (turretMotor.getDirection() == DcMotorSimple.Direction.FORWARD) {
            turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            directionForward = false;
        } else {
            turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            directionForward = true;
        }
    }

    private void resetEncoder() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        currentMode = Mode.MANUAL;
    }

    private void runToPositionTest(int ticks) {
        currentMode = Mode.RUN_TO_POS;
        testTargetTicks = ticks;
        turretMotor.setTargetPosition(ticks);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(0.35);
    }

    private void sleep(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            // Ignore
        }
    }
}