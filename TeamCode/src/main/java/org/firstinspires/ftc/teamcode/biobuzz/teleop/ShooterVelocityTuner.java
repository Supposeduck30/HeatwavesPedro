package org.firstinspires.ftc.teamcode.biobuzz.teleop;

import static org.firstinspires.ftc.teamcode.biobuzz.teleop.BioBuzzConfig.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Set the shooter velocity by hand and shoot to see what works. No driving, no scoring: change the
 * velocity, shoot, look at where the ball goes, repeat. The shooter uses SHOOTER_P / I / D / F from
 * BioBuzzConfig, same as the teleop. Starts at SHOOTER_FIXED_VELOCITY with the flywheel spinning.
 *
 * Every control works on either gamepad (same layout as the teleop's gamepad2):
 *   dpad up / down        = velocity + / - one step
 *   dpad right / left     = bigger / smaller step (10, 25, 50, 100; starts at 50)
 *   triangle (hold)       = open blocker
 *   right bumper          = intake + transfer
 *   left bumper           = intake only
 *   cross                 = reverse both
 *   square                = flywheel on / off
 *
 * When you find a velocity you like, copy the "Paste into BioBuzzConfig" value over
 * SHOOTER_FIXED_VELOCITY.
 */
@TeleOp(name = "Shooter Velocity Tuner", group = "BioBuzz")
public class ShooterVelocityTuner extends OpMode {

    private static final double[] STEPS = {10, 25, 50, 100};

    private DcMotorEx shooter;
    private DcMotor intake, transfer;
    private Servo blocker;

    private double target = SHOOTER_FIXED_VELOCITY;
    private int stepIndex = 2; // starts at step 50
    private boolean spinning = true;

    private boolean lastUp, lastDown, lastLeft, lastRight, lastSquare;

    @Override
    public void init() {
        shooter  = hardwareMap.get(DcMotorEx.class, SHOOTER);
        intake   = hardwareMap.get(DcMotor.class, INTAKE);
        transfer = hardwareMap.get(DcMotor.class, TRANSFER);
        blocker  = hardwareMap.get(Servo.class, BLOCKER);

        shooter.setDirection(SHOOTER_DIR);
        intake.setDirection(INTAKE_DIR);
        transfer.setDirection(TRANSFER_DIR);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F));

        blocker.setPosition(BLOCKER_BLOCK);

        telemetry.addLine("Shooter Velocity Tuner ready. Flywheel starts spinning on start.");
        telemetry.update();
    }

    @Override
    public void loop() {
        /* ---------- VELOCITY INPUT (either gamepad) ---------- */
        boolean up = gamepad1.dpad_up || gamepad2.dpad_up;
        boolean down = gamepad1.dpad_down || gamepad2.dpad_down;
        boolean left = gamepad1.dpad_left || gamepad2.dpad_left;
        boolean right = gamepad1.dpad_right || gamepad2.dpad_right;
        boolean square = gamepad1.square || gamepad2.square;

        if (right && !lastRight) stepIndex = Math.min(stepIndex + 1, STEPS.length - 1);
        if (left && !lastLeft) stepIndex = Math.max(stepIndex - 1, 0);
        if (up && !lastUp) target = Range.clip(target + STEPS[stepIndex], 0, MAX_SHOOTER_VELOCITY);
        if (down && !lastDown) target = Range.clip(target - STEPS[stepIndex], 0, MAX_SHOOTER_VELOCITY);
        if (square && !lastSquare) spinning = !spinning;

        lastUp = up; lastDown = down; lastLeft = left; lastRight = right; lastSquare = square;

        shooter.setVelocity(spinning ? target : 0);

        /* ---------- BLOCKER ---------- */
        boolean blockerOpen = gamepad1.triangle || gamepad2.triangle;
        blocker.setPosition(blockerOpen ? BLOCKER_OPEN : BLOCKER_BLOCK);

        /* ---------- INTAKE ---------- */
        double intakePower = 0, transferPower = 0;
        if (gamepad1.cross || gamepad2.cross) {
            intakePower = INTAKE_REVERSE_POWER;
            transferPower = TRANSFER_REVERSE_POWER;
        } else if (gamepad1.left_bumper || gamepad2.left_bumper) {
            intakePower = INTAKE_IN_POWER;
        } else if (gamepad1.right_bumper || gamepad2.right_bumper) {
            intakePower = INTAKE_IN_POWER;
            transferPower = TRANSFER_IN_POWER;
        }
        intake.setPower(intakePower);
        transfer.setPower(transferPower);

        /* ---------- TELEMETRY ---------- */
        double velocity = shooter.getVelocity();
        boolean ready = spinning && Math.abs(target - velocity) <= SHOOTER_READY_TOLERANCE;

        telemetry.addData("Flywheel", spinning ? (ready ? "READY" : "spinning up...") : "OFF  (square)");
        telemetry.addData("Target (ticks/s)", "%.0f", target);
        telemetry.addData("Actual (ticks/s)", "%.0f", velocity);
        telemetry.addData("Error", "%.0f", spinning ? target - velocity : 0);
        telemetry.addData("Step", "%.0f   (dpad right = bigger, left = smaller)", STEPS[stepIndex]);
        telemetry.addData("Blocker", blockerOpen ? "OPEN" : "BLOCKING");
        telemetry.addData("Intake / Transfer power", "%.1f / %.1f", intakePower, transferPower);
        telemetry.addData("Paste into BioBuzzConfig", "SHOOTER_FIXED_VELOCITY = %.0f", target);
        telemetry.update();
    }

    @Override
    public void stop() {
        shooter.setVelocity(0);
        intake.setPower(0);
        transfer.setPower(0);
    }
}
