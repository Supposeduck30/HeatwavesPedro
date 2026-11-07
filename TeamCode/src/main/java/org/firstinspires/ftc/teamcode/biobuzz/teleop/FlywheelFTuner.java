package org.firstinspires.ftc.teamcode.biobuzz.teleop;

import static org.firstinspires.ftc.teamcode.biobuzz.teleop.BioBuzzConfig.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * Find the shooter flywheel's F (feedforward) on its own, with P = I = D = 0 so F is the only thing
 * driving the motor. Run this BEFORE FlywheelPIDFTuner, then tune P on top of the F you find here.
 * Nothing is saved: copy the final F into SHOOTER_F in BioBuzzConfig.
 *
 * gamepad1 (Logitech names in brackets):
 *   cross (A)             = spin up / stop (the flywheel is OFF until you press it)
 *   dpad up / down        = F + / - one step
 *   dpad right / left     = bigger / smaller F step (0.01, 0.05, 0.1, 0.5)
 *   right / left bumper   = target + / - 10 ticks/s
 *   square (X)            = apply the SUGGESTED F (jump straight to the computed value)
 *   triangle (Y)          = restart the measurement
 *
 * HOW TO USE:
 *   1. Use a fully charged battery. Press cross (A) to spin up.
 *   2. Wait for "Measuring" to fill in (it ignores the first SPINUP_SEC seconds while the wheel speeds up).
 *   3. Read "Avg speed". Press square (X) to jump F to the suggested value, or nudge it with the dpad.
 *   4. Repeat until Avg speed is within a few ticks/s of the target, then copy the
 *      "Paste into BioBuzzConfig" value over SHOOTER_F.
 *
 * Why the suggestion works: with P = 0 the motor speed is roughly proportional to F, so
 * F_new = F_old * target / average speed.
 */
@TeleOp(name = "Flywheel F Tuner", group = "BioBuzz")
public class FlywheelFTuner extends OpMode {

    /** Ignore this long after spin-up / any change while the wheel speeds up. */
    private static final double SPINUP_SEC = 2.0;
    /** Need at least this many samples before a result is shown. */
    private static final int MIN_SAMPLES = 50;
    private static final double[] STEPS = {0.01, 0.05, 0.1, 0.5};
    private static final double TARGET_STEP = 10;
    private static final double MAX_F = 40;

    private DcMotorEx shooter;
    private VoltageSensor battery;

    private double target = SHOOTER_FIXED_VELOCITY;
    private double f = SHOOTER_F;
    private int stepIndex = 1; // starts at 0.05
    private boolean spinning = false;

    // Measurement, restarted whenever F, the target or the spin state changes
    private long startNs;
    private double sum, sumSq, minSpeed, maxSpeed;
    private int samples;

    private boolean lastUp, lastDown, lastLeft, lastRight, lastCross, lastTriangle, lastSquare;
    private boolean lastLb, lastRb;

    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, SHOOTER);
        shooter.setDirection(SHOOTER_DIR);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (hardwareMap.voltageSensor.iterator().hasNext()) {
            battery = hardwareMap.voltageSensor.iterator().next();
        }
        applyCoefficients();
        resetStats();

        telemetry.addLine("Flywheel F Tuner ready. Flywheel is OFF; press cross (A) to spin.");
        telemetry.update();
    }

    @Override
    public void loop() {
        long now = System.nanoTime();

        /* ---------- INPUT ---------- */
        boolean up = gamepad1.dpad_up, down = gamepad1.dpad_down;
        boolean left = gamepad1.dpad_left, right = gamepad1.dpad_right;
        boolean cross = gamepad1.cross, triangle = gamepad1.triangle, square = gamepad1.square;
        boolean lb = gamepad1.left_bumper, rb = gamepad1.right_bumper;

        if (right && !lastRight) stepIndex = Math.min(stepIndex + 1, STEPS.length - 1);
        if (left && !lastLeft) stepIndex = Math.max(stepIndex - 1, 0);

        double delta = 0;
        if (up && !lastUp) delta = STEPS[stepIndex];
        if (down && !lastDown) delta = -STEPS[stepIndex];
        if (delta != 0) {
            setF(f + delta);
        }

        if (square && !lastSquare) {
            double suggested = suggestedF();
            if (suggested > 0) setF(suggested);
        }

        if ((rb && !lastRb) || (lb && !lastLb)) {
            target = Math.max(0, target + (rb && !lastRb ? TARGET_STEP : -TARGET_STEP));
            resetStats();
        }

        if (cross && !lastCross) {
            spinning = !spinning;
            resetStats();
        }
        if (triangle && !lastTriangle) resetStats();

        lastUp = up; lastDown = down; lastLeft = left; lastRight = right;
        lastCross = cross; lastTriangle = triangle; lastSquare = square;
        lastLb = lb; lastRb = rb;

        /* ---------- MOTOR ---------- */
        double velocity = shooter.getVelocity();
        shooter.setVelocity(spinning ? target : 0);

        boolean measuring = spinning && (now - startNs) / 1e9 >= SPINUP_SEC;
        if (measuring) {
            sum += velocity;
            sumSq += velocity * velocity;
            minSpeed = Math.min(minSpeed, velocity);
            maxSpeed = Math.max(maxSpeed, velocity);
            samples++;
        }

        /* ---------- TELEMETRY ---------- */
        telemetry.addData("Flywheel", spinning ? "SPINNING" : "OFF  (cross / A to spin)");
        telemetry.addData("Target (ticks/s)", "%.0f   (bumpers +/- %.0f)", target, TARGET_STEP);
        telemetry.addData("Actual now (ticks/s)", "%.0f", velocity);
        if (battery != null) telemetry.addData("Battery (V)", "%.2f", battery.getVoltage());

        telemetry.addLine();
        telemetry.addData("F", "%.3f   (dpad up / down)", f);
        telemetry.addData("Step", "%.2f   (dpad right = bigger, left = smaller)", STEPS[stepIndex]);
        telemetry.addData("P / I / D", "0 / 0 / 0  (fixed for this tuner)");

        telemetry.addLine();
        if (!spinning) {
            telemetry.addData("Measuring", "-");
        } else if (!measuring) {
            telemetry.addData("Measuring", "spinning up... %.1f s", Math.max(0, SPINUP_SEC - (now - startNs) / 1e9));
        } else if (samples < MIN_SAMPLES) {
            telemetry.addData("Measuring", "collecting samples (%d/%d)...", samples, MIN_SAMPLES);
        } else {
            double avg = sum / samples;
            double sd = Math.sqrt(Math.max(0, sumSq / samples - avg * avg));
            double err = target - avg;
            telemetry.addData("Avg speed (ticks/s)", "%.1f", avg);
            telemetry.addData("Avg error", "%.1f  (%s)", err,
                    Math.abs(err) <= 5 ? "GOOD" : err > 0 ? "too slow -> raise F" : "too fast -> lower F");
            telemetry.addData("Noise (std dev)", "%.1f", sd);
            telemetry.addData("Min / max speed", "%.0f / %.0f", minSpeed, maxSpeed);
            telemetry.addData("SUGGESTED F  (press square / X)", "%.3f", suggestedF());
        }

        telemetry.addLine();
        telemetry.addData("Paste into BioBuzzConfig", "SHOOTER_F = %s", trim(f));
        telemetry.update();
    }

    @Override
    public void stop() {
        shooter.setVelocity(0);
    }

    /** F_new = F_old * target / average speed. Returns -1 when there is no usable measurement yet. */
    private double suggestedF() {
        if (samples < MIN_SAMPLES) return -1;
        double avg = sum / samples;
        if (avg <= 0) return -1;
        return Math.min(MAX_F, f * target / avg);
    }

    private void setF(double newF) {
        f = Math.min(MAX_F, Math.max(0, Math.round(newF * 1000.0) / 1000.0));
        applyCoefficients();
        resetStats();
    }

    private void applyCoefficients() {
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(0, 0, 0, f));
    }

    private void resetStats() {
        startNs = System.nanoTime();
        sum = 0;
        sumSq = 0;
        minSpeed = Double.MAX_VALUE;
        maxSpeed = -Double.MAX_VALUE;
        samples = 0;
    }

    /** Short number for the paste line: 14.9000 -> "14.9". */
    private static String trim(double v) {
        String s = String.format("%.4f", v);
        return s.replaceAll("0+$", "").replaceAll("\\.$", "");
    }
}
