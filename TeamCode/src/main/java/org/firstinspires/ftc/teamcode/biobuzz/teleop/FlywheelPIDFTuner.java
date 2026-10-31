package org.firstinspires.ftc.teamcode.biobuzz.teleop;

import static org.firstinspires.ftc.teamcode.biobuzz.teleop.BioBuzzConfig.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

/**
 * Tune the shooter flywheel's P without redeploying for every guess. The flywheel runs at a FIXED
 * target (TARGET_VELOCITY) with F fixed at SHOOTER_F from BioBuzzConfig; only P changes. It drives the
 * motor the same way BioBuzzTeleOpBase does (RUN_USING_ENCODER + setVelocity), so what works here
 * works there. Nothing is saved: copy the final P into SHOOTER_P in BioBuzzConfig.
 *
 * gamepad1 (Logitech names in brackets):
 *   cross (A)             = spin up / stop (the flywheel is OFF until you press it)
 *   dpad up / down        = P + / - one step
 *   dpad right / left     = bigger / smaller step (0.5, 1, 5, 10)
 *   triangle (Y)          = restart the error stats (use it before feeding balls to measure the dip)
 *
 * HOW TO TUNE:
 *   1. Press cross (A) to spin up. With P at 0 the speed will sit a bit off the target.
 *   2. Raise P with a big step, then a small one, until the error is small and it reaches the
 *      target quickly. Stop the moment the actual speed starts to oscillate or buzz.
 *   3. Judge it on the stats: small "Settle time", small "Overshoot", "Avg error" near 0, small "Error RMS".
 *      Feed a few balls (press triangle first) and watch "Max dip" shrink.
 *   4. Copy the "Paste into BioBuzzConfig" value over SHOOTER_P.
 */
@TeleOp(name = "Flywheel P Tuner", group = "BioBuzz")
public class FlywheelPIDFTuner extends OpMode {

    /** Fixed flywheel target while tuning, ticks/s. Follows the teleop's fixed shooter speed. */
    private static final double TARGET_VELOCITY = SHOOTER_FIXED_VELOCITY;
    private static final double[] STEPS = {0.5, 1, 5, 10};
    /** P is never taken above this (Decode never needed more). */
    private static final double MAX_P = 125;
    /** The speed has to stay inside SHOOTER_READY_TOLERANCE this long to count as settled. */
    private static final double SETTLE_HOLD_SEC = 0.5;

    private DcMotorEx shooter;

    private double p = Math.min(SHOOTER_P, MAX_P);
    private int stepIndex = 2; // starts at step 5
    private boolean spinning = false;

    // Step response / error stats, restarted whenever P or the spin state changes
    private long runStartNs;
    private double reachSec, settleSec, overshoot;
    private long inBandSinceNs;
    private boolean settled;
    private double sumErr, sumSqErr, maxDip, peakAbsErr;
    private int samples;

    private boolean lastUp, lastDown, lastLeft, lastRight, lastCross, lastTriangle;

    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, SHOOTER);
        shooter.setDirection(SHOOTER_DIR);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        applyCoefficients();
        resetStats();

        telemetry.addLine("Flywheel P Tuner ready. Flywheel is OFF; press cross (A) to spin.");
        telemetry.update();
    }

    @Override
    public void loop() {
        long now = System.nanoTime();

        /* ---------- INPUT ---------- */
        boolean up = gamepad1.dpad_up, down = gamepad1.dpad_down;
        boolean left = gamepad1.dpad_left, right = gamepad1.dpad_right;
        boolean cross = gamepad1.cross, triangle = gamepad1.triangle;

        if (right && !lastRight) stepIndex = Math.min(stepIndex + 1, STEPS.length - 1);
        if (left && !lastLeft) stepIndex = Math.max(stepIndex - 1, 0);

        double delta = 0;
        if (up && !lastUp) delta = STEPS[stepIndex];
        if (down && !lastDown) delta = -STEPS[stepIndex];
        if (delta != 0) {
            p = Range.clip(Math.round((p + delta) * 10000.0) / 10000.0, 0, MAX_P);
            applyCoefficients();
            resetStats();
        }

        if (cross && !lastCross) {
            spinning = !spinning;
            resetStats();
        }
        if (triangle && !lastTriangle) resetStats();

        lastUp = up; lastDown = down; lastLeft = left; lastRight = right;
        lastCross = cross; lastTriangle = triangle;

        /* ---------- MOTOR ---------- */
        double velocity = shooter.getVelocity();
        shooter.setVelocity(spinning ? TARGET_VELOCITY : 0);
        if (spinning) updateStats(now, velocity);

        /* ---------- TELEMETRY ---------- */
        double error = TARGET_VELOCITY - velocity;

        telemetry.addData("Flywheel", spinning ? "SPINNING" : "OFF  (cross / A to spin)");
        telemetry.addData("Target (ticks/s)", "%.0f  (fixed)", TARGET_VELOCITY);
        telemetry.addData("Actual (ticks/s)", "%.0f", velocity);
        telemetry.addData("Error now", "%.0f  (%s)", spinning ? error : 0,
                !spinning ? "off" : Math.abs(error) <= SHOOTER_READY_TOLERANCE ? "READY" : error > 0 ? "too slow" : "too fast");

        telemetry.addLine();
        telemetry.addData("P", "%.2f   (dpad up / down)", p);
        telemetry.addData("Step", "%s   (dpad right = bigger, left = smaller)", formatStep(STEPS[stepIndex]));
        telemetry.addData("F (fixed from config)", "%.2f", SHOOTER_F);

        telemetry.addLine();
        telemetry.addData("--- RESPONSE (stats restart when P changes) ---", "");
        telemetry.addData("Reach target", seconds(reachSec));
        telemetry.addData("Settle time", seconds(settleSec));
        telemetry.addData("Overshoot (ticks/s)", spinning ? String.format("%.0f", overshoot) : "-");
        if (settled && samples > 0) {
            telemetry.addData("Avg error (ticks/s)", "%.1f", sumErr / samples);
            telemetry.addData("Error RMS", "%.1f", Math.sqrt(sumSqErr / samples));
            telemetry.addData("Max dip (too slow)", "%.0f", maxDip);
            telemetry.addData("Peak |error|", "%.0f", peakAbsErr);
        } else {
            telemetry.addData("Steady-state stats", spinning ? "waiting to settle..." : "-");
        }

        telemetry.addLine();
        telemetry.addData("Paste into BioBuzzConfig", "SHOOTER_P = %s", trim(p));
        telemetry.update();
    }

    @Override
    public void stop() {
        shooter.setVelocity(0);
    }

    private void applyCoefficients() {
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(p, 0, 0, SHOOTER_F));
    }

    private void resetStats() {
        runStartNs = System.nanoTime();
        reachSec = -1;
        settleSec = -1;
        overshoot = 0;
        inBandSinceNs = -1;
        settled = false;
        sumErr = 0;
        sumSqErr = 0;
        maxDip = 0;
        peakAbsErr = 0;
        samples = 0;
    }

    private void updateStats(long now, double velocity) {
        double error = TARGET_VELOCITY - velocity;
        boolean inBand = Math.abs(error) <= SHOOTER_READY_TOLERANCE;

        if (reachSec < 0 && inBand) {
            reachSec = (now - runStartNs) / 1e9;
        }
        overshoot = Math.max(overshoot, velocity - TARGET_VELOCITY);

        if (!settled) {
            if (!inBand) {
                inBandSinceNs = -1;
            } else if (inBandSinceNs < 0) {
                inBandSinceNs = now;
            } else if ((now - inBandSinceNs) / 1e9 >= SETTLE_HOLD_SEC) {
                settled = true;
                settleSec = (inBandSinceNs - runStartNs) / 1e9;
            }
            return;
        }

        // Settled: everything from here on is the error you would see while shooting.
        sumErr += error;
        sumSqErr += error * error;
        samples++;
        maxDip = Math.max(maxDip, error);
        peakAbsErr = Math.max(peakAbsErr, Math.abs(error));
    }

    private static String seconds(double s) {
        return s < 0 ? "-" : String.format("%.2f s", s);
    }

    private static String formatStep(double step) {
        return step < 1 ? String.format("%.1f", step) : String.format("%.0f", step);
    }

    /** Short number for the paste line: 0.0 -> "0", 12.5 -> "12.5". */
    private static String trim(double v) {
        String s = String.format("%.4f", v);
        return s.replaceAll("0+$", "").replaceAll("\\.$", "");
    }
}
