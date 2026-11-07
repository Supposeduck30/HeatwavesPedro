package org.firstinspires.ftc.teamcode.biobuzz.teleop;

import static org.firstinspires.ftc.teamcode.biobuzz.teleop.BioBuzzConfig.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * Tune the shooter flywheel's P without redeploying for every guess. The flywheel runs at a FIXED
 * target (SHOOTER_FIXED_VELOCITY, the speed you shoot at) with F fixed at SHOOTER_F from BioBuzzConfig,
 * scaled for the battery voltage; only P changes. It drives the motor with RUN_USING_ENCODER +
 * setVelocity, like the teleop. Nothing is saved: copy the final P into SHOOTER_P in BioBuzzConfig.
 *
 * gamepad1 (Logitech names in brackets):
 *   cross (A)             = spin up / stop (the flywheel is OFF until you press it)
 *   dpad up / down        = P + / - one step
 *   dpad right / left     = bigger / smaller step (1, 5, 10)
 *   triangle (Y)          = restart the stats
 *   circle (B)            = HOLD to open the blocker (each press counts as one shot)
 *   right bumper          = intake + transfer, left bumper = intake only
 *
 * HOW TO TUNE:
 *   1. Tune F first with FlywheelFTuner. Press cross (A) to spin up. P starts at 0.
 *   2. Raise P until it reaches the target quickly. Watch "Idle peak-to-peak" (no balls): stop raising
 *      P as soon as it starts to grow; that is the wheel oscillating. Back off one step.
 *   3. Shot test, ONE ball at a time: hold right bumper so the ball rests against the blocker, wait for
 *      READY, then press circle. "Speed at shot" records the wheel speed at that moment. A small
 *      spread (max - min) over several shots means every ball leaves at the same speed.
 *   4. Copy the "Paste into BioBuzzConfig" value over SHOOTER_P.
 */
@TeleOp(name = "Flywheel P Tuner", group = "BioBuzz")
public class FlywheelPIDFTuner extends OpMode {

    /** Fixed flywheel target while tuning, ticks/s: the speed you shoot at, so F and P are tuned there. */
    private static final double TARGET_VELOCITY = SHOOTER_FIXED_VELOCITY;
    private static final double[] STEPS = {1, 5, 10};
    /** P is never taken above this (Decode never needed more). */
    private static final double MAX_P = 125;
    /**
     * Ticks/s band for "settled", READY and dips. Tighter than SHOOTER_READY_TOLERANCE so an oscillating
     * wheel does not count as settled. The hub reports this motor's speed in steps of 20 ticks/s, so
     * this must be more than one step (20) or it can never settle.
     */
    private static final double TOLERANCE = 25;
    /** The speed has to stay inside TOLERANCE this long to count as settled. */
    private static final double SETTLE_HOLD_SEC = 0.5;
    /** Idle stats skip this long after the blocker was open, so a shot's recovery is not counted. */
    private static final double IDLE_AFTER_SHOT_SEC = 1.0;
    /** Idle stats skip this long after spin-up / any P change, settled or not. */
    private static final double IDLE_AFTER_START_SEC = 2.0;
    /** How often the battery voltage is read and the compensated F re-sent to the hub. */
    private static final double VOLTAGE_UPDATE_SEC = 0.5;
    /** Voltage low-pass filter (0..1, lower = smoother). The reading sags while the wheel spins up. */
    private static final double VOLTAGE_ALPHA = 0.2;
    /** Ignore readings below this (sensor glitch / brownout) so the compensation can't explode. */
    private static final double MIN_VALID_VOLTAGE = 9.0;

    private DcMotorEx shooter;
    private DcMotor intake, transfer;
    private Servo blocker;
    private VoltageSensor battery;

    private double p = 0;
    private int stepIndex = 1; // starts at 5
    private boolean spinning = false;

    private double voltage = SHOOTER_F_NOMINAL_VOLTAGE;
    private long lastVoltageNs;

    // Step response / error stats, restarted whenever P or the spin state changes
    private long runStartNs;
    private double reachSec, settleSec, overshoot;
    private long inBandSinceNs;
    private boolean settled;
    private double sumErr, sumSqErr, maxDip, peakAbsErr;
    private int samples;
    // Idle: speed range while no ball is being shot
    private double idleMin, idleMax;
    // Shot test: speed when the blocker opens, how far it drops and how long it takes to come back
    private int shots;
    private double lastShotSpeed, shotMin, shotMax;
    private long lastBlockerOpenNs;
    private double lowestSpeed;
    private boolean recovering;
    private long dipStartNs;
    private double lastRecoverySec, worstRecoverySec;

    private boolean lastUp, lastDown, lastLeft, lastRight, lastCross, lastTriangle, lastCircle;

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
        blocker.setPosition(BLOCKER_BLOCK);
        if (hardwareMap.voltageSensor.iterator().hasNext()) {
            battery = hardwareMap.voltageSensor.iterator().next();
            double v = battery.getVoltage();
            if (v >= MIN_VALID_VOLTAGE) voltage = v;
        }
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
        boolean cross = gamepad1.cross, triangle = gamepad1.triangle, circle = gamepad1.circle;

        if (right && !lastRight) stepIndex = Math.min(stepIndex + 1, STEPS.length - 1);
        if (left && !lastLeft) stepIndex = Math.max(stepIndex - 1, 0);

        double delta = 0;
        if (up && !lastUp) delta = STEPS[stepIndex];
        if (down && !lastDown) delta = -STEPS[stepIndex];
        if (delta != 0) {
            p = Range.clip(p + delta, 0, MAX_P);
            applyCoefficients();
            resetStats();
        }

        if (cross && !lastCross) {
            spinning = !spinning;
            resetStats();
        }
        if (triangle && !lastTriangle) resetStats();

        /* ---------- MOTOR ---------- */
        updateVoltage(now);
        double velocity = shooter.getVelocity();
        shooter.setVelocity(spinning ? TARGET_VELOCITY : 0);

        // Each blocker press is one shot: record the wheel speed the ball is released into.
        if (circle && !lastCircle && spinning) {
            shots++;
            lastShotSpeed = velocity;
            shotMin = Math.min(shotMin, velocity);
            shotMax = Math.max(shotMax, velocity);
        }
        if (circle) lastBlockerOpenNs = now;

        if (spinning) updateStats(now, velocity);

        lastUp = up; lastDown = down; lastLeft = left; lastRight = right;
        lastCross = cross; lastTriangle = triangle; lastCircle = circle;

        /* ---------- FEED (same controls as the teleop, on gamepad1) ---------- */
        blocker.setPosition(circle ? BLOCKER_OPEN : BLOCKER_BLOCK);
        double intakePower = 0, transferPower = 0;
        if (gamepad1.left_bumper) {
            intakePower = INTAKE_IN_POWER;
        } else if (gamepad1.right_bumper) {
            intakePower = PIDF_INTAKE_TRANSFER_POWER;
            transferPower = PIDF_INTAKE_TRANSFER_POWER;
        }
        intake.setPower(intakePower);
        transfer.setPower(transferPower);

        /* ---------- TELEMETRY ---------- */
        double error = TARGET_VELOCITY - velocity;

        telemetry.addData("Flywheel", spinning ? "SPINNING" : "OFF  (cross / A to spin)");
        telemetry.addData("Target (ticks/s)", "%.0f  (fixed)", TARGET_VELOCITY);
        telemetry.addData("Actual (ticks/s)", "%.0f", velocity);
        telemetry.addData("Error now", "%.0f  (%s)", spinning ? error : 0,
                !spinning ? "off" : Math.abs(error) <= TOLERANCE ? "READY" : error > 0 ? "too slow" : "too fast");

        telemetry.addLine();
        telemetry.addData("P", "%.0f   (dpad up / down)", p);
        telemetry.addData("Step", "%.0f   (dpad right = bigger, left = smaller)", STEPS[stepIndex]);
        telemetry.addData("F", "%.3f config -> %.3f applied at %.2f V%s", SHOOTER_F, compensatedF(), voltage,
                battery == null ? "  (NO VOLTAGE SENSOR)" : "");

        telemetry.addLine();
        telemetry.addData("--- RESPONSE (stats restart when P changes) ---", "");
        telemetry.addData("Reach target", seconds(reachSec));
        telemetry.addData("Settle time", seconds(settleSec));
        telemetry.addData("Overshoot (ticks/s)", spinning ? String.format("%.0f", overshoot) : "-");
        if (spinning && samples > 0) {
            telemetry.addData("--- IDLE (no balls, since the last P change / triangle) ---", "");
            telemetry.addData("AVG ERROR (ticks/s)", "%.1f   (want near 0)", sumErr / samples);
            telemetry.addData("ERROR RMS", "%.1f   (want small; grows when it oscillates)", Math.sqrt(sumSqErr / samples));
            telemetry.addData("Idle peak-to-peak", "%s", idlePeakToPeak());
            telemetry.addData("Peak |error|", "%.0f", peakAbsErr);
        } else {
            telemetry.addData("Idle stats", spinning ? "spinning up..." : "-");
        }

        telemetry.addLine();
        telemetry.addData("--- SHOT TEST (one ball: hold right bumper, wait READY, press circle) ---", "");
        telemetry.addData("Shots", "%d", shots);
        if (shots > 0) {
            telemetry.addData("SPEED AT SHOT (last)", "%.0f", lastShotSpeed);
            telemetry.addData("Speed at shot min / max", "%.0f / %.0f", shotMin, shotMax);
            telemetry.addData("SHOT SPREAD (max - min)", "%.0f", shotMax - shotMin);
        }
        if (spinning && (now - runStartNs) / 1e9 >= IDLE_AFTER_START_SEC) {
            telemetry.addData("Max dip (ticks/s below target)", "%.0f", maxDip);
            telemetry.addData("Lowest speed", "%.0f", lowestSpeed);
            telemetry.addData("Recovery time (last / worst)", "%s / %s%s",
                    seconds(lastRecoverySec), seconds(worstRecoverySec), recovering ? "  (recovering now...)" : "");
        }

        telemetry.addLine();
        telemetry.addData("SEND TO CLAUDE", "P=%.0f F=%.3f V=%.2f avgErr=%s rms=%s settle=%s overshoot=%.0f idleP2P=%s shots=%d spread=%s maxDip=%s recovery=%s",
                p, SHOOTER_F, voltage,
                samples > 0 ? String.format("%.1f", sumErr / samples) : "-",
                samples > 0 ? String.format("%.1f", Math.sqrt(sumSqErr / samples)) : "-",
                seconds(settleSec), overshoot, idlePeakToPeak(), shots,
                shots > 0 ? String.format("%.0f", shotMax - shotMin) : "-",
                shots > 0 ? String.format("%.0f", maxDip) : "-",
                seconds(worstRecoverySec));
        telemetry.addData("Paste into BioBuzzConfig", "SHOOTER_P = %.0f", p);
        telemetry.update();
    }

    @Override
    public void stop() {
        shooter.setVelocity(0);
        intake.setPower(0);
        transfer.setPower(0);
    }

    /** SHOOTER_F is defined at SHOOTER_F_NOMINAL_VOLTAGE; a lower battery needs proportionally more. */
    private double compensatedF() {
        return SHOOTER_F * SHOOTER_F_NOMINAL_VOLTAGE / voltage;
    }

    private void updateVoltage(long now) {
        if (battery == null || (now - lastVoltageNs) / 1e9 < VOLTAGE_UPDATE_SEC) return;
        double v = battery.getVoltage();
        if (v >= MIN_VALID_VOLTAGE) voltage += VOLTAGE_ALPHA * (v - voltage);
        applyCoefficients();
    }

    private void applyCoefficients() {
        lastVoltageNs = System.nanoTime();
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(p, 0, 0, compensatedF()));
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
        idleMin = Double.MAX_VALUE;
        idleMax = -Double.MAX_VALUE;
        shots = 0;
        lastShotSpeed = 0;
        shotMin = Double.MAX_VALUE;
        shotMax = -Double.MAX_VALUE;
        lowestSpeed = Double.MAX_VALUE;
        recovering = false;
        lastRecoverySec = -1;
        worstRecoverySec = -1;
    }

    private void updateStats(long now, double velocity) {
        double error = TARGET_VELOCITY - velocity;
        boolean inBand = Math.abs(error) <= TOLERANCE;

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
        }

        // Everything below skips the spin-up, but does not wait for "settled": at low P the speed can
        // sit off the target and never settle, and these stats are what tell you that.
        if ((now - runStartNs) / 1e9 < IDLE_AFTER_START_SEC) return;

        // Idle: no ball near the wheel (blocker closed, and a shot's recovery is over).
        if ((now - lastBlockerOpenNs) / 1e9 >= IDLE_AFTER_SHOT_SEC) {
            sumErr += error;
            sumSqErr += error * error;
            samples++;
            peakAbsErr = Math.max(peakAbsErr, Math.abs(error));
            idleMin = Math.min(idleMin, velocity);
            idleMax = Math.max(idleMax, velocity);
        }

        maxDip = Math.max(maxDip, error);
        lowestSpeed = Math.min(lowestSpeed, velocity);

        // A dip starts when the speed falls more than the tolerance below target and ends when it is back within it.
        if (!recovering && error > TOLERANCE) {
            recovering = true;
            dipStartNs = now;
        } else if (recovering && error <= TOLERANCE) {
            recovering = false;
            lastRecoverySec = (now - dipStartNs) / 1e9;
            worstRecoverySec = Math.max(worstRecoverySec, lastRecoverySec);
        }
    }

    private String idlePeakToPeak() {
        return idleMax < idleMin ? "-" : String.format("%.0f", idleMax - idleMin);
    }

    private static String seconds(double s) {
        return s < 0 ? "-" : String.format("%.2f s", s);
    }
}
