package org.firstinspires.ftc.teamcode.biobuzz.teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * Shooter graph test: shoot balls while Panels plots the wheel's ACTUAL speed against the TARGET, so you can
 * see whether P = 130 / F = 13.85 really hold 1450 and how fast the wheel recovers after each ball.
 * Standalone on purpose: copy this one file into any repo that has Panels (com.bylazar:fullpanels) and the same
 * robot config. Nothing here reads BioBuzzConfig; the numbers you may change are the constants below.
 *
 * VIEW IT: connect the laptop to the robot's Wi-Fi, open  http://192.168.43.1:8001  (Control Hub), add a GRAPH
 * widget and tick these keys (all numbers are plotted automatically):
 *   Graph 1 (ticks/s):  "Target", "Actual", "Shot gate"  (Shot gate is a step that goes UP while the blocker is open)
 *   Graph 2 (power):    "Power (hub)" and "Power (F only)"   (the gap between them is what P is adding)
 *   "Error" is Target - Actual if you want it on its own graph.
 *
 * gamepad1 (PS names; Logitech in brackets):
 *   cross (A)          = spin up / stop
 *   circle (B, hold)   = open the blocker (balls go into the wheel)
 *   right bumper       = intake + transfer, left bumper = intake only
 *   right trigger      = intake + transfer, BOTH at 0.9
 *   dpad up / down     = P + / - 5  (LIVE, so you can compare 120 vs 100 on the same graph)
 *   dpad right / left  = F + / - 0.05 (LIVE)
 *   triangle (Y)       = clear the shot stats
 *
 * WHAT TO LOOK FOR: after each ball "Actual" drops below "Target" and should climb back quickly with no
 * wobble. Wobble that keeps going after the ball is gone = P too high. A flat line that sits below Target
 * = F too low (or too high if above). "Time below band" is how long one burst kept the wheel out of the
 * +/- TOL band; the shots that came out during that time were slower than the rest.
 */
@TeleOp(name = "Shooter Graph Test", group = "BioBuzz")
public class ShooterGraphTest extends OpMode {

    // ================= COPY THESE FROM YOUR CONFIG =================
    private static final String SHOOTER = "shooter", INTAKE = "intake", TRANSFER = "transfer", BLOCKER = "blocker";
    private static final DcMotorSimple.Direction SHOOTER_DIR = DcMotorSimple.Direction.REVERSE;
    private static final DcMotorSimple.Direction INTAKE_DIR = DcMotorSimple.Direction.FORWARD;
    private static final DcMotorSimple.Direction TRANSFER_DIR = DcMotorSimple.Direction.REVERSE;
    private static final double BLOCKER_BLOCK = 0.49, BLOCKER_OPEN = 0.68;

    private static final double TARGET = 1450 + 0; // SHOOTER_FIXED_VELOCITY + PIDF_VELOCITY_OFFSET
    private static final double START_P = 130, START_F = 13.85; // = SHOOTER_P / SHOOTER_F in BioBuzzConfig (2026-11-20; was P 140 / F 13.85, P 135 / F 14.6). F is NOT voltage-scaled (same as the teleops).
    private static final double INTAKE_TRANSFER_POWER = 1.0, INTAKE_POWER = 1.0;
    /** Transfer power while feeding (right bumper). 0.9 since 2026-11-20 (was 1.0, same as the intake). */
    private static final double TRANSFER_POWER = 0.9;
    /** Right trigger: intake AND transfer both at this power (2026-11-20). */
    private static final double TRIGGER_FEED_POWER = 0.9;
    /** Within this many ticks/s of the target counts as "at speed". (The hub reads in steps of 20.) */
    private static final double TOL = 25;
    // ===============================================================

    private static final double P_STEP = 5, F_STEP = 0.05, MAX_P = 300, MAX_F = 40;
    private static final double VOLTAGE_UPDATE_SEC = 0.5, VOLTAGE_ALPHA = 0.2, MIN_VALID_VOLTAGE = 9.0;
    private static final double BURST_END_HOLD_SEC = 0.15;

    private DcMotorEx shooter;
    private DcMotor intake, transfer;
    private Servo blocker;
    private VoltageSensor battery;
    private TelemetryManager panels;

    private double p = START_P, f = START_F;
    private boolean spinning = false;
    private double voltage = 12.0; // display only
    private double lastVoltageSec;

    // Burst stats: one burst = from the blocker opening until the wheel is back at speed after it closes.
    private boolean burstActive;
    private double burstStart, burstLow, belowBandSec, lastTickSec, backAtSpeedSince;
    private int bursts;
    private double lastDip, lastBelowBand, worstDip, worstBelowBand;
    private double lastBurstP, lastBurstF;

    private boolean lUp, lDown, lLeft, lRight, lCross, lTriangle;

    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, SHOOTER);
        intake = hardwareMap.get(DcMotor.class, INTAKE);
        transfer = hardwareMap.get(DcMotor.class, TRANSFER);
        blocker = hardwareMap.get(Servo.class, BLOCKER);
        shooter.setDirection(SHOOTER_DIR);
        intake.setDirection(INTAKE_DIR);
        transfer.setDirection(TRANSFER_DIR);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        blocker.setPosition(BLOCKER_BLOCK);

        if (hardwareMap.voltageSensor.iterator().hasNext()) {
            battery = hardwareMap.voltageSensor.iterator().next();
            double v = battery.getVoltage();
            if (v >= MIN_VALID_VOLTAGE) voltage = v;
        }
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        applyCoefficients();

        panels.addData("Status", "Ready. Open http://192.168.43.1:8001, add a Graph, press cross (A) to spin.");
        panels.update(telemetry);
    }

    @Override
    public void loop() {
        double t = System.nanoTime() / 1e9;

        /* ---------- INPUT ---------- */
        boolean up = gamepad1.dpad_up, down = gamepad1.dpad_down;
        boolean left = gamepad1.dpad_left, right = gamepad1.dpad_right;
        boolean cross = gamepad1.cross, triangle = gamepad1.triangle;
        boolean circle = gamepad1.circle;

        if (cross && !lCross) {
            spinning = !spinning;
            applyCoefficients();
        }
        if (up && !lUp) { p = Range.clip(p + P_STEP, 0, MAX_P); applyCoefficients(); }
        if (down && !lDown) { p = Range.clip(p - P_STEP, 0, MAX_P); applyCoefficients(); }
        if (right && !lRight) { f = Range.clip(f + F_STEP, 0, MAX_F); applyCoefficients(); }
        if (left && !lLeft) { f = Range.clip(f - F_STEP, 0, MAX_F); applyCoefficients(); }
        if (triangle && !lTriangle) clearStats();
        lUp = up; lDown = down; lLeft = left; lRight = right; lCross = cross; lTriangle = triangle;

        /* ---------- MOTOR ---------- */
        updateVoltage(t);
        double velocity = shooter.getVelocity();
        double command = spinning ? TARGET : 0;
        shooter.setVelocity(command);
        double power = shooter.getPower();
        double ffPower = spinning ? compensatedF() * TARGET / 32767.0 : 0;

        /* ---------- FEED ---------- */
        blocker.setPosition(circle ? BLOCKER_OPEN : BLOCKER_BLOCK);
        double intakePower = 0, transferPower = 0;
        if (gamepad1.right_trigger > 0.5) {
            intakePower = TRIGGER_FEED_POWER;
            transferPower = TRIGGER_FEED_POWER;
        } else if (gamepad1.left_bumper) {
            intakePower = INTAKE_POWER;
        } else if (gamepad1.right_bumper) {
            intakePower = INTAKE_TRANSFER_POWER;
            transferPower = TRANSFER_POWER;
        }
        intake.setPower(intakePower);
        transfer.setPower(transferPower);

        trackBurst(t, circle, velocity);

        /* ---------- GRAPH (every number key is plottable in Panels) ---------- */
        panels.addData("Target", command);
        panels.addData("Actual", velocity);
        panels.addData("Error", command - velocity);
        panels.addData("Power (hub)", power);
        panels.addData("Power (F only)", ffPower);
        // A step that sits 300 below the target and goes up 40 while shooting, so shots line up with the dips.
        panels.addData("Shot gate", TARGET - 300 + (circle ? 40 : 0));

        panels.addData("Flywheel", spinning ? (Math.abs(command - velocity) <= TOL ? "AT SPEED" : "recovering") : "OFF");
        panels.addData("P / F (live, dpad)", String.format("P=%.0f  F=%.3f  (battery %.2f V, F not scaled)", p, f, voltage));
        panels.addData("Bursts", bursts);
        if (bursts > 0) {
            panels.addData("Last burst dip / time below band", String.format("%.0f ticks/s / %.2f s   (at P=%.0f F=%.3f)", lastDip, lastBelowBand, lastBurstP, lastBurstF));
            panels.addData("Worst dip / time below band", String.format("%.0f ticks/s / %.2f s", worstDip, worstBelowBand));
        }
        panels.update(telemetry);
    }

    @Override
    public void stop() {
        shooter.setPower(0);
        intake.setPower(0);
        transfer.setPower(0);
    }

    /** A burst starts when the blocker opens and ends once it is closed AND the wheel is back at speed for a moment. */
    private void trackBurst(double t, boolean blockerOpen, double velocity) {
        if (!spinning) { burstActive = false; return; }
        boolean atSpeed = Math.abs(TARGET - velocity) <= TOL;
        if (!burstActive) {
            if (blockerOpen) {
                burstActive = true;
                burstStart = t;
                burstLow = velocity;
                belowBandSec = 0;
                lastTickSec = t;
                backAtSpeedSince = -1;
            }
            return;
        }
        burstLow = Math.min(burstLow, velocity);
        if (velocity < TARGET - TOL) belowBandSec += t - lastTickSec;
        lastTickSec = t;

        if (blockerOpen || !atSpeed) {
            backAtSpeedSince = -1;
        } else if (backAtSpeedSince < 0) {
            backAtSpeedSince = t;
        } else if (t - backAtSpeedSince >= BURST_END_HOLD_SEC) {
            bursts++;
            lastDip = TARGET - burstLow;
            lastBelowBand = belowBandSec;
            lastBurstP = p;
            lastBurstF = f;
            worstDip = Math.max(worstDip, lastDip);
            worstBelowBand = Math.max(worstBelowBand, lastBelowBand);
            burstActive = false;
        }
        // Safety: a burst that never recovers (jam / wrong speed) is closed after 5 s so stats do not freeze.
        if (burstActive && t - burstStart > 5.0 && !blockerOpen) {
            bursts++;
            lastDip = TARGET - burstLow;
            lastBelowBand = belowBandSec;
            lastBurstP = p;
            lastBurstF = f;
            worstDip = Math.max(worstDip, lastDip);
            worstBelowBand = Math.max(worstBelowBand, lastBelowBand);
            burstActive = false;
        }
    }

    private void clearStats() {
        bursts = 0;
        lastDip = lastBelowBand = worstDip = worstBelowBand = 0;
        burstActive = false;
    }

    /** No voltage scaling (the teleops use F as-is). */
    private double compensatedF() {
        return f;
    }

    /** Battery voltage is shown only. */
    private void updateVoltage(double t) {
        if (battery == null || t - lastVoltageSec < VOLTAGE_UPDATE_SEC) return;
        lastVoltageSec = t;
        double v = battery.getVoltage();
        if (v >= MIN_VALID_VOLTAGE) voltage += VOLTAGE_ALPHA * (v - voltage);
    }

    /** P is 0 while OFF so stopping coasts instead of reverse-driving the wheel. */
    private void applyCoefficients() {
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(spinning ? p : 0, 0, 0, compensatedF()));
    }
}
