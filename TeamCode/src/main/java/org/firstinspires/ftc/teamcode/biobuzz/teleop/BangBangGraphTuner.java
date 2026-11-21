package org.firstinspires.ftc.teamcode.biobuzz.teleop;

import static org.firstinspires.ftc.teamcode.biobuzz.teleop.BioBuzzConfig.*;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

/**
 * Bang-bang flywheel tuner with Panels graphs.
 *
 * WHAT BANG-BANG IS: the motor is either fully ON (below the target) or OFF (above it). That is the fastest possible
 * recovery after a ball steals speed, which is why it is popular for heavy flywheels. Plain bang-bang (OFF = 0
 * power) makes the wheel saw-tooth up and down around the target. The standard fix (WPILib's recommendation) is
 * bang-bang + FEEDFORWARD: "OFF" is not 0 but a HOLD power that by itself keeps the wheel just under the target, so
 * bang-bang only has to make up for shots. Hold is set slightly LOW on purpose (about 0.95 x the exact value),
 * because bang-bang can only push the speed up, never pull it down. The wheel is in COAST so zero power never brakes.
 *
 * The control (same as BangBangShooter): ON when speed < target - hysteresis, OFF when speed > target + hysteresis,
 * otherwise keep the last state. Power = ON ? ON power : HOLD power. So BANG_BANG_OFF_POWER is the hold power.
 *
 * gamepad1 (PS5 names):
 *   cross              = spin up / stop
 *   square             = AUTO-FIND hold power (open loop, ~10-20 s, keep balls out; square again cancels)
 *   dpad up / down     = HOLD (OFF) power + / - step
 *   dpad right / left  = ON power + / - step
 *   R3 / L3 (stick click) = hysteresis + / - 5 ticks/s
 *   options            = step 0.01 <-> 0.05
 *   circle (hold)      = open the blocker (balls go into the wheel)
 *   R1 = intake + transfer (feed), L1 = intake only
 *   triangle           = clear the stats
 *
 * HOW TO TUNE:
 *   1. Spin up (cross), then press square and wait for "HOLD FOUND". It sets HOLD to 0.95 x the power that holds the
 *      target by itself.
 *   2. With no balls, look at "Idle ripple" (max - min). Under ~40 ticks/s is good. Big ripple = raise HOLD a bit
 *      (closer to the exact value). Speed sitting ABOVE target + hysteresis = HOLD too high, lower it.
 *   3. Shoot bursts (R1 + circle). Watch "Dip" and "Time below band". Keep ON = 1.0 unless it overshoots a lot after
 *      each recovery ("Overshoot"); then lower ON or raise hysteresis.
 *   4. Copy the "PASTE INTO BioBuzzConfig" lines.
 *
 * PANELS: open http://192.168.43.1:8001, add a Graph and tick "Target", "Actual", "Band low", "Band high", "Shot gate";
 * a second graph with "Power" and "Hold power".
 */
@TeleOp(name = "Bang-Bang Graph Tuner", group = "BioBuzz Tuning")
public class BangBangGraphTuner extends OpMode {

    /** Within this many ticks/s of the target counts as "at speed" for the burst stats. */
    private static final double TOL = 25;
    /** HOLD is set to this x the exact hold power found by the auto-find (bang-bang can only push up). */
    private static final double HOLD_SAFETY = 0.95;

    // Auto-find
    private static final double FIND_SETTLE_SEC = 1.5, FIND_AVERAGE_SEC = 1.0, FIND_DONE_ERROR = 10;
    private static final int FIND_MAX_ROUNDS = 8;

    private static final double IDLE_SETTLE_SEC = 1.5, BURST_END_HOLD_SEC = 0.15;

    private DcMotorEx shooter;
    private DcMotor intake, transfer;
    private Servo blocker;
    private VoltageSensor battery;
    private List<LynxModule> hubs;
    private TelemetryManager panels;

    private final double target = SHOOTER_FIXED_VELOCITY;
    private double onPower = BANG_BANG_ON_POWER;
    private double holdPower = BANG_BANG_OFF_POWER;
    private double hysteresis = BANG_BANG_HYSTERESIS;
    private double step = 0.01;

    private boolean spinning = false;
    private boolean motorOn = false;
    private int switches = 0;
    private double switchCountStart;

    // Auto-find state
    private boolean finding = false;
    private int findRound;
    private double findPower, findLegStart, findSum, bestFindPower, bestFindError;
    private int findSamples;
    private String findStatus = "not run (press square while spinning)";
    private double exactHold = Double.NaN;

    // Idle ripple (no feeding, not recovering)
    private double idleSince = -1, idleMin, idleMax;

    // Bursts
    private boolean burstActive;
    private double burstStart, burstLow, belowBandSec, lastTickSec, backAtSpeedSince, burstPeak;
    private int bursts;
    private double lastDip, lastBelowBand, lastOvershoot, worstDip, worstBelowBand;
    private double overshootWatchUntil = -1, overshootPeak;

    private double loopMs, lastLoopSec;

    private boolean lCross, lSquare, lUp, lDown, lLeft, lRight, lR3, lL3, lOptions, lTriangle;

    @Override
    public void init() {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        shooter = hardwareMap.get(DcMotorEx.class, SHOOTER);
        intake = hardwareMap.get(DcMotor.class, INTAKE);
        transfer = hardwareMap.get(DcMotor.class, TRANSFER);
        blocker = hardwareMap.get(Servo.class, BLOCKER);
        shooter.setDirection(SHOOTER_DIR);
        intake.setDirection(INTAKE_DIR);
        transfer.setDirection(TRANSFER_DIR);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Raw power: we are the controller. getVelocity() still reads the encoder.
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // COAST: with BRAKE, every OFF moment would actively slow the wheel and fight the controller.
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        blocker.setPosition(BLOCKER_BLOCK);

        if (hardwareMap.voltageSensor.iterator().hasNext()) battery = hardwareMap.voltageSensor.iterator().next();
        panels = PanelsTelemetry.INSTANCE.getTelemetry();

        // First guess for hold if the config has none: the hub's F term maps F * velocity / 32767 to power.
        if (holdPower <= 0) holdPower = round2(Range.clip(HOLD_SAFETY * SHOOTER_F * target / 32767.0, 0, 1));

        panels.addData("Status", "Ready. Press cross to spin, then square to auto-find the hold power.");
        panels.update(telemetry);
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) hub.clearBulkCache();
        double t = System.nanoTime() / 1e9;
        if (lastLoopSec > 0) loopMs = 0.9 * loopMs + 0.1 * (t - lastLoopSec) * 1000;
        lastLoopSec = t;

        /* ---------- INPUT ---------- */
        boolean cross = gamepad1.cross, square = gamepad1.square, triangle = gamepad1.triangle;
        boolean up = gamepad1.dpad_up, down = gamepad1.dpad_down, left = gamepad1.dpad_left, right = gamepad1.dpad_right;
        boolean r3 = gamepad1.right_stick_button, l3 = gamepad1.left_stick_button, options = gamepad1.options;

        if (cross && !lCross) {
            spinning = !spinning;
            finding = false;
            motorOn = false;
            resetIdle();
            switches = 0;
            switchCountStart = t;
        }
        if (square && !lSquare) {
            if (finding) {
                finding = false;
                findStatus = "cancelled";
            } else if (spinning) {
                startFind(t);
            } else {
                findStatus = "spin up first (cross), then press square";
            }
        }
        if (up && !lUp)       { holdPower = round2(Range.clip(holdPower + step, 0, 1)); changed(t); }
        if (down && !lDown)   { holdPower = round2(Range.clip(holdPower - step, 0, 1)); changed(t); }
        if (right && !lRight) { onPower = round2(Range.clip(onPower + step, 0, 1));     changed(t); }
        if (left && !lLeft)   { onPower = round2(Range.clip(onPower - step, 0, 1));     changed(t); }
        if (r3 && !lR3)       { hysteresis += 5;                                        changed(t); }
        if (l3 && !lL3)       { hysteresis = Math.max(0, hysteresis - 5);               changed(t); }
        if (options && !lOptions) step = step == 0.01 ? 0.05 : 0.01;
        if (triangle && !lTriangle) clearStats(t);
        lCross = cross; lSquare = square; lUp = up; lDown = down; lLeft = left; lRight = right;
        lR3 = r3; lL3 = l3; lOptions = options; lTriangle = triangle;

        /* ---------- CONTROL ---------- */
        double velocity = shooter.getVelocity();
        double power;
        if (!spinning) {
            power = 0;
        } else if (finding) {
            power = updateFind(t, velocity);
        } else {
            boolean wasOn = motorOn;
            if (velocity < target - hysteresis) motorOn = true;
            else if (velocity > target + hysteresis) motorOn = false;
            if (motorOn != wasOn) switches++;
            power = motorOn ? onPower : holdPower;
        }
        shooter.setPower(power);

        /* ---------- FEED (blocked while auto-finding) ---------- */
        boolean shooting = gamepad1.circle && !finding;
        blocker.setPosition(shooting ? BLOCKER_OPEN : BLOCKER_BLOCK);
        double intakePower = 0, transferPower = 0;
        if (!finding && gamepad1.right_bumper) {
            intakePower = BANG_BANG_INTAKE_TRANSFER_POWER;
            transferPower = BANG_BANG_INTAKE_TRANSFER_POWER;
        } else if (!finding && gamepad1.left_bumper) {
            intakePower = INTAKE_IN_POWER;
        }
        intake.setPower(intakePower);
        transfer.setPower(transferPower);

        /* ---------- STATS ---------- */
        boolean controlling = spinning && !finding;
        trackBurst(t, controlling, shooting, velocity);
        trackIdle(t, controlling && !shooting && !burstActive && intakePower == 0, velocity);

        /* ---------- GRAPH + TELEMETRY ---------- */
        panels.addData("Target", spinning ? target : 0);
        panels.addData("Actual", velocity);
        panels.addData("Error", spinning ? target - velocity : 0);
        panels.addData("Band low", spinning ? target - hysteresis : 0);
        panels.addData("Band high", spinning ? target + hysteresis : 0);
        panels.addData("Power", power);
        panels.addData("Hold power", holdPower);
        panels.addData("Shot gate", target - 300 + (shooting ? 40 : 0));

        panels.addData("Flywheel", !spinning ? "OFF (cross to spin)" : finding ? "AUTO-FINDING HOLD" : (motorOn ? "ON" : "hold"));
        panels.addData("ON / HOLD / hysteresis", String.format("%.2f / %.2f / %.0f   (step %.2f)", onPower, holdPower, hysteresis, step));
        panels.addData("Hold auto-find", findStatus);
        if (battery != null) panels.addData("Battery", String.format("%.2f V  (open-loop power depends on it)", battery.getVoltage()));
        panels.addData("Loop", String.format("%.1f ms", loopMs));
        double switchSec = t - switchCountStart;
        panels.addData("Switches / s", String.format("%.1f", controlling && switchSec > 0 ? switches / switchSec : 0));
        if (idleSince > 0 && t - idleSince >= IDLE_SETTLE_SEC && idleMax >= idleMin) {
            panels.addData("Idle ripple (min / max / p-p)", String.format("%.0f / %.0f / %.0f", idleMin, idleMax, idleMax - idleMin));
        } else {
            panels.addData("Idle ripple (min / max / p-p)", controlling ? "settling..." : "-");
        }
        panels.addData("Bursts", bursts);
        if (bursts > 0) {
            panels.addData("Last dip / time below band / overshoot", String.format("%.0f / %.2f s / %.0f", lastDip, lastBelowBand, lastOvershoot));
            panels.addData("Worst dip / time below band", String.format("%.0f / %.2f s", worstDip, worstBelowBand));
        }
        panels.addData("PASTE INTO BioBuzzConfig", String.format("BANG_BANG_ON_POWER = %.2f; BANG_BANG_OFF_POWER = %.2f; BANG_BANG_HYSTERESIS = %.0f;",
                onPower, holdPower, hysteresis));
        panels.update(telemetry);
    }

    @Override
    public void stop() {
        shooter.setPower(0);
        intake.setPower(0);
        transfer.setPower(0);
    }

    // ================= AUTO-FIND HOLD POWER =================
    // Open loop: run a fixed power, let the wheel settle, average its speed, then rescale the power by target / speed
    // (a flywheel's speed is close to proportional to power). Repeat until the average is within FIND_DONE_ERROR.

    private void startFind(double t) {
        finding = true;
        findRound = 1;
        findPower = Range.clip(!Double.isNaN(exactHold) ? exactHold : SHOOTER_F * target / 32767.0, 0.05, 1.0);
        bestFindPower = findPower;
        bestFindError = Double.MAX_VALUE;
        beginFindLeg(t);
    }

    private void beginFindLeg(double t) {
        findLegStart = t;
        findSum = 0;
        findSamples = 0;
        findStatus = String.format("round %d: power %.3f, settling...", findRound, findPower);
    }

    private double updateFind(double t, double velocity) {
        double age = t - findLegStart;
        if (age >= FIND_SETTLE_SEC) {
            findSum += velocity;
            findSamples++;
            findStatus = String.format("round %d: power %.3f, measuring...", findRound, findPower);
        }
        if (age >= FIND_SETTLE_SEC + FIND_AVERAGE_SEC && findSamples > 0) {
            double avg = findSum / findSamples;
            double err = target - avg;
            if (Math.abs(err) < bestFindError) {
                bestFindError = Math.abs(err);
                bestFindPower = findPower;
            }
            if (avg < 100) {
                return finishFind(Double.NaN, "FAILED: the wheel barely moved (" + Math.round(avg) + " ticks/s). Check the motor / direction.");
            }
            if (Math.abs(err) <= FIND_DONE_ERROR) {
                return finishFind(findPower, String.format("HOLD FOUND: %.3f holds %.0f (error %+.0f)", findPower, avg, err));
            }
            if (findRound >= FIND_MAX_ROUNDS) {
                return finishFind(bestFindPower, String.format("best after %d rounds: %.3f (error %.0f)", findRound, bestFindPower, bestFindError));
            }
            if (findPower >= 1.0 && avg < target) {
                return finishFind(Double.NaN, String.format("FAILED: full power only reaches %.0f, target %.0f is too high", avg, target));
            }
            findPower = Range.clip(findPower * target / avg, 0.05, 1.0);
            findRound++;
            beginFindLeg(t);
        }
        return findPower;
    }

    private double finishFind(double exact, String status) {
        finding = false;
        findStatus = status;
        if (!Double.isNaN(exact)) {
            exactHold = exact;
            holdPower = round2(HOLD_SAFETY * exact);
            findStatus += String.format("  -> HOLD set to %.2f (%.2f x)", holdPower, HOLD_SAFETY);
        }
        motorOn = false;
        changed(System.nanoTime() / 1e9);
        return holdPower;
    }

    // ================= STATS =================

    private void changed(double t) {
        resetIdle();
        switches = 0;
        switchCountStart = t;
    }

    private void resetIdle() {
        idleSince = -1;
    }

    private void trackIdle(double t, boolean idle, double velocity) {
        if (!idle) {
            idleSince = -1;
            return;
        }
        if (idleSince < 0) {
            idleSince = t;
            idleMin = Double.MAX_VALUE;
            idleMax = -Double.MAX_VALUE;
        }
        if (t - idleSince >= IDLE_SETTLE_SEC) {
            idleMin = Math.min(idleMin, velocity);
            idleMax = Math.max(idleMax, velocity);
        }
    }

    /** A burst runs from the blocker opening until it's closed and the wheel is back at speed for a moment. */
    private void trackBurst(double t, boolean controlling, boolean blockerOpen, double velocity) {
        // Overshoot: highest speed in the second after a burst ends.
        if (overshootWatchUntil > 0) {
            overshootPeak = Math.max(overshootPeak, velocity - target);
            if (t >= overshootWatchUntil) {
                lastOvershoot = Math.max(0, overshootPeak);
                overshootWatchUntil = -1;
            }
        }
        if (!controlling) {
            burstActive = false;
            return;
        }
        boolean atSpeed = Math.abs(target - velocity) <= TOL;
        if (!burstActive) {
            if (blockerOpen) {
                burstActive = true;
                burstStart = t;
                burstLow = velocity;
                burstPeak = velocity;
                belowBandSec = 0;
                lastTickSec = t;
                backAtSpeedSince = -1;
            }
            return;
        }
        burstLow = Math.min(burstLow, velocity);
        if (velocity < target - TOL) belowBandSec += t - lastTickSec;
        lastTickSec = t;

        boolean ended = false;
        if (blockerOpen || !atSpeed) {
            backAtSpeedSince = -1;
        } else if (backAtSpeedSince < 0) {
            backAtSpeedSince = t;
        } else if (t - backAtSpeedSince >= BURST_END_HOLD_SEC) {
            ended = true;
        }
        if (!ended && t - burstStart > 5.0 && !blockerOpen) ended = true; // never recovered: close it anyway
        if (ended) {
            bursts++;
            lastDip = target - burstLow;
            lastBelowBand = belowBandSec;
            worstDip = Math.max(worstDip, lastDip);
            worstBelowBand = Math.max(worstBelowBand, lastBelowBand);
            burstActive = false;
            overshootPeak = velocity - target;
            overshootWatchUntil = t + 1.0;
        }
    }

    private void clearStats(double t) {
        bursts = 0;
        lastDip = lastBelowBand = lastOvershoot = worstDip = worstBelowBand = 0;
        burstActive = false;
        overshootWatchUntil = -1;
        changed(t);
    }

    private static double round2(double v) {
        return Math.round(v * 100.0) / 100.0;
    }
}
