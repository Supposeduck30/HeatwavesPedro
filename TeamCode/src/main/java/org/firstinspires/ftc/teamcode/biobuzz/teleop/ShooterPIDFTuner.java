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

import java.util.ArrayDeque;
import java.util.ArrayList;

/**
 * One OpMode that tunes the PIDF shooter flywheel (inertia flywheel, REV hub velocity PIDF) from scratch.
 * Target = SHOOTER_FIXED_VELOCITY (1450). Nothing is saved: copy the pasted lines into BioBuzzConfig.
 *
 * HOW THE HUB LOOP WORKS: motor power = (F * target + P * error) / 32767 (I and D are left at 0: the hub's
 * speed reading moves in steps of ~20 ticks/s, so D only adds noise, and a correct F needs no I).
 *   F = feedforward: the power that holds the target speed. It does ~all the work, so it is found FIRST, alone.
 *   P = fights disturbances (a ball dragging the wheel down). Too low = slow recovery, too high = it chatters.
 * F is NOT voltage-scaled (same as PIDFShooter): the F it locks is the F for the battery you tuned on, so tune on a fresh battery.
 *
 * STAGE 1 - F (automatic). P = 0 so F is the only thing driving the motor. Press cross (A) and wait. It spins up,
 *   waits until the speed is flat, averages 1.5 s, then jumps F to the value that should hit the target
 *   (ratio first, then a secant fit through its last two points) and repeats until the average is within
 *   F_TOL ticks/s of the target. Usually 3-4 rounds, ~15 s. "F LOCKED" appears when done.
 * STAGE 2 - P (automatic). Press square (X) to switch. The target alternates between HIGH (target) and LOW
 *   (target - DIP), like a ball dragging the wheel down. For each P it measures how fast the wheel gets back to
 *   target (reach), overshoot, and the noise while holding (RMS / peak-to-peak). Press start to sweep a list of Ps
 *   hands-free (~90 s); it recommends the SMALLEST P that recovers about as fast as the best one without
 *   overshoot or extra noise, and loads it. Or step P by hand with the dpad and read the numbers.
 *   Then press back to switch to HOLD (steady target) and do a real ball test: hold right bumper (ball rests
 *   on the blocker), wait for READY, press circle. "Dip" and "Recovery" are measured for that shot.
 *
 * gamepad1 (Logitech names in brackets):
 *   cross (A)        = spin / stop (OFF until pressed; stopping coasts, it never reverse-brakes)
 *   square (X)       = switch stage F <-> P
 *   triangle (Y)     = restart (F: redo the auto-F from the current F; P: restart the stats)
 *   dpad up / down   = F + / - (stage F)   or   P + / - (stage P)
 *   dpad right/left  = bigger / smaller step
 *   start            = stage P: run / abort the automatic P sweep
 *   back             = stage P: step CYCLE <-> HOLD steady target (for the ball test)
 *   circle (B, hold) = open blocker (each press = one shot in HOLD)
 *   right bumper     = intake + transfer, left bumper = intake only
 */
@TeleOp(name = "Shooter PIDF Tuner", group = "BioBuzz")
public class ShooterPIDFTuner extends OpMode {

    private static final double TARGET = SHOOTER_FIXED_VELOCITY + PIDF_VELOCITY_OFFSET; // what the PIDF teleops / autos run at (1450)

    // ---- F stage ----
    /** F is "done" when the average speed is within this many ticks/s of the target (P and the hub do the rest). */
    private static final double F_TOL = 12;
    /** The speed counts as flat when the last two 0.5 s averages differ by less than this (ticks/s). */
    private static final double FLAT_TOL = 6;
    private static final double WINDOW_SEC = 1.0;
    private static final double SETTLE_TIMEOUT_SEC = 8.0;
    private static final double MEASURE_SEC = 1.5;
    private static final int MIN_SAMPLES = 30;
    private static final int MAX_F_ROUNDS = 12;
    private static final double MAX_F = 40;
    private static final double[] F_STEPS = {0.01, 0.05, 0.1, 0.5};

    // ---- P stage ----
    /** How far below the target the LOW phase sits: about what one ball costs the wheel. */
    private static final double DIP = 150;
    /** READY / "reached" band in ticks/s. Must be > the hub's 20 ticks/s reading step. */
    private static final double TOL = 25;
    private static final double LOW_SETTLE_SEC = 0.3;
    private static final double LOW_TIMEOUT_SEC = 3.0;
    private static final double HIGH_HOLD_SEC = 2.0;
    /** Hold stats (noise) skip this long after each up-step. */
    private static final double HOLD_SKIP_SEC = 1.0;
    private static final double[] P_STEPS = {1, 5, 10};
    private static final double MAX_P = 200;
    private static final double[] SWEEP_P = {10, 20, 35, 50, 65, 80, 100, 120, 150};
    private static final int SWEEP_CYCLES = 3;
    /** A P that overshoots the target by more than this (ticks/s) is rejected by the sweep. */
    private static final double OVERSHOOT_MAX = 40;
    private static final double SHOT_WATCH_SEC = 3.0;

    private static final double VOLTAGE_UPDATE_SEC = 0.5;
    private static final double VOLTAGE_ALPHA = 0.2;
    private static final double MIN_VALID_VOLTAGE = 9.0;

    private enum Stage { F, P }
    private enum FPhase { SETTLING, MEASURING, DONE, FAILED }
    private enum Cycle { LOW, HIGH }

    private DcMotorEx shooter;
    private DcMotor intake, transfer;
    private Servo blocker;
    private VoltageSensor battery;

    private Stage stage = Stage.F;
    private boolean spinning = false;
    private double f, p = 0;
    private double startF;
    private int fStepIndex = 1, pStepIndex = 1;

    private double voltage = SHOOTER_F_NOMINAL_VOLTAGE;
    private long lastVoltageNs;

    // F auto
    private FPhase fPhase = FPhase.SETTLING;
    private boolean fLocked = false;
    private double fPhaseStart;
    private final ArrayDeque<double[]> window = new ArrayDeque<>();
    private double mSum;
    private int mN;
    private int fRound = 0;
    private final ArrayList<double[]> fHist = new ArrayList<>(); // {f, avg}
    private final ArrayList<String> fLog = new ArrayList<>();
    private String fMessage = "";

    // P cycling
    private boolean cycling = true;
    private Cycle cycle = Cycle.LOW;
    private double cycleStart, lowInBandSince = -1;
    private boolean reached;
    private double reachSec, cycOvershoot;
    // trial totals (restart when P changes)
    private int cycles, reachedCycles;
    private double reachSum, trialOvershoot, holdSumErr, holdSumSq, holdMin, holdMax;
    private int holdN;
    private double lastReach, lastOvershoot;

    // sweep
    private boolean sweeping = false, sweepDone = false;
    private int sweepIdx;
    private final double[][] results = new double[SWEEP_P.length][6]; // p, reach, overshoot, rms, p2p, bias
    private final boolean[] resultOk = new boolean[SWEEP_P.length];
    private int resultsCount;
    private double recommendedP = -1;

    // shot test (HOLD)
    private int shots;
    private double lastShotSpeed, shotMin, shotMax;
    private boolean watching;
    private double watchStart, watchLow, lastDip, worstDip, lastRecovery, worstRecovery;

    private boolean lUp, lDown, lLeft, lRight, lCross, lSquare, lTriangle, lStart, lBack, lCircle;

    @Override
    public void init() {
        shooter  = hardwareMap.get(DcMotorEx.class, SHOOTER);
        intake   = hardwareMap.get(DcMotor.class, INTAKE);
        transfer = hardwareMap.get(DcMotor.class, TRANSFER);
        blocker  = hardwareMap.get(Servo.class, BLOCKER);
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

        // Starting F = the hub's own default for this motor type: 32767 / max ticks per second.
        double guess = SHOOTER_F;
        try {
            double maxTps = shooter.getMotorType().getAchieveableMaxTicksPerSecond();
            if (maxTps > 100) guess = 32767.0 / maxTps;
        } catch (Exception ignored) { }
        startF = f = Range.clip(Math.round(guess * 1000.0) / 1000.0, 1, MAX_F);

        applyCoefficients();
        restartF(now());
        restartTrial(now());

        telemetry.addLine("Shooter PIDF Tuner ready. Target " + (int) TARGET + " ticks/s. Press cross (A) to spin: stage 1 finds F by itself.");
        telemetry.update();
    }

    @Override
    public void loop() {
        double t = now();

        /* ---------- INPUT ---------- */
        boolean up = gamepad1.dpad_up, down = gamepad1.dpad_down;
        boolean left = gamepad1.dpad_left, right = gamepad1.dpad_right;
        boolean cross = gamepad1.cross, square = gamepad1.square, triangle = gamepad1.triangle;
        boolean start = gamepad1.start, back = gamepad1.back, circle = gamepad1.circle;

        if (cross && !lCross) {
            spinning = !spinning;
            applyCoefficients();
            if (stage == Stage.F) restartF(t);
            restartTrial(t);
            stopSweep();
        }
        if (square && !lSquare) {
            stage = stage == Stage.F ? Stage.P : Stage.F;
            stopSweep();
            applyCoefficients();
            if (stage == Stage.F) restartF(t);
            restartTrial(t);
        }
        if (triangle && !lTriangle) {
            if (stage == Stage.F) restartF(t); else restartTrial(t);
        }

        int dir = (up && !lUp ? 1 : 0) - (down && !lDown ? 1 : 0);
        int stepDir = (right && !lRight ? 1 : 0) - (left && !lLeft ? 1 : 0);
        if (stage == Stage.F) {
            fStepIndex = Range.clip(fStepIndex + stepDir, 0, F_STEPS.length - 1);
            if (dir != 0) {
                setF(f + dir * F_STEPS[fStepIndex]);
                fLocked = false;
                fLog.add("manual F -> " + trim(f));
                restartF(t);
            }
        } else {
            pStepIndex = Range.clip(pStepIndex + stepDir, 0, P_STEPS.length - 1);
            if (dir != 0 && !sweeping) {
                setP(p + dir * P_STEPS[pStepIndex]);
                restartTrial(t);
            }
            if (back && !lBack) {
                cycling = !cycling;
                stopSweep();
                restartTrial(t);
            }
            if (start && !lStart) {
                if (sweeping) stopSweep();
                else if (!spinning) fMessage = "Spin up first (cross / A).";
                else startSweep(t);
            }
        }
        lUp = up; lDown = down; lLeft = left; lRight = right; lCross = cross;
        lSquare = square; lTriangle = triangle; lStart = start; lBack = back;

        /* ---------- MOTOR ---------- */
        updateVoltage(t);
        double velocity = shooter.getVelocity();
        double command = 0;
        if (spinning) {
            if (stage == Stage.F) {
                command = TARGET;
                tuneF(t, velocity);
            } else {
                command = tuneP(t, velocity);
            }
        }
        shooter.setVelocity(command);

        // Shot test (HOLD mode, stage P)
        if (stage == Stage.P && spinning && !cycling) {
            if (circle && !lCircle) {
                shots++;
                lastShotSpeed = velocity;
                shotMin = Math.min(shotMin, velocity);
                shotMax = Math.max(shotMax, velocity);
                watching = true;
                watchStart = t;
                watchLow = velocity;
            }
            if (watching) watchShot(t, velocity);
        }
        lCircle = circle;

        /* ---------- FEED ---------- */
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
        telemetry.addData("STAGE", stage == Stage.F ? "1 / 2  F  (P = 0, automatic)" : "2 / 2  P  (F locked in)   [square = switch]");
        telemetry.addData("Flywheel", spinning ? "SPINNING" : "OFF  (cross / A to spin)");
        telemetry.addData("Target / Actual (ticks/s)", "%.0f / %.0f   error %.0f", spinning ? command : 0.0, velocity, spinning ? command - velocity : 0.0);
        telemetry.addData("Battery", battery == null ? "NO VOLTAGE SENSOR" : String.format("%.2f V (info only, F is not scaled)", voltage));
        telemetry.addData("F", "%.3f   (hub default guess %.3f)", f, startF);
        if (!fMessage.isEmpty()) telemetry.addData("NOTE", fMessage);
        telemetry.addLine();
        if (stage == Stage.F) addFTelemetry(t);
        else addPTelemetry(t);
        telemetry.addLine();
        telemetry.addData("PASTE INTO BioBuzzConfig", "SHOOTER_P = %s, SHOOTER_I = 0, SHOOTER_D = 0, SHOOTER_F = %s%s",
                trim(p), trim(f), fLocked ? "" : "   (F not locked yet)");
        telemetry.update();
    }

    @Override
    public void stop() {
        shooter.setPower(0);
        intake.setPower(0);
        transfer.setPower(0);
    }

    /* ======================= STAGE 1: F ======================= */

    private void tuneF(double t, double v) {
        if (fPhase == FPhase.DONE || fPhase == FPhase.FAILED) return;
        if (fPhase == FPhase.SETTLING) {
            window.addLast(new double[]{t, v});
            while (window.peekFirst()[0] < t - WINDOW_SEC) window.removeFirst();
            boolean flat = windowFlat(t);
            if (flat || t - fPhaseStart > SETTLE_TIMEOUT_SEC) {
                fPhase = FPhase.MEASURING;
                fPhaseStart = t;
                mSum = 0;
                mN = 0;
            }
        } else {
            mSum += v;
            mN++;
            if (t - fPhaseStart >= MEASURE_SEC && mN >= MIN_SAMPLES) finishFRound(mSum / mN);
        }
    }

    private boolean windowFlat(double t) {
        if (window.size() < 10 || t - window.peekFirst()[0] < WINDOW_SEC * 0.9) return false;
        double mid = t - WINDOW_SEC / 2, s1 = 0, s2 = 0;
        int n1 = 0, n2 = 0;
        for (double[] s : window) {
            if (s[0] < mid) { s1 += s[1]; n1++; } else { s2 += s[1]; n2++; }
        }
        return n1 > 0 && n2 > 0 && Math.abs(s1 / n1 - s2 / n2) <= FLAT_TOL;
    }

    private void finishFRound(double avg) {
        fRound++;
        double err = TARGET - avg;
        fHist.add(new double[]{f, avg});
        fLog.add(String.format("#%d  F=%.3f -> avg %.1f  (error %+.1f)", fRound, f, avg, err));

        if (avg < 0) {
            fPhase = FPhase.FAILED;
            fMessage = "Speed is NEGATIVE: flip SHOOTER_DIR in BioBuzzConfig.";
            return;
        }
        // With P = 0 nothing corrects the speed, so battery sag / friction make it wander by a few ticks/s and
        // the hub reads in 20 ticks/s steps: demanding a tighter F than that only makes it chase noise.
        boolean twoClose = fHist.size() >= 2 && Math.abs(err) <= 2 * F_TOL
                && Math.abs(TARGET - fHist.get(fHist.size() - 2)[1]) <= 2 * F_TOL;
        if (Math.abs(err) <= F_TOL || twoClose) {
            lockF(f, "F LOCKED = " + trim(f) + " (avg error " + String.format("%+.0f", err) + "). Press square (X) for stage 2.");
            return;
        }
        // Even 100% power cannot reach the target: F alone can never converge, so say so.
        double power = compensatedF() * TARGET / 32767.0;
        if (avg < TARGET - F_TOL && power >= 0.98) {
            fPhase = FPhase.FAILED;
            fMessage = "Wheel tops out at ~" + (int) avg + " ticks/s at full power: it cannot reach " + (int) TARGET
                    + ". Check the belt / mechanics / battery, or lower SHOOTER_FIXED_VELOCITY.";
            return;
        }
        if (fRound >= MAX_F_ROUNDS) {
            double[] best = fHist.get(0);
            for (double[] h : fHist) if (Math.abs(TARGET - h[1]) < Math.abs(TARGET - best[1])) best = h;
            double bestErr = TARGET - best[1];
            if (Math.abs(bestErr) <= 40) {
                f = best[0];
                applyCoefficients();
                lockF(best[0], "F locked at the best round: " + trim(best[0]) + " (avg error " + String.format("%+.0f", bestErr)
                        + "; speed was wandering). P will fix the rest. Press square (X) for stage 2.");
            } else {
                fPhase = FPhase.FAILED;
                fMessage = "F never got within 40 ticks/s in " + MAX_F_ROUNDS + " rounds (best error " + String.format("%+.0f", bestErr)
                        + "). Look at the rounds above: speed not responding to F = check motor/encoder/belt.";
            }
            return;
        }

        // Ratio first (speed ~ proportional to F), then a secant through the last two points (handles friction).
        double nf = f * Range.clip(TARGET / Math.max(avg, 1), 0.5, 2.0);
        if (fHist.size() >= 2) {
            double[] a = fHist.get(fHist.size() - 2), b = fHist.get(fHist.size() - 1);
            double df = b[0] - a[0];
            if (Math.abs(df) >= 0.3) { // a short baseline just amplifies noise
                double slope = (b[1] - a[1]) / df;
                if (slope > 0.3 * b[1] / b[0]) nf = Range.clip(b[0] + err / slope, 0.5 * b[0], 2.0 * b[0]);
            }
        }
        if (Math.abs(err) < 40) nf = f + 0.7 * (nf - f); // small error: damp so noise is not chased
        setF(nf);
        restartPhase();
    }

    private void lockF(double value, String message) {
        fPhase = FPhase.DONE;
        fLocked = true;
        fMessage = message;
    }

    /** Start the automatic F search again from the current F. */
    private void restartF(double t) {
        fPhase = FPhase.SETTLING;
        fRound = 0;
        fHist.clear();
        fLog.clear();
        fMessage = "";
        fLocked = false;
        restartPhase();
    }

    private void restartPhase() {
        fPhase = FPhase.SETTLING;
        fPhaseStart = now();
        window.clear();
        mSum = 0;
        mN = 0;
    }

    private void addFTelemetry(double t) {
        String state;
        switch (fPhase) {
            case SETTLING: state = spinning ? "waiting for the speed to go flat..." : "OFF"; break;
            case MEASURING: state = String.format("measuring %.1f / %.1f s", Math.min(MEASURE_SEC, t - fPhaseStart), MEASURE_SEC); break;
            case DONE: state = "DONE"; break;
            default: state = "FAILED"; break;
        }
        telemetry.addData("F auto", state);
        telemetry.addData("Rounds", "%d (max %d)", fRound, MAX_F_ROUNDS);
        for (int i = Math.max(0, fLog.size() - 5); i < fLog.size(); i++) telemetry.addLine("  " + fLog.get(i));
        telemetry.addData("Manual", "dpad up/down F +/- %.2f (right/left = step)   triangle = redo", F_STEPS[fStepIndex]);
    }

    /* ======================= STAGE 2: P ======================= */

    /** Returns the velocity command. */
    private double tuneP(double t, double v) {
        double high = TARGET, low = TARGET - DIP;
        if (!cycling) return high;

        if (cycle == Cycle.LOW) {
            boolean inBand = Math.abs(v - low) <= TOL;
            if (inBand) {
                if (lowInBandSince < 0) lowInBandSince = t;
            } else {
                lowInBandSince = -1;
            }
            boolean settled = lowInBandSince >= 0 && t - lowInBandSince >= LOW_SETTLE_SEC;
            if (settled || t - cycleStart > LOW_TIMEOUT_SEC) {
                cycle = Cycle.HIGH;
                cycleStart = t;
                reached = false;
                cycOvershoot = 0;
                return high;
            }
            return low;
        }

        double age = t - cycleStart;
        if (!reached && Math.abs(v - high) <= TOL) {
            reached = true;
            reachSec = age;
        }
        cycOvershoot = Math.max(cycOvershoot, v - high);
        if (age >= HOLD_SKIP_SEC) {
            double e = high - v;
            holdSumErr += e;
            holdSumSq += e * e;
            holdN++;
            holdMin = Math.min(holdMin, v);
            holdMax = Math.max(holdMax, v);
        }
        if (age >= HIGH_HOLD_SEC) endCycle(t);
        return high;
    }

    private void endCycle(double t) {
        cycles++;
        if (reached) {
            reachedCycles++;
            reachSum += reachSec;
            lastReach = reachSec;
        } else {
            lastReach = -1;
        }
        lastOvershoot = cycOvershoot;
        trialOvershoot = Math.max(trialOvershoot, cycOvershoot);
        cycle = Cycle.LOW;
        cycleStart = t;
        lowInBandSince = -1;

        if (sweeping && cycles >= SWEEP_CYCLES) advanceSweep(t);
    }

    private double trialReach() { return reachedCycles == cycles && cycles > 0 ? reachSum / cycles : -1; }
    private double trialRms() { return holdN > 0 ? Math.sqrt(holdSumSq / holdN) : -1; }
    private double trialBias() { return holdN > 0 ? holdSumErr / holdN : 0; }
    private double trialP2P() { return holdN > 0 ? holdMax - holdMin : -1; }

    private void restartTrial(double t) {
        cycle = Cycle.LOW;
        cycleStart = t;
        lowInBandSince = -1;
        cycles = 0;
        reachedCycles = 0;
        reachSum = 0;
        trialOvershoot = 0;
        holdSumErr = 0;
        holdSumSq = 0;
        holdN = 0;
        holdMin = Double.MAX_VALUE;
        holdMax = -Double.MAX_VALUE;
        lastReach = -1;
        lastOvershoot = 0;
        shots = 0;
        shotMin = Double.MAX_VALUE;
        shotMax = -Double.MAX_VALUE;
        watching = false;
        lastDip = worstDip = 0;
        lastRecovery = worstRecovery = -1;
    }

    /* ---------- sweep ---------- */

    private void startSweep(double t) {
        sweeping = true;
        sweepDone = false;
        resultsCount = 0;
        recommendedP = -1;
        sweepIdx = 0;
        cycling = true;
        setP(SWEEP_P[0]);
        restartTrial(t);
    }

    private void stopSweep() {
        sweeping = false;
    }

    private void advanceSweep(double t) {
        double[] r = results[resultsCount];
        r[0] = p; r[1] = trialReach(); r[2] = trialOvershoot; r[3] = trialRms(); r[4] = trialP2P(); r[5] = trialBias();
        resultsCount++;
        sweepIdx++;
        if (sweepIdx < SWEEP_P.length) {
            setP(SWEEP_P[sweepIdx]);
            restartTrial(t);
            return;
        }
        // Done: reject P that never reached / overshoots / is noisier than the quiet ones, then take the
        // smallest P whose recovery is within 10% of the best.
        sweeping = false;
        sweepDone = true;
        double rmsFloor = Double.MAX_VALUE;
        for (int i = 0; i < resultsCount; i++) if (results[i][3] >= 0) rmsFloor = Math.min(rmsFloor, results[i][3]);
        double bestReach = Double.MAX_VALUE;
        for (int i = 0; i < resultsCount; i++) {
            double[] q = results[i];
            resultOk[i] = q[1] >= 0 && q[2] <= OVERSHOOT_MAX && q[3] <= rmsFloor * 1.35 + 3;
            if (resultOk[i]) bestReach = Math.min(bestReach, q[1]);
        }
        recommendedP = -1;
        for (int i = 0; i < resultsCount; i++) {
            if (resultOk[i] && results[i][1] <= bestReach * 1.10 + 0.03) { recommendedP = results[i][0]; break; }
        }
        if (recommendedP >= 0) setP(recommendedP);
        restartTrial(t);
    }

    private void addPTelemetry(double t) {
        telemetry.addData("P", "%.0f   (dpad up/down, step %.0f; right/left = step)", p, P_STEPS[pStepIndex]);
        telemetry.addData("Mode", cycling
                ? String.format("CYCLE  HIGH %.0f <-> LOW %.0f   (back = HOLD for ball test)", TARGET, TARGET - DIP)
                : "HOLD steady target   (back = CYCLE)");
        if (!fLocked) telemetry.addData("WARNING", "F is not locked. Finish stage 1 first or P results are meaningless.");

        if (cycling) {
            telemetry.addData("Sweep", sweeping
                    ? String.format("RUNNING  P=%.0f  (%d/%d)  start = abort", p, sweepIdx + 1, SWEEP_P.length)
                    : "start = run automatic P sweep");
            telemetry.addData("Phase", "%s   cycles %d%s", cycle, cycles, sweeping ? "/" + SWEEP_CYCLES : "");
            telemetry.addData("Last step: reach / overshoot", "%s / %.0f", secs(lastReach), lastOvershoot);
            telemetry.addData("Trial: avg reach", reachedCycles < cycles ? "missed " + (cycles - reachedCycles) + " of " + cycles : secs(trialReach()));
            telemetry.addData("Trial: worst overshoot", "%.0f", trialOvershoot);
            telemetry.addData("Trial: hold RMS / p2p / bias", "%s / %s / %s", num(trialRms()), num(trialP2P()), holdN > 0 ? String.format("%+.1f", trialBias()) : "-");
        } else {
            telemetry.addData("SHOT TEST", "hold right bumper, wait for READY, press circle");
            telemetry.addData("Shots", "%d", shots);
            if (shots > 0) {
                telemetry.addData("Speed at shot last / min / max", "%.0f / %.0f / %.0f   (spread %.0f)", lastShotSpeed, shotMin, shotMax, shotMax - shotMin);
                telemetry.addData("Dip last / worst (below target)", "%.0f / %.0f", lastDip, worstDip);
                telemetry.addData("Recovery last / worst", "%s / %s%s", secs(lastRecovery), secs(worstRecovery), watching ? "  (watching...)" : "");
            }
        }

        if (resultsCount > 0) {
            telemetry.addLine("--- SWEEP RESULTS: P | reach | overshoot | RMS | p2p ---");
            for (int i = 0; i < resultsCount; i++) {
                double[] q = results[i];
                String tag = sweepDone ? (resultOk[i] ? (q[0] == recommendedP ? "  <== RECOMMENDED" : "  ok") : "  rejected") : "";
                telemetry.addLine(String.format("  P=%3.0f | %s | %3.0f | %s | %s%s", q[0], secs(q[1]), q[2], num(q[3]), num(q[4]), tag));
            }
            if (sweepDone) {
                telemetry.addData("Sweep result", recommendedP >= 0
                        ? "P = " + trim(recommendedP) + " loaded. Confirm in HOLD with real balls (back)."
                        : "no P passed: check F is locked and the wheel is not oscillating");
            }
        }
    }

    private void watchShot(double t, double v) {
        watchLow = Math.min(watchLow, v);
        double age = t - watchStart;
        boolean back = v >= TARGET - TOL;
        if (age > 0.15 && back && watchLow < TARGET - TOL) {
            lastDip = TARGET - watchLow;
            worstDip = Math.max(worstDip, lastDip);
            lastRecovery = age;
            worstRecovery = Math.max(worstRecovery, lastRecovery);
            watching = false;
        } else if (age > SHOT_WATCH_SEC) {
            lastDip = TARGET - watchLow;
            worstDip = Math.max(worstDip, lastDip);
            lastRecovery = -1;
            watching = false;
        }
    }

    /* ======================= shared ======================= */

    private void setF(double newF) {
        f = Range.clip(Math.round(newF * 1000.0) / 1000.0, 0, MAX_F);
        applyCoefficients();
    }

    private void setP(double newP) {
        p = Range.clip(newP, 0, MAX_P);
        applyCoefficients();
    }

    /** No voltage scaling (the teleops / autos use F as-is), so the applied F is just F. */
    private double compensatedF() {
        return f;
    }

    /** Battery voltage is only read for the telemetry display now. */
    private void updateVoltage(double t) {
        if (battery == null || t - lastVoltageNs / 1e9 < VOLTAGE_UPDATE_SEC) return;
        lastVoltageNs = System.nanoTime();
        double v = battery.getVoltage();
        if (v >= MIN_VALID_VOLTAGE) voltage += VOLTAGE_ALPHA * (v - voltage);
    }

    /** P is 0 in stage F and while OFF (so stopping coasts instead of reverse-driving the wheel). */
    private void applyCoefficients() {
        lastVoltageNs = System.nanoTime();
        double effP = (spinning && stage == Stage.P) ? p : 0;
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(effP, 0, 0, compensatedF()));
    }

    private static double now() {
        return System.nanoTime() / 1e9;
    }

    private static String secs(double s) {
        return s < 0 ? "-" : String.format("%.2f s", s);
    }

    private static String num(double v) {
        return v < 0 ? "-" : String.format("%.0f", v);
    }

    /** 14.9000 -> "14.9". */
    private static String trim(double v) {
        return String.format("%.4f", v).replaceAll("0+$", "").replaceAll("\\.$", "");
    }
}
