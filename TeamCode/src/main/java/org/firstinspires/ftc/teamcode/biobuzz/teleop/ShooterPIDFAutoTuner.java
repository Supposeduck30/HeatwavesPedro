package org.firstinspires.ftc.teamcode.biobuzz.teleop;

import static org.firstinspires.ftc.teamcode.biobuzz.teleop.BioBuzzConfig.*;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Shooter PIDF Auto Tuner (v2). Fully automatic F and P for the flywheel, then a live ball test.
 *
 * THE METHOD (standard flywheel practice, e.g. WPILib's flywheel tuning guide):
 *   1. Feedforward first. F should do almost all the work. Instead of guessing, this CHARACTERIZES the wheel: it runs
 *      5 fixed powers open loop, measures the steady speed at each, and fits  power = kS + kV * speed  (kS = friction,
 *      kV = power per tick/s). The hub's F has no kS term, so the F that is exact AT THE TARGET is
 *      F = 32767 * (kS + kV * target) / target. Then it checks that F in the hub's own loop (P = 0) and corrects it
 *      until the wheel holds the target within 10 ticks/s by F alone.
 *   2. Then P, judged on what actually matters: recovering after a ball. A ball is a LOAD disturbance (it drags the
 *      wheel down while the target stays the same). So for each P it lets the wheel coast with 0 power until it has
 *      lost DIP ticks/s (like a shot), re-engages at the same target, and measures the recovery time, overshoot and
 *      ringing. It also measures how noisy the speed is while holding (too much P amplifies the hub's ~20 ticks/s
 *      reading steps). Each P is tested 3 times (median). P values that overshoot, ring or get noisy are rejected.
 *      The winner is the SMALLEST P that recovers within 10% of the fastest one (more margin, less noise), then it
 *      tests the halfway points around it to refine.
 *   3. Live check with real balls at the chosen P (dip and time below band per burst). P is adjustable by dpad.
 *   I and D stay 0: a correct F leaves no steady error for I to fix, and D only adds noise on a constant target.
 *
 * Nothing is saved: copy the "PASTE INTO BioBuzzConfig" line. Use a CHARGED battery: F is not voltage-scaled, so it is
 * right for the voltage you tune at. Keep balls out until step 3.
 *
 * gamepad1 (PS5 names):
 *   cross        = run the full auto-tune (F then P)  /  abort while it runs  /  stop the wheel in step 3
 *   square       = run only the P search, using the current F
 *   --- step 3 (live check) ---
 *   dpad up/down = P + / - 5        dpad right/left = F + / - 0.05
 *   circle (hold) = open the blocker   R1 = intake + transfer   L1 = intake only   triangle = clear burst stats
 *
 * PANELS (http://192.168.43.1:8001): graph "Target", "Actual" (and "Power").
 */
@TeleOp(name = "Shooter PIDF Auto Tuner", group = "BioBuzz Tuning")
public class ShooterPIDFAutoTuner extends LinearOpMode {

    private static final double TARGET = SHOOTER_FIXED_VELOCITY + PIDF_VELOCITY_OFFSET;

    // ---- feedforward ----
    private static final double[] FF_POWERS = {0.30, 0.42, 0.54, 0.66, 0.78};
    private static final double FLAT_TOL = 6;          // two 0.5 s averages within this = steady
    private static final double FLAT_TIMEOUT = 5.0;
    private static final double FF_AVERAGE_SEC = 1.0;
    private static final double F_TOL = 10;            // F alone must hold the target within this
    private static final int F_VERIFY_ROUNDS = 5;
    private static final double MAX_F = 40;

    // ---- P search ----
    private static final double[] P_COARSE = {15, 30, 50, 75, 100, 135, 180, 240};
    private static final double DIP = 150;             // ticks/s lost in the simulated shot
    private static final double DIP_TIMEOUT = 1.2;
    private static final double TOL = 25;              // "at speed" band (> the hub's 20 ticks/s step)
    private static final double IN_BAND_HOLD = 0.25;   // must stay in band this long to count as recovered
    private static final double RECOVERY_WINDOW = 2.0; // watch this long after each re-engage
    private static final double SETTLE_TIMEOUT = 4.0;
    private static final double NOISE_SEC = 1.0;
    private static final int TRIALS = 3;
    private static final double OVERSHOOT_MAX = 40;
    private static final int MAX_RING_CROSSINGS = 2;   // crossings of the target by more than TOL after recovery starts
    private static final double PICK_MARGIN = 1.10;    // smallest P within 10% of the fastest recovery

    private static final double MIN_VALID_VOLTAGE = 9.0;

    private DcMotorEx shooter;
    private DcMotor intake, transfer;
    private Servo blocker;
    private VoltageSensor battery;
    private List<LynxModule> hubs;
    private TelemetryManager panels;

    // live state
    private double velocity, voltage = 12.0, lastVoltageSec, commandTarget, commandPower;
    private boolean aborted;
    private boolean lCross, lSquare, lUp, lDown, lLeft, lRight, lTriangle;
    private String stage = "IDLE: press cross to auto-tune (balls out, charged battery)";
    private String detail = "";

    // results
    private final List<double[]> ffPoints = new ArrayList<>(); // {power, speed, volts}
    private double kS = Double.NaN, kV = Double.NaN, r2 = Double.NaN, fFit = Double.NaN;
    private double f = SHOOTER_F, p = SHOOTER_P;
    private String fResult = "not run (using SHOOTER_F " + SHOOTER_F + ")";
    private final List<PResult> pResults = new ArrayList<>();
    private String pResult = "not run (using SHOOTER_P " + SHOOTER_P + ")";
    private double tuneVoltage = Double.NaN;

    private static class PResult {
        double p, holdRms, holdPp, recovery, overshoot;
        int ring;
        boolean ok;
        String why = "";
    }

    @Override
    public void runOpMode() {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        shooter = hardwareMap.get(DcMotorEx.class, SHOOTER);
        intake = hardwareMap.get(DcMotor.class, INTAKE);
        transfer = hardwareMap.get(DcMotor.class, TRANSFER);
        blocker = hardwareMap.get(Servo.class, BLOCKER);
        shooter.setDirection(SHOOTER_DIR);
        intake.setDirection(INTAKE_DIR);
        transfer.setDirection(TRANSFER_DIR);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // coast: 0 power never brakes the wheel
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blocker.setPosition(BLOCKER_BLOCK);
        if (hardwareMap.voltageSensor.iterator().hasNext()) battery = hardwareMap.voltageSensor.iterator().next();
        panels = PanelsTelemetry.INSTANCE.getTelemetry();

        while (opModeInInit()) {
            tick();
        }

        boolean prevCross = gamepad1.cross, prevSquare = gamepad1.square;
        while (opModeIsActive()) {
            tick();
            boolean cross = gamepad1.cross, square = gamepad1.square;
            if (pressed(cross, prevCross)) {
                runFullTune();
            } else if (pressed(square, prevSquare)) {
                runPSearchOnly();
            }
            // Re-read after a run: the button that aborted it may still be held, and must not start a new run.
            prevCross = gamepad1.cross;
            prevSquare = gamepad1.square;
        }
        stopAll();
    }

    // ======================================================================
    // FLOW
    // ======================================================================

    private void runFullTune() {
        aborted = false;
        pResults.clear();
        tuneVoltage = voltage;
        if (!characterizeFeedforward()) { idle("aborted / failed in feedforward. " + fResult); return; }
        if (!verifyF()) { idle("aborted / failed checking F. " + fResult); return; }
        if (!searchP()) { idle("aborted in the P search. " + pResult); return; }
        liveCheck();
    }

    private void runPSearchOnly() {
        aborted = false;
        pResults.clear();
        tuneVoltage = voltage;
        if (!searchP()) { idle("aborted in the P search. " + pResult); return; }
        liveCheck();
    }

    private void idle(String why) {
        stopAll();
        stage = "IDLE: " + why + "  (cross = full tune, square = P only)";
        detail = "";
    }

    // ======================================================================
    // STEP 1: FEEDFORWARD CHARACTERIZATION (open loop)
    // ======================================================================

    private boolean characterizeFeedforward() {
        ffPoints.clear();
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        List<Double> powers = new ArrayList<>();
        for (double pw : FF_POWERS) powers.add(pw);

        for (int i = 0; i < powers.size(); i++) {
            double pw = powers.get(i);
            stage = String.format("STEP 1/3 FEEDFORWARD: power %.2f (%d of %d), waiting for steady speed", pw, i + 1, powers.size());
            setPower(pw);
            if (!waitFlat()) return false;
            stage = String.format("STEP 1/3 FEEDFORWARD: power %.2f (%d of %d), measuring", pw, i + 1, powers.size());
            double[] avg = average(FF_AVERAGE_SEC);
            if (avg == null) return false;
            ffPoints.add(new double[]{pw, avg[0], avg[1]});
            if (avg[0] < 100) {
                fResult = String.format("FAILED: power %.2f only reached %.0f ticks/s. Check the shooter motor / direction / encoder.", pw, avg[0]);
                return false;
            }
            // Make sure the fit brackets the target: if the last planned power is still below it, add more.
            if (i == powers.size() - 1 && avg[0] < TARGET && pw < 1.0) powers.add(Math.min(1.0, pw + 0.12));
        }

        // Least squares: power = kS + kV * speed
        int n = ffPoints.size();
        double sx = 0, sy = 0, sxx = 0, sxy = 0;
        for (double[] pt : ffPoints) { sx += pt[1]; sy += pt[0]; sxx += pt[1] * pt[1]; sxy += pt[1] * pt[0]; }
        double den = n * sxx - sx * sx;
        if (Math.abs(den) < 1e-9) { fResult = "FAILED: speeds did not change with power"; return false; }
        kV = (n * sxy - sx * sy) / den;
        kS = (sy - kV * sx) / n;
        double mean = sy / n, ssTot = 0, ssRes = 0;
        for (double[] pt : ffPoints) {
            double pred = kS + kV * pt[1];
            ssRes += (pt[0] - pred) * (pt[0] - pred);
            ssTot += (pt[0] - mean) * (pt[0] - mean);
        }
        r2 = ssTot > 0 ? 1 - ssRes / ssTot : 0;
        fFit = Range.clip(32767.0 * (kS + kV * TARGET) / TARGET, 0, MAX_F);
        f = round3(fFit);
        fResult = String.format("fit: power = %.4f + %.6f x speed (R^2 %.4f) -> F at %.0f = %.3f", kS, kV, r2, TARGET, fFit);
        return true;
    }

    /** Hub loop with P = 0: F alone must hold the target. Corrects F by the speed ratio until it does. */
    private boolean verifyF() {
        for (int round = 1; round <= F_VERIFY_ROUNDS; round++) {
            stage = String.format("STEP 1/3 FEEDFORWARD: checking F %.3f in the hub loop (round %d)", f, round);
            runVelocity(0, f);
            if (!waitFlat()) return false;
            double[] avg = average(1.5);
            if (avg == null) return false;
            double err = TARGET - avg[0];
            detail = String.format("F %.3f alone holds %.0f (error %+.0f)", f, avg[0], err);
            if (Math.abs(err) <= F_TOL) {
                fResult += String.format("  | VERIFIED F = %.3f (holds %.0f alone)", f, avg[0]);
                return true;
            }
            if (avg[0] < 100) { fResult = "FAILED: wheel not turning in velocity mode"; return false; }
            f = round3(Range.clip(f * TARGET / avg[0], 0, MAX_F));
        }
        fResult += String.format("  | F = %.3f after %d corrections (not within %.0f; P will cover the rest)", f, F_VERIFY_ROUNDS, F_TOL);
        return true;
    }

    // ======================================================================
    // STEP 2: P SEARCH (load-disturbance recovery)
    // ======================================================================

    private boolean searchP() {
        List<Double> tested = new ArrayList<>();
        for (double cand : P_COARSE) {
            if (!testP(cand)) return false;
            tested.add(cand);
        }
        PResult pick = choose();
        if (pick != null) {
            // Refine: halfway to the neighbours of the pick.
            double lower = -1, upper = -1;
            Double[] sorted = tested.toArray(new Double[0]);
            Arrays.sort(sorted);
            for (Double t : sorted) {
                if (t < pick.p) lower = t;
                if (t > pick.p && upper < 0) upper = t;
            }
            if (lower > 0 && !testP(Math.round((lower + pick.p) / 2.0))) return false;
            if (upper > 0 && !testP(Math.round((upper + pick.p) / 2.0))) return false;
            pick = choose();
        }
        if (pick == null) {
            pResult = "NO P passed (all overshoot / ring / noisy). Kept P = " + p + ". Check F and the wheel.";
        } else {
            p = pick.p;
            pResult = String.format("CHOSE P = %.0f: recovery %.2f s, overshoot %.0f, hold noise %.1f RMS", pick.p, pick.recovery, pick.overshoot, pick.holdRms);
        }
        return true;
    }

    /** Smallest passing P whose recovery is within PICK_MARGIN of the fastest passing one. */
    private PResult choose() {
        double best = Double.MAX_VALUE;
        for (PResult r : pResults) if (r.ok) best = Math.min(best, r.recovery);
        PResult pick = null;
        for (PResult r : pResults) {
            if (r.ok && r.recovery <= best * PICK_MARGIN + 0.02 && (pick == null || r.p < pick.p)) pick = r;
        }
        return pick;
    }

    private boolean testP(double candP) {
        PResult r = new PResult();
        r.p = candP;
        String head = String.format("STEP 2/3 P SEARCH: P = %.0f", candP);

        // Settle at the target, then measure hold noise.
        stage = head + ": settling";
        runVelocity(candP, f);
        if (!waitInBand(SETTLE_TIMEOUT)) return false;
        stage = head + ": measuring hold noise";
        double t0 = now(), sumSq = 0, min = Double.MAX_VALUE, max = -Double.MAX_VALUE;
        int n = 0;
        while (now() - t0 < NOISE_SEC) {
            if (!tick()) return false;
            double e = velocity - TARGET;
            sumSq += e * e;
            min = Math.min(min, velocity);
            max = Math.max(max, velocity);
            n++;
        }
        r.holdRms = n > 0 ? Math.sqrt(sumSq / n) : 0;
        r.holdPp = max - min;

        // Simulated shots.
        double[] rec = new double[TRIALS];
        boolean allRecovered = true;
        for (int k = 0; k < TRIALS; k++) {
            stage = String.format("%s: simulated shot %d of %d", head, k + 1, TRIALS);
            // Coast with 0 power until the wheel has lost DIP (a ball's worth).
            shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            setPower(0);
            double td = now();
            while (velocity > TARGET - DIP && now() - td < DIP_TIMEOUT) {
                if (!tick()) return false;
            }
            // Re-engage at the same target and watch.
            runVelocity(candP, f);
            double te = now(), inBandSince = -1, recovered = -1, overshoot = 0;
            boolean started = false;
            int crossings = 0, lastSide = 0;
            while (now() - te < RECOVERY_WINDOW) {
                if (!tick()) return false;
                double t = now() - te, e = velocity - TARGET;
                if (Math.abs(e) <= TOL) {
                    if (inBandSince < 0) inBandSince = t;
                    if (recovered < 0 && t - inBandSince >= IN_BAND_HOLD) recovered = inBandSince;
                } else {
                    inBandSince = -1;
                }
                if (e > 0) started = true; // reached the target at least once
                if (started) overshoot = Math.max(overshoot, e);
                int side = e > TOL ? 1 : (e < -TOL ? -1 : 0);
                if (started && side != 0 && lastSide != 0 && side != lastSide) crossings++;
                if (side != 0) lastSide = side;
            }
            rec[k] = recovered < 0 ? Double.NaN : recovered;
            if (recovered < 0) allRecovered = false;
            r.overshoot = Math.max(r.overshoot, overshoot);
            r.ring = Math.max(r.ring, crossings);
            detail = String.format("P %.0f shot %d: recovery %s, overshoot %.0f", candP, k + 1,
                    recovered < 0 ? "NONE" : String.format("%.2f s", recovered), overshoot);
        }
        double[] sorted = rec.clone();
        Arrays.sort(sorted); // NaN sorts last
        r.recovery = sorted[TRIALS / 2];

        // Verdict. Noise limit is relative to the quietest (lowest) P tested so far.
        double baseRms = r.holdRms;
        for (PResult o : pResults) baseRms = Math.min(baseRms, o.holdRms);
        double noiseLimit = Math.max(baseRms * 1.8, baseRms + 12);
        if (!allRecovered || Double.isNaN(r.recovery)) { r.ok = false; r.why = "did not recover"; }
        else if (r.overshoot > OVERSHOOT_MAX) { r.ok = false; r.why = "overshoot"; }
        else if (r.ring > MAX_RING_CROSSINGS) { r.ok = false; r.why = "rings"; }
        else if (r.holdRms > noiseLimit) { r.ok = false; r.why = "noisy"; }
        else { r.ok = true; r.why = "ok"; }
        pResults.add(r);
        return true;
    }

    // ======================================================================
    // STEP 3: LIVE CHECK WITH REAL BALLS
    // ======================================================================

    private void liveCheck() {
        runVelocity(p, f);
        boolean burst = false;
        double burstLow = 0, below = 0, lastT = now(), backSince = -1;
        int bursts = 0;
        double lastDip = 0, lastBelow = 0, worstDip = 0, worstBelow = 0;

        while (opModeIsActive()) {
            if (!tick()) break; // cross = stop
            double t = now();
            if (pressed(gamepad1.dpad_up, lUp))       { p = Range.clip(p + 5, 0, 400); runVelocity(p, f); }
            if (pressed(gamepad1.dpad_down, lDown))   { p = Range.clip(p - 5, 0, 400); runVelocity(p, f); }
            if (pressed(gamepad1.dpad_right, lRight)) { f = round3(Range.clip(f + 0.05, 0, MAX_F)); runVelocity(p, f); }
            if (pressed(gamepad1.dpad_left, lLeft))   { f = round3(Range.clip(f - 0.05, 0, MAX_F)); runVelocity(p, f); }
            if (pressed(gamepad1.triangle, lTriangle)) { bursts = 0; lastDip = lastBelow = worstDip = worstBelow = 0; }
            lUp = gamepad1.dpad_up; lDown = gamepad1.dpad_down; lRight = gamepad1.dpad_right;
            lLeft = gamepad1.dpad_left; lTriangle = gamepad1.triangle;

            boolean shooting = gamepad1.circle;
            blocker.setPosition(shooting ? BLOCKER_OPEN : BLOCKER_BLOCK);
            double in = 0, tr = 0;
            if (gamepad1.right_bumper) { in = PIDF_INTAKE_TRANSFER_POWER; tr = PIDF_INTAKE_TRANSFER_POWER; }
            else if (gamepad1.left_bumper) in = INTAKE_IN_POWER;
            intake.setPower(in);
            transfer.setPower(tr);

            // Burst = blocker opens ... closed and back at speed for 0.15 s.
            boolean atSpeed = Math.abs(TARGET - velocity) <= TOL;
            if (!burst && shooting) { burst = true; burstLow = velocity; below = 0; backSince = -1; }
            if (burst) {
                burstLow = Math.min(burstLow, velocity);
                if (velocity < TARGET - TOL) below += t - lastT;
                if (shooting || !atSpeed) backSince = -1;
                else if (backSince < 0) backSince = t;
                else if (t - backSince >= 0.15) {
                    burst = false;
                    bursts++;
                    lastDip = TARGET - burstLow;
                    lastBelow = below;
                    worstDip = Math.max(worstDip, lastDip);
                    worstBelow = Math.max(worstBelow, lastBelow);
                }
            }
            lastT = t;
            stage = String.format("STEP 3/3 LIVE CHECK at P %.0f / F %.3f: shoot bursts (R1 + circle). dpad = adjust, cross = stop", p, f);
            detail = bursts == 0 ? "no bursts yet"
                    : String.format("bursts %d | last dip %.0f, below band %.2f s | worst dip %.0f, below band %.2f s",
                    bursts, lastDip, lastBelow, worstDip, worstBelow);
        }
        intake.setPower(0);
        transfer.setPower(0);
        blocker.setPosition(BLOCKER_BLOCK);
        idle("done. Values above are the last used.");
    }

    // ======================================================================
    // HELPERS
    // ======================================================================

    /** One loop: read sensors, check abort, draw telemetry. Returns false if stopping / aborted. */
    private boolean tick() {
        for (LynxModule hub : hubs) hub.clearBulkCache();
        velocity = shooter.getVelocity();
        double t = now();
        if (battery != null && t - lastVoltageSec > 0.5) {
            lastVoltageSec = t;
            double v = battery.getVoltage();
            if (v >= MIN_VALID_VOLTAGE) voltage = v;
        }
        // cross aborts anything that is running (the main loop reads its own press)
        boolean cross = gamepad1.cross;
        if (cross && !lCross && !stage.startsWith("IDLE")) aborted = true;
        lCross = cross;
        lSquare = gamepad1.square;
        draw();
        return opModeIsActive() && !aborted;
    }

    private void draw() {
        panels.addData("Target", commandTarget);
        panels.addData("Actual", velocity);
        panels.addData("Power", commandPower);
        panels.addData("Stage", stage);
        if (!detail.isEmpty()) panels.addData("Now", detail);
        panels.addData("Battery", String.format("%.2f V%s", voltage, voltage < 12.3 ? "  (LOW: charge it, F depends on voltage)" : ""));
        panels.addData("Target speed", String.format("%.0f ticks/s", TARGET));
        StringBuilder pts = new StringBuilder();
        for (double[] pt : ffPoints) pts.append(String.format("%.2f->%.0f  ", pt[0], pt[1]));
        if (pts.length() > 0) panels.addData("FF points (power->speed)", pts.toString());
        panels.addData("F", fResult);
        for (PResult r : pResults) {
            panels.addData(String.format("P %3.0f", r.p), String.format("%-4s rec %s  over %.0f  ring %d  noise %.1f rms / %.0f p-p",
                    r.ok ? "OK" : "X " + r.why, Double.isNaN(r.recovery) ? "none" : String.format("%.2fs", r.recovery),
                    r.overshoot, r.ring, r.holdRms, r.holdPp));
        }
        panels.addData("P", pResult);
        panels.addData("PASTE INTO BioBuzzConfig", String.format("SHOOTER_P = %.0f, SHOOTER_I = 0, SHOOTER_D = 0, SHOOTER_F = %.3f;%s",
                p, f, Double.isNaN(tuneVoltage) ? "" : String.format("  // tuned at %.2f V", tuneVoltage)));
        panels.update(telemetry);
    }

    /** Waits until two consecutive 0.5 s average speeds agree within FLAT_TOL (or the timeout). */
    private boolean waitFlat() {
        double start = now(), winStart = start, sum = 0, prev = Double.NaN;
        int n = 0;
        while (now() - start < FLAT_TIMEOUT) {
            if (!tick()) return false;
            sum += velocity;
            n++;
            if (now() - winStart >= 0.5) {
                double m = sum / n;
                if (!Double.isNaN(prev) && Math.abs(m - prev) < FLAT_TOL) return true;
                prev = m;
                sum = 0;
                n = 0;
                winStart = now();
            }
        }
        return true; // timed out: use what we have
    }

    /** Waits until the speed has stayed within TOL of the target for 0.5 s (or the timeout). */
    private boolean waitInBand(double timeout) {
        double start = now(), since = -1;
        while (now() - start < timeout) {
            if (!tick()) return false;
            if (Math.abs(velocity - TARGET) <= TOL) {
                if (since < 0) since = now();
                if (now() - since >= 0.5) return true;
            } else {
                since = -1;
            }
        }
        return true;
    }

    /** Average {speed, voltage} over the given time; null if aborted. */
    private double[] average(double sec) {
        double start = now(), sv = 0, svolt = 0;
        int n = 0;
        while (now() - start < sec) {
            if (!tick()) return null;
            sv += velocity;
            svolt += voltage;
            n++;
        }
        return n == 0 ? new double[]{velocity, voltage} : new double[]{sv / n, svolt / n};
    }

    private void setPower(double pw) {
        commandPower = pw;
        commandTarget = 0;
        shooter.setPower(pw);
    }

    private void runVelocity(double pGain, double fGain) {
        if (shooter.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(pGain, 0, 0, fGain));
        shooter.setVelocity(TARGET);
        commandTarget = TARGET;
        commandPower = fGain * TARGET / 32767.0;
    }

    private void stopAll() {
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setPower(0);
        intake.setPower(0);
        transfer.setPower(0);
        blocker.setPosition(BLOCKER_BLOCK);
    }

    private static boolean pressed(boolean now, boolean prev) {
        return now && !prev;
    }

    private static double now() {
        return System.nanoTime() / 1e9;
    }

    private static double round3(double v) {
        return Math.round(v * 1000.0) / 1000.0;
    }
}
