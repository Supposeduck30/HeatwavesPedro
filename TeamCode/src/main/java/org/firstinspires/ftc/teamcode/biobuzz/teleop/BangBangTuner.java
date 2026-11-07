package org.firstinspires.ftc.teamcode.biobuzz.teleop;

import static org.firstinspires.ftc.teamcode.biobuzz.teleop.BioBuzzConfig.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * Tune the bang-bang shooter live: change ON power, OFF power and hysteresis from the gamepad
 * without redeploying. Nothing is saved: copy the final values into BioBuzzConfig.
 *
 * gamepad1 (Logitech names in brackets):
 *   cross (A)              = spin up / stop (the flywheel is OFF until you press it)
 *   dpad up / down         = OFF power + / -
 *   dpad right / left      = ON power + / -
 *   right / left stick click = hysteresis + / - 1
 *   options (start)        = power step 0.05 <-> 0.01
 *   triangle (Y)           = restart the stats (press before a shot burst)
 *   circle (B)             = HOLD to open the blocker
 *   right bumper           = intake + transfer (feed), left bumper = intake only
 *
 * HOW TO TUNE:
 *   1. Spin up with no balls. Tune OFF power until "Idle min / max" is within about +/-15 of the target.
 *      Speed creeping high and staying there = OFF too high. Big saw-tooth drops = OFF too low.
 *   2. Press triangle, then feed a burst. Watch "Lowest speed", "Overshoot" and "Recovery".
 *      Overshoot large after recovery = lower ON. Recovery slow = keep ON at 1.0 and slow the feed.
 *   3. Copy the "Paste into BioBuzzConfig" lines.
 */
@TeleOp(name = "Bang-Bang Tuner", group = "BioBuzz")
public class BangBangTuner extends OpMode {

    /** Ignore this long after spin-up / any change while the wheel settles. */
    private static final double SETTLE_SEC = 2.0;

    private DcMotorEx shooter;
    private DcMotor intake, transfer;
    private Servo blocker;
    private VoltageSensor battery;

    private final double target = SHOOTER_FIXED_VELOCITY;
    private double onPower = BANG_BANG_ON_POWER;
    private double offPower = BANG_BANG_OFF_POWER;
    private double hysteresis = BANG_BANG_HYSTERESIS;
    private double step = 0.05;

    private boolean spinning = false;
    private boolean motorOn = false;

    // Raw encoder readout
    private int lastPos;
    private long lastLoopNs;
    private double ticksPerSecMeasured;
    private int ticksThisLoop;
    private int switches = 0;

    // Stats, restarted on any change
    private long startNs;
    private double minSpeed, maxSpeed, sum;
    private int samples;
    private double lowestSpeed, overshoot;
    private boolean recovering;
    private long dipStartNs;
    private double lastRecoverySec, worstRecoverySec;

    private boolean lastUp, lastDown, lastLeft, lastRight, lastCross, lastTriangle, lastOptions;
    private boolean lastRs, lastLs;

    @Override
    public void init() {
        shooter  = hardwareMap.get(DcMotorEx.class, SHOOTER);
        intake   = hardwareMap.get(DcMotor.class, INTAKE);
        transfer = hardwareMap.get(DcMotor.class, TRANSFER);
        blocker  = hardwareMap.get(Servo.class, BLOCKER);
        shooter.setDirection(SHOOTER_DIR);
        intake.setDirection(INTAKE_DIR);
        transfer.setDirection(TRANSFER_DIR);
        // Raw power, no built-in velocity PID. getVelocity() still reads the encoder.
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // BRAKE at zero power, same as the teleops (they were tuned with it first).
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blocker.setPosition(BLOCKER_BLOCK);
        if (hardwareMap.voltageSensor.iterator().hasNext()) {
            battery = hardwareMap.voltageSensor.iterator().next();
        }
        resetStats();
        lastPos = shooter.getCurrentPosition();
        lastLoopNs = System.nanoTime();

        telemetry.addLine("Bang-Bang Tuner ready. Flywheel is OFF; press cross (A) to spin.");
        telemetry.update();
    }

    @Override
    public void loop() {
        long now = System.nanoTime();

        /* ---------- INPUT ---------- */
        boolean up = gamepad1.dpad_up, down = gamepad1.dpad_down;
        boolean left = gamepad1.dpad_left, right = gamepad1.dpad_right;
        boolean cross = gamepad1.cross, triangle = gamepad1.triangle, options = gamepad1.options;
        boolean rs = gamepad1.right_stick_button, ls = gamepad1.left_stick_button;

        if (up && !lastUp)       { offPower = clipPower(offPower + step); resetStats(); }
        if (down && !lastDown)   { offPower = clipPower(offPower - step); resetStats(); }
        if (right && !lastRight) { onPower  = clipPower(onPower + step);  resetStats(); }
        if (left && !lastLeft)   { onPower  = clipPower(onPower - step);  resetStats(); }
        if (rs && !lastRs)       { hysteresis = hysteresis + 1;            resetStats(); }
        if (ls && !lastLs)       { hysteresis = Math.max(0, hysteresis - 1); resetStats(); }
        if (options && !lastOptions) step = (step == 0.05) ? 0.01 : 0.05;

        if (cross && !lastCross) {
            spinning = !spinning;
            motorOn = false;
            resetStats();
        }
        if (triangle && !lastTriangle) resetStats();

        lastUp = up; lastDown = down; lastLeft = left; lastRight = right;
        lastCross = cross; lastTriangle = triangle; lastOptions = options;
        lastRs = rs; lastLs = ls;

        /* ---------- BANG-BANG ---------- */
        double velocity = shooter.getVelocity();

        // Count the encoder ticks ourselves as a cross-check on getVelocity()
        int pos = shooter.getCurrentPosition();
        ticksThisLoop = pos - lastPos;
        double dt = (now - lastLoopNs) / 1e9;
        if (dt > 0) ticksPerSecMeasured = ticksThisLoop / dt;
        lastPos = pos;
        lastLoopNs = now;

        if (spinning) {
            boolean wasOn = motorOn;
            if (velocity < target - hysteresis) motorOn = true;
            else if (velocity > target + hysteresis) motorOn = false;
            // in between: keep the previous state
            if (motorOn != wasOn) switches++;
            shooter.setPower(motorOn ? onPower : offPower);
        } else {
            shooter.setPower(0);
        }

        boolean settled = spinning && (now - startNs) / 1e9 >= SETTLE_SEC;
        if (settled) updateStats(now, velocity);

        /* ---------- FEED ---------- */
        blocker.setPosition(gamepad1.circle ? BLOCKER_OPEN : BLOCKER_BLOCK);
        double intakePower = 0, transferPower = 0;
        if (gamepad1.left_bumper) {
            intakePower = INTAKE_IN_POWER;
        } else if (gamepad1.right_bumper) {
            intakePower = INTAKE_IN_POWER;
            transferPower = TRANSFER_IN_POWER;
        }
        intake.setPower(intakePower);
        transfer.setPower(transferPower);

        /* ---------- TELEMETRY ---------- */
        double error = target - velocity;
        telemetry.addData("Flywheel", spinning ? (motorOn ? "SPINNING  [ON]" : "SPINNING  [off]") : "OFF  (cross / A to spin)");
        telemetry.addData("Target (ticks/s)", "%.0f  (from SHOOTER_FIXED_VELOCITY)", target);
        telemetry.addData("Actual (ticks/s)", "%.0f   error %.0f", velocity, spinning ? error : 0);
        if (battery != null) telemetry.addData("Battery (V)", "%.2f", battery.getVoltage());

        telemetry.addLine();
        telemetry.addData("--- ENCODER TICKS ---", "");
        telemetry.addData("Total ticks (position)", "%d", pos);
        telemetry.addData("Ticks this loop", "%d   (loop %.0f ms)", ticksThisLoop, dt * 1000);
        telemetry.addData("Ticks/s (counted)", "%.0f", ticksPerSecMeasured);
        telemetry.addData("Ticks/s (getVelocity)", "%.0f", velocity);

        telemetry.addLine();
        telemetry.addData("OFF power", "%.2f   (dpad up / down)", offPower);
        telemetry.addData("ON power", "%.2f   (dpad right / left)", onPower);
        telemetry.addData("Hysteresis", "%.0f   (right / left stick click)", hysteresis);
        telemetry.addData("Step", "%.2f   (options to switch)", step);
        telemetry.addData("Estimated hold power", "%.2f  (keep OFF below, ON above)", SHOOTER_F * target / 32767.0);

        telemetry.addLine();
        if (!settled) {
            telemetry.addData("Stats", spinning ? "settling..." : "-");
        } else {
            double runSec = (now - startNs) / 1e9 - SETTLE_SEC;
            telemetry.addData("Idle min / max", "%.0f / %.0f", minSpeed, maxSpeed);
            telemetry.addData("Avg speed", "%.1f", samples > 0 ? sum / samples : 0);
            telemetry.addData("Switches / s", "%.1f", runSec > 0 ? switches / runSec : 0);
            telemetry.addData("Lowest speed (shot dip)", "%.0f", lowestSpeed);
            telemetry.addData("Overshoot above target", "%.0f", overshoot);
            telemetry.addData("Recovery (last / worst)", "%s / %s%s",
                    seconds(lastRecoverySec), seconds(worstRecoverySec), recovering ? "  (recovering...)" : "");
        }

        telemetry.addLine();
        telemetry.addData("SEND TO CLAUDE", "on=%.2f off=%.2f hyst=%.0f min=%.0f max=%.0f lowest=%.0f recovery=%s",
                onPower, offPower, hysteresis,
                settled ? minSpeed : 0, settled ? maxSpeed : 0, settled ? lowestSpeed : 0,
                seconds(worstRecoverySec));
        telemetry.addLine("Paste into BioBuzzConfig:");
        telemetry.addLine(String.format("  BANG_BANG_ON_POWER = %.2f;", onPower));
        telemetry.addLine(String.format("  BANG_BANG_OFF_POWER = %.2f;", offPower));
        telemetry.addLine(String.format("  BANG_BANG_HYSTERESIS = %.0f;", hysteresis));
        telemetry.update();
    }

    @Override
    public void stop() {
        shooter.setPower(0);
        intake.setPower(0);
        transfer.setPower(0);
    }

    private void updateStats(long now, double velocity) {
        double error = target - velocity;
        minSpeed = Math.min(minSpeed, velocity);
        maxSpeed = Math.max(maxSpeed, velocity);
        sum += velocity;
        samples++;
        lowestSpeed = Math.min(lowestSpeed, velocity);
        overshoot = Math.max(overshoot, velocity - target);

        // A dip starts when the speed falls below the ready tolerance and ends when it is back within it.
        if (!recovering && error > SHOOTER_READY_TOLERANCE) {
            recovering = true;
            dipStartNs = now;
        } else if (recovering && error <= SHOOTER_READY_TOLERANCE) {
            recovering = false;
            lastRecoverySec = (now - dipStartNs) / 1e9;
            worstRecoverySec = Math.max(worstRecoverySec, lastRecoverySec);
        }
    }

    private void resetStats() {
        startNs = System.nanoTime();
        minSpeed = Double.MAX_VALUE;
        maxSpeed = -Double.MAX_VALUE;
        sum = 0;
        samples = 0;
        switches = 0;
        lowestSpeed = Double.MAX_VALUE;
        overshoot = 0;
        recovering = false;
        lastRecoverySec = -1;
        worstRecoverySec = -1;
    }

    private static double clipPower(double v) {
        return Range.clip(Math.round(v * 100.0) / 100.0, 0, 1);
    }

    private static String seconds(double s) {
        return s < 0 ? "-" : String.format("%.2f s", s);
    }
}
