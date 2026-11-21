package org.firstinspires.ftc.teamcode.biobuzz.teleop;

import static org.firstinspires.ftc.teamcode.biobuzz.teleop.BioBuzzConfig.*;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import java.util.ArrayList;
import java.util.List;

/**
 * Finds where the turret's magnetic limit switch triggers, so the turret can re-zero itself on it.
 *
 * The magnet doesn't trigger at a single point, it triggers over a band of a few degrees, and the
 * edge you see depends on which way you're turning (sensor hysteresis plus loop latency). So this
 * records the entry and exit edge of every pass, takes the band CENTER, and averages the CCW and
 * CW centers. That number doesn't depend on direction: put it in TURRET_LIMIT_ANGLE_DEG.
 *
 * It also checks for skipping: each full turn the magnet should show up exactly 360 deg later. "Drift"
 * is how far off it was. A steady drift with the same sign every pass means TURRET_TICKS_PER_DEGREE
 * (the gear ratio) is wrong. Random jumps mean the gears are actually skipping.
 *
 * SETUP: point the turret straight out the front (90 deg, TURRET_START_ANGLE_DEG) BEFORE you press init.
 *
 * gamepad1:
 *   left stick X   = drive turret by hand (cancels the sweep)
 *   A              = slow sweep CCW (+)
 *   B              = slow sweep CW (-)
 *   X              = stop
 *   dpad up / down = sweep power +- 0.02 (open loop; the encoder is wired opposite to motor power, so no velocity PID)
 *   Y              = clear recorded passes
 *   back           = re-zero: the current position becomes TURRET_START_ANGLE_DEG (90)
 */
@TeleOp(name = "Turret Limit Finder", group = "BioBuzz Tuning")
public class TurretLimitFinder extends LinearOpMode {

    /** Passes narrower than this are noise / a direction change inside the band. */
    private static final double MIN_BAND_DEG = 0.5;
    private static final double MANUAL_POWER = 0.3;

    private static class Pass {
        final double entryDeg, exitDeg;
        final int dir; // +1 = CCW, -1 = CW

        Pass(double entryDeg, double exitDeg, int dir) {
            this.entryDeg = entryDeg;
            this.exitDeg = exitDeg;
            this.dir = dir;
        }

        double center() { return (entryDeg + exitDeg) / 2.0; }
        double width()  { return Math.abs(exitDeg - entryDeg); }
    }

    private DcMotorEx motor;
    private TouchSensor limit; // configured as "REV Touch Sensor" in the robot config
    private int tickOffset = 0;

    private final List<Pass> ccw = new ArrayList<>();
    private final List<Pass> cw = new ArrayList<>();
    private String lastEvent = "none yet";

    @Override
    public void runOpMode() {
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        motor = hardwareMap.get(DcMotorEx.class, TURRET);
        motor.setDirection(TURRET_DIR);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        limit = hardwareMap.get(TouchSensor.class, TURRET_LIMIT);

        while (opModeInInit()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();
            telemetry.addLine("Point the turret STRAIGHT FORWARD, then press start.");
            telemetry.addData("Magnet", triggered() ? "DETECTED" : "no");
            telemetry.addData("Angle", "%.2f deg", angleDeg(ticks()));
            telemetry.update();
        }

        double sweepPower = 0.15;
        int sweepDir = 0; // 0 = not sweeping

        boolean prevTriggered = triggered();
        int prevTicks = ticks();
        double entryDeg = 0;
        int entryDir = 0;
        int lastMoveDir = 0;
        boolean bandValid = false; // true while inside the band and we saw the entry edge

        boolean prevUp = false, prevDown = false, prevY = false, prevBack = false;
        long lastNanos = System.nanoTime();
        double loopMs = 0;

        while (opModeIsActive()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();

            long now = System.nanoTime();
            loopMs = 0.9 * loopMs + 0.1 * (now - lastNanos) / 1e6;
            lastNanos = now;

            // ---------- buttons ----------
            if (gamepad1.a) sweepDir = 1;
            if (gamepad1.b) sweepDir = -1;
            if (gamepad1.x) sweepDir = 0;
            if (gamepad1.dpad_up && !prevUp) sweepPower = Math.min(sweepPower + 0.02, 0.5);
            if (gamepad1.dpad_down && !prevDown) sweepPower = Math.max(sweepPower - 0.02, 0.1);
            if (gamepad1.y && !prevY) {
                ccw.clear();
                cw.clear();
                lastEvent = "cleared";
            }
            if (gamepad1.back && !prevBack) {
                tickOffset = -motor.getCurrentPosition();
                ccw.clear();
                cw.clear();
                bandValid = false;
                lastEvent = "re-zeroed (passes cleared)";
            }
            prevUp = gamepad1.dpad_up;
            prevDown = gamepad1.dpad_down;
            prevY = gamepad1.y;
            prevBack = gamepad1.back;

            // ---------- drive ----------
            double stick = -gamepad1.left_stick_x; // stick left = CCW = +
            if (Math.abs(stick) > 0.05) {
                sweepDir = 0;
                motor.setPower(TURRET_POWER_SIGN * stick * MANUAL_POWER);
            } else if (sweepDir != 0) {
                motor.setPower(TURRET_POWER_SIGN * sweepDir * sweepPower);
            } else {
                motor.setPower(0);
            }

            // ---------- edge detection ----------
            int curTicks = ticks();
            boolean curTriggered = triggered();
            int motionDir = Integer.signum(curTicks - prevTicks);
            // At slow sweeps the encoder often doesn't tick between loops, so remember the last real direction.
            if (motionDir != 0) lastMoveDir = motionDir;
            // The edge happened somewhere between the two samples: split the difference.
            double edgeDeg = angleDeg((prevTicks + curTicks) / 2.0);

            if (curTriggered && !prevTriggered) {
                entryDeg = edgeDeg;
                entryDir = lastMoveDir;
                bandValid = lastMoveDir != 0;
            } else if (curTriggered && motionDir != 0 && motionDir != entryDir) {
                bandValid = false; // reversed inside the band, this pass is junk
            } else if (!curTriggered && prevTriggered) {
                if (bandValid && Math.abs(edgeDeg - entryDeg) >= MIN_BAND_DEG) {
                    record(new Pass(entryDeg, edgeDeg, entryDir));
                } else {
                    lastEvent = "pass ignored (reversed in band / too narrow)";
                }
                bandValid = false;
            }
            prevTriggered = curTriggered;
            prevTicks = curTicks;

            // ---------- telemetry ----------
            telemetry.addData("Magnet", curTriggered ? "DETECTED" : "no");
            telemetry.addData("Angle", "%.2f deg (wrapped %.2f)", angleDeg(curTicks), wrap180(angleDeg(curTicks)));
            telemetry.addData("Sweep", sweepDir == 0 ? "off" : (sweepDir > 0 ? "CCW" : "CW") + String.format(" @ %.2f power", sweepPower));
            telemetry.addData("Loop", "%.1f ms", loopMs);
            telemetry.addData("Last", lastEvent);
            telemetry.addLine();
            summarize("CCW", ccw);
            summarize("CW ", cw);
            telemetry.addLine();
            if (!ccw.isEmpty() && !cw.isEmpty()) {
                double c = wrap180((avgCenter(ccw) + avgCenter(cw)) / 2.0);
                telemetry.addData(">> TURRET_LIMIT_ANGLE_DEG", "%.2f", c);
                telemetry.addData("   hysteresis (CCW vs CW center)", "%.2f deg", wrap180(avgCenter(ccw) - avgCenter(cw)));
            } else {
                telemetry.addLine(">> Need at least one CCW AND one CW pass (A then B).");
            }
            telemetry.update();
        }
        motor.setPower(0);
    }

    private void record(Pass p) {
        List<Pass> list = p.dir > 0 ? ccw : cw;
        String drift = "";
        if (!list.isEmpty()) {
            double delta = p.center() - list.get(list.size() - 1).center();
            double turns = Math.round(delta / 360.0);
            if (turns != 0) {
                drift = String.format(", drift %.2f deg over %.0f turn(s)", delta - turns * 360.0, Math.abs(turns));
            }
        }
        list.add(p);
        lastEvent = String.format("%s pass: center %.2f, width %.2f%s",
                p.dir > 0 ? "CCW" : "CW", wrap180(p.center()), p.width(), drift);
    }

    private void summarize(String name, List<Pass> list) {
        if (list.isEmpty()) {
            telemetry.addData(name, "no passes");
            return;
        }
        double minC = Double.MAX_VALUE, maxC = -Double.MAX_VALUE, width = 0, entry = 0;
        for (Pass p : list) {
            double c = wrap180(p.center());
            minC = Math.min(minC, c);
            maxC = Math.max(maxC, c);
            width += p.width();
            entry += wrap180(p.entryDeg);
        }
        int n = list.size();
        telemetry.addData(name, "n=%d  center %.2f  (spread %.2f)  width %.2f  leading edge %.2f",
                n, wrap180(avgCenter(list)), maxC - minC, width / n, entry / n);
    }

    /** Average of the centers, wrapped to the first pass so passes on different turns line up. */
    private static double avgCenter(List<Pass> list) {
        double ref = list.get(0).center();
        double sum = 0;
        for (Pass p : list) sum += ref + wrap180(p.center() - ref);
        return sum / list.size();
    }

    private boolean triggered() {
        // Configured as a REV Touch Sensor: isPressed() is already the inverted digital line (LOW = pressed),
        // and the magnetic switch pulls LOW when the magnet is there, so pressed == magnet detected. Don't invert again.
        return limit.isPressed();
    }

    private int ticks() {
        return motor.getCurrentPosition() + tickOffset;
    }

    private static double angleDeg(double ticks) {
        return TURRET_START_ANGLE_DEG + ticks / TURRET_TICKS_PER_DEGREE;
    }

    private static double wrap180(double deg) {
        deg %= 360.0;
        if (deg > 180.0) deg -= 360.0;
        if (deg <= -180.0) deg += 360.0;
        return deg;
    }
}
