package org.firstinspires.ftc.teamcode.biobuzz.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Bang-bang shooter test: no PID, no hub velocity control. Every loop the flywheel gets either
 *   HIGH_POWER (default 1.0) when its speed is BELOW the target, or
 *   LOW_POWER  (default 0.3) when its speed is AT / ABOVE the target.
 * Panels plots the actual speed against the target so you can see how tightly it holds.
 *
 * VIEW IT: laptop on the robot's Wi-Fi, open http://192.168.43.1:8001, add a GRAPH widget and tick:
 *   Graph 1 (ticks/s):  "Target", "Actual", "Shot gate"
 *   Graph 2:            "Error" (Target - Actual; positive = too slow), "Power"
 *
 * PANELS: Configurables tab -> "BangBangShooterTest": TARGET, HIGH_POWER, LOW_POWER, DEADBAND,
 * INTAKE_WITH_TRANSFER. Edit them while the OpMode runs; they take effect on the next loop.
 *   DEADBAND = hysteresis in ticks/s. 0 = switch exactly at the target. With e.g. 20 it goes to HIGH only
 *   below (target - 20) and to LOW only above (target + 20), and holds whatever it was doing in between.
 *
 * gamepad1:
 *   cross (A)          = spin up / stop
 *   circle (B, hold)   = open the blocker (balls go into the wheel)
 *   right trigger / right bumper = intake (INTAKE_WITH_TRANSFER) + transfer 1.0, left bumper = intake only
 *   triangle (Y)       = clear the shot stats
 *
 * Standalone on purpose (same robot config as ShooterGraphTest).
 */
@Configurable
@TeleOp(name = "Bang Bang Shooter Test", group = "BioBuzz")
public class BangBangShooterTest extends OpMode {

    // ================= COPY THESE FROM YOUR CONFIG =================
    private static final String SHOOTER = "shooter", INTAKE = "intake", TRANSFER = "transfer", BLOCKER = "blocker";
    private static final DcMotorSimple.Direction SHOOTER_DIR = DcMotorSimple.Direction.REVERSE;
    private static final DcMotorSimple.Direction INTAKE_DIR = DcMotorSimple.Direction.FORWARD;
    private static final DcMotorSimple.Direction TRANSFER_DIR = DcMotorSimple.Direction.REVERSE;
    private static final double BLOCKER_BLOCK = 0.49, BLOCKER_OPEN = 0.68;
    private static final double INTAKE_POWER = 1.0, TRANSFER_POWER = 1.0;
    /** Within this many ticks/s of the target counts as "at speed". */
    private static final double TOL = 25;
    // ===============================================================

    // ================= EDITABLE IN PANELS (Configurables tab) =================
    /** Target flywheel velocity (ticks/s). */
    public static double TARGET = 1500;
    /** Power while the wheel is below the target. */
    public static double HIGH_POWER = 1.0;
    /** Power while the wheel is at / above the target. */
    public static double LOW_POWER = 0.3;
    /** Hysteresis band (ticks/s) around the target; 0 = switch exactly at the target. */
    public static double DEADBAND = 0;
    /** Intake power whenever it runs together with the transfer. */
    public static double INTAKE_WITH_TRANSFER = 1.0;
    // ==========================================================================

    private static final double BURST_END_HOLD_SEC = 0.15;

    private DcMotorEx shooter;
    private DcMotor intake, transfer;
    private Servo blocker;
    private TelemetryManager panels;

    private boolean spinning = false;
    private boolean highOn = true; // current bang-bang state (used for the deadband hold)

    // Burst stats: one burst = from the blocker opening until the wheel is back at speed after it closes.
    private boolean burstActive;
    private double burstStart, burstLow, belowBandSec, lastTickSec, backAtSpeedSince;
    private int bursts;
    private double lastDip, lastBelowBand, worstDip, worstBelowBand;

    private boolean lCross, lTriangle;

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
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // raw power; we only read the velocity
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        blocker.setPosition(BLOCKER_BLOCK);

        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        panels.addData("Status", "Ready. Open http://192.168.43.1:8001, add a Graph, press cross (A) to spin.");
        panels.update(telemetry);
    }

    @Override
    public void loop() {
        double t = System.nanoTime() / 1e9;

        /* ---------- INPUT ---------- */
        boolean cross = gamepad1.cross, triangle = gamepad1.triangle;
        boolean circle = gamepad1.circle;
        if (cross && !lCross) {
            spinning = !spinning;
            highOn = true;
        }
        if (triangle && !lTriangle) clearStats();
        lCross = cross; lTriangle = triangle;

        double high = Range.clip(HIGH_POWER, -1, 1);
        double low = Range.clip(LOW_POWER, -1, 1);
        double band = Math.max(0, DEADBAND);

        /* ---------- BANG-BANG ---------- */
        double velocity = shooter.getVelocity();
        double power = 0;
        if (spinning) {
            if (velocity < TARGET - band) highOn = true;
            else if (velocity >= TARGET + band) highOn = false;
            // inside the band: keep the previous state (with band = 0 this switches exactly at TARGET)
            power = highOn ? high : low;
        }
        shooter.setPower(power);
        double command = spinning ? TARGET : 0;

        /* ---------- FEED ---------- */
        blocker.setPosition(circle ? BLOCKER_OPEN : BLOCKER_BLOCK);
        double intakePower = 0, transferPower = 0;
        if (gamepad1.right_trigger > 0.5 || gamepad1.right_bumper) {
            intakePower = INTAKE_WITH_TRANSFER;
            transferPower = TRANSFER_POWER;
        } else if (gamepad1.left_bumper) {
            intakePower = INTAKE_POWER;
        }
        intake.setPower(intakePower);
        transfer.setPower(transferPower);

        trackBurst(t, circle, velocity);

        /* ---------- GRAPH ---------- */
        panels.addData("Target", command);
        panels.addData("Actual", velocity);
        panels.addData("Error", command - velocity);
        panels.addData("Power", power);
        panels.addData("Shot gate", TARGET - 300 + (circle ? 40 : 0));

        panels.addData("Flywheel", spinning ? (Math.abs(TARGET - velocity) <= TOL ? "AT SPEED" : (velocity < TARGET ? "below (HIGH)" : "above (LOW)")) : "OFF");
        panels.addData("Bang-bang", String.format("HIGH=%.2f LOW=%.2f DEADBAND=%.0f  now %s", high, low, band, spinning ? (highOn ? "HIGH" : "LOW") : "off"));
        panels.addData("Bursts", bursts);
        if (bursts > 0) {
            panels.addData("Last burst dip / time below band", String.format("%.0f ticks/s / %.2f s", lastDip, lastBelowBand));
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
            endBurst();
        }
        // Safety: a burst that never recovers (jam / wrong speed) is closed after 5 s so stats do not freeze.
        if (burstActive && t - burstStart > 5.0 && !blockerOpen) endBurst();
    }

    private void endBurst() {
        bursts++;
        lastDip = TARGET - burstLow;
        lastBelowBand = belowBandSec;
        worstDip = Math.max(worstDip, lastDip);
        worstBelowBand = Math.max(worstBelowBand, lastBelowBand);
        burstActive = false;
    }

    private void clearStats() {
        bursts = 0;
        lastDip = lastBelowBand = worstDip = worstBelowBand = 0;
        burstActive = false;
    }
}
