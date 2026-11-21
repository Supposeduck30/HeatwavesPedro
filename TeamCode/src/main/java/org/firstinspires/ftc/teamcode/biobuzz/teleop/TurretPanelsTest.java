package org.firstinspires.ftc.teamcode.biobuzz.teleop;

import static org.firstinspires.ftc.teamcode.biobuzz.teleop.BioBuzzConfig.*;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * Turret-only test: just the turret motor (and the magnet switch if it is in the config). No drivetrain, no
 * Pedro, no goal math. Drive the turret by hand, or tell it an angle and watch it get there on Panels.
 * Uses the same motor name, direction, ticks-per-degree and angle convention as {@link BioBuzzTurret}
 * (0 = robot right, 90 = robot front, 180 = left, positive = counter-clockwise).
 *
 * START with the turret pointing straight forward: the encoder is zeroed at init and that spot reads
 * TURRET_START_ANGLE_DEG (90). There is NO homing and NO limit, so keep the wiring free to spin.
 *
 * VIEW IT: laptop on the robot's Wi-Fi, open http://192.168.43.1:8001, add a GRAPH widget and tick:
 *   Graph 1 (deg):  "Target", "Actual"        Graph 2: "Error" (Target - Actual), "Power"
 *   "Magnet" is 1 while the limit switch sees the magnet (0 otherwise / no switch).
 *
 * PANELS: Configurables tab -> "TurretPanelsTest": TARGET_ANGLE, KP, KD, MIN_POWER, MAX_POWER, DEADBAND_DEG,
 * SHORTEST_WAY, POWER_SIGN, MANUAL_MAX_POWER, NUDGE_DEG. Edits apply on the next loop.
 *
 * gamepad1:
 *   cross (A)          = toggle HOLD mode (PD drives to TARGET_ANGLE) / MANUAL mode (power = left stick x)
 *   left stick x       = manual power in MANUAL mode (right = counter-clockwise, positive angle)
 *   dpad left / right  = TARGET_ANGLE -/+ NUDGE_DEG (left = clockwise)
 *   dpad up / down     = TARGET_ANGLE +/- 90
 *   square (X)         = TARGET_ANGLE = 90 (straight forward)
 *   triangle (Y)       = declare the turret's current position to be 90 deg (zero the encoder)
 *   circle (B)         = stop the motor and switch to MANUAL
 */
@Configurable
@TeleOp(name = "Turret Panels Test", group = "BioBuzz")
public class TurretPanelsTest extends OpMode {

    // ================= EDITABLE IN PANELS (Configurables tab) =================
    /** Angle HOLD mode drives to (deg, 90 = front). Not wrapped: 450 is one full turn past 90. */
    public static double TARGET_ANGLE = TURRET_START_ANGLE_DEG;
    public static double KP = TURRET_KP;
    public static double KD = TURRET_KD;
    /** Smallest power that still moves the turret (overcomes friction) when outside the deadband. */
    public static double MIN_POWER = TURRET_MIN_POWER;
    public static double MAX_POWER = TURRET_MAX_POWER;
    /** Inside this many degrees of the target the motor is stopped. */
    public static double DEADBAND_DEG = TURRET_DEADBAND_DEG;
    /** true = turn the shortest way (error within +-180, like the real turret); false = go to the exact angle. */
    public static boolean SHORTEST_WAY = true;
    /** Sign applied to every motor power (-1 on this robot, so that positive power raises the angle). */
    public static double POWER_SIGN = TURRET_POWER_SIGN;
    /** Max power from the stick in MANUAL mode. */
    public static double MANUAL_MAX_POWER = 0.3;
    /** Degrees per dpad left / right press. */
    public static double NUDGE_DEG = 10;
    // ==========================================================================

    private DcMotorEx motor;
    private TouchSensor limit;
    private TelemetryManager panels;

    private int offsetTicks = 0;
    private boolean hold = false;
    private double prevAngle;
    private long lastNanos;

    private boolean lCross, lUp, lDown, lLeft, lRight, lSquare, lTriangle;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, TURRET);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(TURRET_DIR);
        try {
            limit = hardwareMap.get(TouchSensor.class, TURRET_LIMIT);
        } catch (RuntimeException e) {
            limit = null; // not in this config
        }
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        prevAngle = angle();
        lastNanos = System.nanoTime();
        panels.addData("Status", "Ready. Point the turret straight forward. cross (A) = hold / manual.");
        panels.update(telemetry);
    }

    @Override
    public void start() {
        prevAngle = angle();
        lastNanos = System.nanoTime();
    }

    @Override
    public void loop() {
        /* ---------- INPUT ---------- */
        boolean cross = gamepad1.cross, up = gamepad1.dpad_up, down = gamepad1.dpad_down;
        boolean left = gamepad1.dpad_left, right = gamepad1.dpad_right;
        boolean square = gamepad1.square, triangle = gamepad1.triangle;

        if (cross && !lCross) hold = !hold;
        if (gamepad1.circle) hold = false;
        if (left && !lLeft) TARGET_ANGLE -= NUDGE_DEG;
        if (right && !lRight) TARGET_ANGLE += NUDGE_DEG;
        if (up && !lUp) TARGET_ANGLE += 90;
        if (down && !lDown) TARGET_ANGLE -= 90;
        if (square && !lSquare) TARGET_ANGLE = TURRET_FRONT_ANGLE_DEG;
        if (triangle && !lTriangle) {
            offsetTicks = -motor.getCurrentPosition();
            prevAngle = angle();
        }
        lCross = cross; lUp = up; lDown = down; lLeft = left; lRight = right; lSquare = square; lTriangle = triangle;

        /* ---------- CONTROL ---------- */
        double now = angle();
        double error = TARGET_ANGLE - now;
        if (SHORTEST_WAY) error = wrap180(error);

        long nanos = System.nanoTime();
        double dt = Math.max((nanos - lastNanos) / 1e9, 1e-3);
        lastNanos = nanos;
        double velocity = (now - prevAngle) / dt; // deg/s
        prevAngle = now;

        double power = 0;
        if (gamepad1.circle) {
            power = 0;
        } else if (hold) {
            if (Math.abs(error) > DEADBAND_DEG) {
                power = KP * error - KD * velocity;
                power = Range.clip(power, -MAX_POWER, MAX_POWER);
                if (Math.abs(power) < MIN_POWER) power = Math.signum(error) * MIN_POWER;
            }
        } else {
            power = Range.clip(gamepad1.left_stick_x, -1, 1) * Math.abs(MANUAL_MAX_POWER);
        }
        motor.setPower(POWER_SIGN * power);

        /* ---------- PANELS ---------- */
        boolean magnet = limit != null && limit.isPressed();
        panels.addData("Target", TARGET_ANGLE);
        panels.addData("Actual", now);
        panels.addData("Error", error);
        panels.addData("Power", power);
        panels.addData("Magnet", magnet ? 1 : 0);

        panels.addData("Mode", hold ? "HOLD" : "MANUAL");
        panels.addData("Ticks", motor.getCurrentPosition() + offsetTicks);
        panels.addData("Velocity (deg/s)", velocity);
        panels.addData("Limit switch", limit == null ? "not in config" : (magnet ? "MAGNET" : "clear"));
        panels.addData("Gains", String.format("KP=%.4f KD=%.5f min=%.2f max=%.2f deadband=%.1f sign=%.0f",
                KP, KD, MIN_POWER, MAX_POWER, DEADBAND_DEG, POWER_SIGN));
        panels.update(telemetry);
    }

    @Override
    public void stop() {
        motor.setPower(0);
    }

    private double angle() {
        return TURRET_START_ANGLE_DEG + (motor.getCurrentPosition() + offsetTicks) / TURRET_TICKS_PER_DEGREE;
    }

    private static double wrap180(double a) {
        a = a % 360.0;
        if (a > 180) a -= 360;
        if (a <= -180) a += 360;
        return a;
    }
}
