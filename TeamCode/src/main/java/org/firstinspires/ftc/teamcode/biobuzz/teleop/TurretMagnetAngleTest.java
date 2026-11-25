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

import java.util.ArrayList;
import java.util.List;

/**
 * Simplest magnet test: shows the turret's live angle, and every time the magnet switch triggers it writes down the
 * angle it happened at. Nothing is corrected, nothing tracks a goal, no drivetrain.
 *
 * HOW TO USE
 *   1. Point the turret straight forward, THEN press init (that spot = START_ANGLE_DEG = 0 deg). Press start.
 *   2. Turn the turret by hand (power is off unless you use the stick), or use gamepad1 left stick X.
 *   3. Each time the magnet reaches the switch, a line shows up in the list:
 *          "ENTER  CW  at 33.1 deg"   = the magnet just started being detected, turning clockwise, at that angle
 *          "EXIT   CW  at 29.8 deg"   = the magnet stopped being detected
 *      The band between ENTER and EXIT is how wide the magnet reads. The clockwise ENTER angle is what goes into
 *      TURRET_LIMIT_ANGLE_DEG. Do 5 slow clockwise passes: "AVG clockwise ENTER" is the number to use.
 *   4. If the angle in the list changes a lot from pass to pass, the belt is slipping (or the encoder moved).
 *
 * Panels (http://192.168.43.1:8001): graph keys "Angle" and "Magnet" (1 = detected).
 * Configurables -> "TurretMagnetAngleTest": MANUAL_MAX_POWER, POWER_SIGN.
 *
 * gamepad1: left stick X = turn the turret (CCW = right), triangle = declare the current position to be START_ANGLE_DEG (0),
 *           square = clear the list
 */
@Configurable
@TeleOp(name = "Turret Magnet Angle Test", group = "BioBuzz Tuning")
public class TurretMagnetAngleTest extends OpMode {

    /** Max power from the stick. Keep it low so passes are slow and exact. */
    public static double MANUAL_MAX_POWER = 0.2;
    /** Sign on motor power so stick right = angle up (-1 on this robot). */
    public static double POWER_SIGN = TURRET_POWER_SIGN;
    /**
     * The angle the turret reads at init / after triangle. 0 here. BioBuzzConfig uses 90 (= TURRET_START_ANGLE_DEG),
     * so the telemetry also shows every angle converted to that frame (the one to paste into TURRET_LIMIT_ANGLE_DEG).
     */
    public static double START_ANGLE_DEG = 0;

    private static final int MAX_EVENTS = 14;

    private DcMotorEx motor;
    private TouchSensor limit;
    private TelemetryManager panels;

    private int offsetTicks = 0;
    private boolean prevPressed;
    private int prevTicks;
    private int lastDir = 0; // +1 = CCW (angle up), -1 = CW

    private final List<String> events = new ArrayList<>();
    private double cwEnterSum = 0;
    private int cwEnterCount = 0;
    private double lastEnterAngle = Double.NaN;
    private double lastExitAngle = Double.NaN;
    private boolean lTriangle, lSquare;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, TURRET);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // easy to turn by hand
        motor.setDirection(TURRET_DIR);
        try {
            limit = hardwareMap.get(TouchSensor.class, TURRET_LIMIT);
        } catch (RuntimeException e) {
            limit = null;
        }
        prevTicks = motor.getCurrentPosition();
        prevPressed = pressed();
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        telemetry.addData("Status", limit == null
                ? "NO limit switch named \"" + TURRET_LIMIT + "\" in the config!"
                : "Ready. Turret should be pointing straight forward now (= START_ANGLE_DEG).");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.triangle && !lTriangle) {
            offsetTicks = -motor.getCurrentPosition();
        }
        if (gamepad1.square && !lSquare) {
            events.clear();
            cwEnterSum = 0;
            cwEnterCount = 0;
        }
        lTriangle = gamepad1.triangle;
        lSquare = gamepad1.square;

        motor.setPower(POWER_SIGN * Range.clip(gamepad1.left_stick_x, -1, 1) * Math.abs(MANUAL_MAX_POWER));

        int ticks = motor.getCurrentPosition();
        double angle = angleAt(ticks);
        boolean now = pressed();
        int step = ticks - prevTicks;
        if (step != 0) lastDir = Integer.signum(step);

        // Magnet just started / stopped being detected: write down where we are.
        if (now != prevPressed) {
            String dir = lastDir < 0 ? "CW" : (lastDir > 0 ? "CCW" : "?");
            if (now) {
                lastEnterAngle = angle;
                if (lastDir < 0) {
                    // Wrap to one turn (in BioBuzzConfig's frame) so passes on different turns average correctly.
                    cwEnterSum += wrap180(angle + TURRET_START_ANGLE_DEG - START_ANGLE_DEG);
                    cwEnterCount++;
                }
            } else {
                lastExitAngle = angle;
            }
            events.add(String.format("%s  %-3s  at %.2f deg", now ? "ENTER" : "EXIT ", dir, angle));
            if (events.size() > MAX_EVENTS) events.remove(0);
        }
        prevPressed = now;
        prevTicks = ticks;

        telemetry.addData("ANGLE (deg)", "%.2f", angle);
        telemetry.addData("MAGNET", limit == null ? "no switch in config" : (now ? "DETECTED" : "clear"));
        telemetry.addData("Last ENTER / EXIT angle", "%s / %s", fmt(lastEnterAngle), fmt(lastExitAngle));
        double toConfig = TURRET_START_ANGLE_DEG - START_ANGLE_DEG; // add to get BioBuzzConfig's frame (front = 90)
        // The angle keeps counting past +-360 (no hard stops), so the average is wrapped to one turn.
        telemetry.addData("AVG clockwise ENTER (this test's frame, wrapped)",
                cwEnterCount == 0 ? "-" : String.format("%.2f  (%d passes)", cwEnterSum / cwEnterCount - toConfig, cwEnterCount));
        telemetry.addData("  = in BioBuzzConfig frame -> TURRET_LIMIT_ANGLE_DEG",
                cwEnterCount == 0 ? "-" : String.format("%.2f", cwEnterSum / cwEnterCount));
        telemetry.addData("Events (newest last; square = clear)", events.isEmpty() ? "-" : "");
        for (int i = 0; i < events.size(); i++) telemetry.addData(" " + (i + 1), events.get(i));
        telemetry.update();

        panels.addData("Angle", angle);
        panels.addData("Angle (BioBuzzConfig frame)", angle + toConfig);
        panels.addData("Magnet", now ? 1 : 0);
        panels.update();
    }

    @Override
    public void stop() {
        motor.setPower(0);
    }

    private boolean pressed() {
        return limit != null && limit.isPressed();
    }

    private double angleAt(int motorTicks) {
        return START_ANGLE_DEG + (motorTicks + offsetTicks) / TURRET_TICKS_PER_DEGREE;
    }

    /** Same angle, moved into -180..180 by whole turns. */
    private static double wrap180(double deg) {
        deg %= 360.0;
        if (deg > 180.0) deg -= 360.0;
        if (deg <= -180.0) deg += 360.0;
        return deg;
    }

    private static String fmt(double v) {
        return Double.isNaN(v) ? "-" : String.format("%.2f", v);
    }
}
