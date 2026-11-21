package org.firstinspires.ftc.teamcode.biobuzz.teleop;

import static org.firstinspires.ftc.teamcode.biobuzz.teleop.BioBuzzConfig.*;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * LIVE-TUNABLE magnet limit switch + goal tracking for the turret. Use {@link TurretLimitPanelsBlue} or
 * {@link TurretLimitPanelsRed}.
 *
 * WHAT THE MAGNET DOES
 *   The belt / gears skip, so the encoder angle slowly drifts. A magnet on the turret passes a REV magnetic switch
 *   (a "REV Touch Sensor" named TURRET_LIMIT in the config). While the magnet is over the switch the sensor reads
 *   "detected" for a band a few degrees wide. We only trust ONE edge of that band: the edge the turret hits when it
 *   is spinning CLOCKWISE. The moment it crosses that edge we know EXACTLY where the turret is, so we overwrite the
 *   encoder angle with LIMIT_ANGLE_DEG ("re-home"), and the goal tracking just keeps going from the corrected angle.
 *   Counter-clockwise entries are ignored.
 *
 * WHAT THIS OPMODE DOES
 *   Drives the robot (Pedro), tracks the goal with the turret (same angle math as BioBuzzTurret, same goal-selection
 *   rule as the teleop), and re-homes on every clockwise pass over the magnet. EVERYTHING below can be changed live
 *   in Panels. This class is separate from BioBuzzTurret, so when you are happy, COPY the numbers into BioBuzzConfig
 *   (TURRET_LIMIT_ANGLE_DEG, TURRET_REHOME_*, TURRET_HOME_*, TURRET_KP ...).
 *
 * HOW TO TUNE (do these in order)
 *   0. Config: a "REV Touch Sensor" named "turretLimit". Point the turret straight forward, THEN press init.
 *   1. Does it see the magnet? Press cross until Mode = MANUAL, turn the turret by hand (or left stick X on gamepad2).
 *      "Magnet" on the graph should go 0 -> 1 -> 0 as the magnet passes. If not: wiring / config name / magnet gap.
 *   2. Find LIMIT_ANGLE_DEG. Set REHOME_ON_PASS = false (so nothing gets corrected). Pass the magnet CLOCKWISE slowly
 *      (stick or by hand) 5+ times. Each pass shows "Last edge angle"; "Avg edge angle" is the mean. Type that average
 *      into LIMIT_ANGLE_DEG. (options clears the averages. This only works if the encoder hasn't skipped, so start
 *      from a fresh init with the turret straight forward. TurretLimitFinder does the same thing with sweeps.)
 *   3. Prove the re-home. Turn REHOME_ON_PASS back on. Mess the angle up on purpose: turn the turret by hand with
 *      power off, then press triangle (declares "straight forward = 90" wrong on purpose). Now drive the turret
 *      CLOCKWISE through the magnet: "Angle" JUMPS by "Last correction" and "Re-homes" goes up by 1. After that the
 *      angle should match reality again (turret points at the goal).
 *   4. Seek test: with the angle wrong, press square. The turret spins clockwise at SEEK_POWER until it finds the
 *      magnet, re-homes, and then goes back to TRACK on its own. FAILED means no magnet within MAX_SEEK_DEG.
 *   5. Speed: if a re-home is skipped while tracking, the turret was moving more than REHOME_MAX_STEP_DEG per loop
 *      over the magnet. Raise it (less exact, error up to about half of it) or aim slower.
 *   6. Gains: KP / KD / MIN_POWER / MAX_POWER / DEADBAND_DEG are the tracking PD. POWER_SIGN must be -1 on this robot.
 *
 * Panels (http://192.168.43.1:8001): Configurables -> "TurretLimitPanelsBase". Graph keys:
 *   Graph 1 (deg): "Target", "Actual"   Graph 2: "Error", "Power"   Graph 3: "Magnet" (1 = detected)
 *
 * gamepad1: left stick = drive/strafe, right stick = turn, triangle = reset pose
 * gamepad2: cross = toggle TRACK goal / MANUAL (left stick X = turret power), square = SEEK the magnet (clockwise,
 *           then back to TRACK), circle = stop + MANUAL, triangle = declare the turret's position straight forward,
 *           dpad left / right = aim trim, options = clear the edge averages
 */
@Configurable
public abstract class TurretLimitPanelsBase extends OpMode {

    // ================= MAGNET / RE-HOME (editable in Panels) =================
    /** Turret angle (deg) at the CLOCKWISE edge of the magnet band. Find it with step 2 above. NaN turns re-homing off. */
    public static double LIMIT_ANGLE_DEG = TURRET_LIMIT_ANGLE_DEG;
    /** true = re-home on every clockwise pass while tracking; false = only while seeking (and just measure edges). */
    public static boolean REHOME_ON_PASS = true;
    /** Ignore a pass if the turret moved more than this many degrees in one loop (fast passes are less exact). */
    public static double REHOME_MAX_STEP_DEG = TURRET_REHOME_MAX_STEP_DEG;
    /** true = only clockwise entries count (the only direction the angle is calibrated for). Leave this on. */
    public static boolean CLOCKWISE_ONLY = true;

    // ================= SEEK (square button) =================
    /** Power while spinning clockwise to find the magnet. Keep it slow so the edge is exact. */
    public static double SEEK_POWER = TURRET_HOME_POWER;
    /** If the turret starts ON the magnet it backs off counter-clockwise at this power first. */
    public static double BACKOFF_POWER = TURRET_HOME_BACKOFF_POWER;
    /** ...until it is this many degrees clear of the magnet. */
    public static double BACKOFF_DEG = TURRET_HOME_BACKOFF_DEG;
    /** Give up (FAILED) after this many degrees clockwise without finding the magnet. */
    public static double MAX_SEEK_DEG = TURRET_HOME_MAX_SEEK_DEG;
    /** Give up if still on the magnet after this many degrees of backing off. */
    public static double MAX_BACKOFF_DEG = TURRET_HOME_MAX_BACKOFF_DEG;

    // ================= TRACKING PD =================
    public static double KP = TURRET_KP;
    public static double KD = TURRET_KD;
    public static double MIN_POWER = TURRET_MIN_POWER;
    public static double MAX_POWER = TURRET_MAX_POWER;
    public static double DEADBAND_DEG = TURRET_DEADBAND_DEG;
    /** Sign on every motor power so positive power raises the angle (-1 on this robot). */
    public static double POWER_SIGN = TURRET_POWER_SIGN;
    /** MANUAL mode: max power from gamepad2 left stick X. */
    public static double MANUAL_MAX_POWER = 0.3;
    // ========================================================================

    private enum HomeState { IDLE, BACKING_OFF, SEEKING, DONE, FAILED }

    private final BioBuzzAlliance alliance;

    private Follower follower;
    private DcMotorEx motor;
    private TouchSensor limit;
    private TelemetryManager panels;

    // angle / encoder
    private int offsetTicks = 0;
    private double prevAngle;
    private long lastLoopNanos;
    private double trimDeg;

    // limit-switch edge detection (motor ticks WITHOUT the offset, so a re-home doesn't disturb it)
    private int prevMotorTicks;
    private boolean prevPressed;
    private int clearSamples = 0;
    private int lastMoveDir = 0;      // +1 = CCW (angle up), -1 = CW (angle down), sticky
    private boolean edgePending = false;
    private double pendingEdgeTicks;

    // stats
    private int rehomeCount = 0;
    private double lastCorrectionDeg = 0;
    private double lastEdgeAngle = Double.NaN;
    private int edgeCount = 0;
    private double edgeSum = 0;

    // modes
    private boolean tracking = true;
    private HomeState homeState = HomeState.IDLE;
    private String homeMessage = "";
    private double homeLegStartAngle;
    private double backoffClearedAngle = Double.NaN;

    // goal tracking
    private Pose lastPose = null;
    private boolean firstLoop = true;
    private boolean lCross, lSquare, lTriangle, lOptions;

    protected TurretLimitPanelsBase(BioBuzzAlliance alliance) {
        this.alliance = alliance;
    }

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(alliance.startPose);

        motor = hardwareMap.get(DcMotorEx.class, TURRET);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(TURRET_DIR);
        try {
            limit = hardwareMap.get(TouchSensor.class, TURRET_LIMIT);
        } catch (RuntimeException e) {
            limit = null;
        }
        trimDeg = alliance.startTrimDeg;
        prevMotorTicks = motor.getCurrentPosition();
        prevPressed = magnet();
        prevAngle = angle();
        lastLoopNanos = System.nanoTime();

        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        telemetry.addData("Status", "Ready (" + alliance + "). Turret straight forward at init. " +
                (limit == null ? "NO limit switch \"" + TURRET_LIMIT + "\" in config!" : "limit switch found"));
        telemetry.update();
    }

    @Override
    public void start() {
        prevAngle = angle();
        lastLoopNanos = System.nanoTime();
    }

    @Override
    public void loop() {
        follower.update();
        if (firstLoop) {
            firstLoop = false;
            return;
        }
        if (gamepad1.triangle) follower.setPose(alliance.resetPose);
        follower.manual(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        Pose pose = follower.pose();

        /* ---------- GAMEPAD 2 ---------- */
        boolean cross = gamepad2.cross, square = gamepad2.square;
        boolean triangle = gamepad2.triangle, options = gamepad2.options;
        if (cross && !lCross) {
            cancelHoming();
            tracking = !tracking;
        }
        if (square && !lSquare) startSeek();
        if (gamepad2.circle) {
            cancelHoming();
            tracking = false;
        }
        if (triangle && !lTriangle) {
            cancelHoming();
            offsetTicks = -motor.getCurrentPosition();
            prevAngle = angle();
        }
        if (options && !lOptions) {
            edgeCount = 0;
            edgeSum = 0;
            lastEdgeAngle = Double.NaN;
            rehomeCount = 0;
            lastCorrectionDeg = 0;
        }
        lCross = cross; lSquare = square; lTriangle = triangle; lOptions = options;
        double trimRate = (gamepad2.dpad_left ? TURRET_TRIM_RATE_DEG_PER_SEC : 0)
                - (gamepad2.dpad_right ? TURRET_TRIM_RATE_DEG_PER_SEC : 0);
        long nanos = System.nanoTime();
        double dt = Math.max((nanos - lastLoopNanos) / 1e9, 1e-3);
        lastLoopNanos = nanos;
        trimDeg += trimRate * dt;

        /* ---------- MAGNET (every loop, no matter the mode) ---------- */
        boolean rehomedNow = senseLimit();

        /* ---------- CONTROL ---------- */
        double now = angle();
        double velocity = (now - prevAngle) / dt;
        prevAngle = now;

        boolean secondary = alliance.useSecondaryGoalAt(pose.y());
        double target = targetAngle(pose, secondary);
        double error = wrap180(target - now);

        double power = 0;
        if (isHoming()) {
            power = runHoming(rehomedNow, now);
        } else if (tracking) {
            if (Math.abs(error) > DEADBAND_DEG) {
                power = KP * error - KD * velocity;
                power = Range.clip(power, -MAX_POWER, MAX_POWER);
                if (Math.abs(power) < MIN_POWER) power = Math.signum(error) * MIN_POWER;
            }
        } else {
            power = Range.clip(gamepad2.left_stick_x, -1, 1) * Math.abs(MANUAL_MAX_POWER);
        }
        motor.setPower(POWER_SIGN * Range.clip(power, -1, 1));

        /* ---------- TELEMETRY ---------- */
        boolean magnet = magnet();
        panels.addData("Target", target);
        panels.addData("Actual", now);
        panels.addData("Error", error);
        panels.addData("Power", power);
        panels.addData("Magnet", magnet ? 1 : 0);

        String mode = isHoming() ? "SEEKING MAGNET (" + homeState + ")" : (tracking ? "TRACK goal" : "MANUAL");
        panels.addData("Mode", mode);
        panels.addData("Home state", homeState + (homeMessage.isEmpty() ? "" : ": " + homeMessage));
        panels.addData("Limit switch", limit == null ? "NOT IN CONFIG" : (magnet ? "MAGNET" : "clear"));
        panels.addData("Re-homes / last correction (deg)", String.format("%d / %.2f", rehomeCount, lastCorrectionDeg));
        panels.addData("Last edge angle (before re-home)", Double.isNaN(lastEdgeAngle) ? "-" : String.format("%.2f", lastEdgeAngle));
        panels.addData("AVG EDGE ANGLE -> LIMIT_ANGLE_DEG", edgeCount == 0 ? "-" : String.format("%.2f  (%d passes)", edgeSum / edgeCount, edgeCount));
        panels.addData("Goal", (secondary ? "SECONDARY" : "PRIMARY") + String.format(" (robot y %.1f)", pose.y()));
        panels.addData("Ticks / trim", String.format("%d / %.1f", motor.getCurrentPosition() + offsetTicks, trimDeg));
        panels.update(telemetry);
    }

    @Override
    public void stop() {
        motor.setPower(0);
    }

    // ================= MAGNET =================
    private boolean magnet() {
        return limit != null && limit.isPressed();
    }

    private boolean canHome() {
        return limit != null && !Double.isNaN(LIMIT_ANGLE_DEG);
    }

    /**
     * Reads the encoder and the switch once. A clockwise entry into the magnet is an "edge": it is recorded (so you
     * can average it for LIMIT_ANGLE_DEG) and, if re-homing is on or we are seeking, the angle is corrected. The edge
     * is placed halfway between this read and the last, and must still read "detected" on the NEXT loop (so one noisy
     * read can't re-home). Returns true on the loop the angle was corrected.
     */
    private boolean senseLimit() {
        int ticks = motor.getCurrentPosition();
        boolean pressed = magnet();
        int step = ticks - prevMotorTicks;
        if (step != 0) lastMoveDir = Integer.signum(step);

        boolean seeking = homeState == HomeState.SEEKING;
        boolean rehomed = false;

        if (edgePending) {
            edgePending = false;
            if (pressed) {
                double edgeAngle = TURRET_START_ANGLE_DEG + (pendingEdgeTicks + offsetTicks) / TURRET_TICKS_PER_DEGREE;
                lastEdgeAngle = edgeAngle;
                edgeCount++;
                edgeSum += edgeAngle;
                if (canHome() && (REHOME_ON_PASS || seeking)) {
                    rehome(pendingEdgeTicks);
                    rehomed = true;
                }
            }
        }

        boolean risingEdge = pressed && !prevPressed && clearSamples >= 2;
        boolean clockwise = !CLOCKWISE_ONLY || lastMoveDir < 0;
        boolean slowEnough = seeking || Math.abs(step) / TURRET_TICKS_PER_DEGREE <= REHOME_MAX_STEP_DEG;
        if (risingEdge && clockwise && slowEnough) {
            edgePending = true;
            pendingEdgeTicks = (prevMotorTicks + ticks) / 2.0;
        }

        clearSamples = pressed ? 0 : clearSamples + 1;
        prevPressed = pressed;
        prevMotorTicks = ticks;
        return rehomed;
    }

    /** Make the given motor-tick position read as LIMIT_ANGLE_DEG (keeping the same number of full turns). */
    private void rehome(double edgeTicks) {
        double before = TURRET_START_ANGLE_DEG + (edgeTicks + offsetTicks) / TURRET_TICKS_PER_DEGREE;
        double edgeAngle = LIMIT_ANGLE_DEG + 360.0 * Math.round((before - LIMIT_ANGLE_DEG) / 360.0);
        offsetTicks = (int) Math.round((edgeAngle - TURRET_START_ANGLE_DEG) * TURRET_TICKS_PER_DEGREE - edgeTicks);
        double after = TURRET_START_ANGLE_DEG + (edgeTicks + offsetTicks) / TURRET_TICKS_PER_DEGREE;
        lastCorrectionDeg = after - before;
        rehomeCount++;
        prevAngle += lastCorrectionDeg; // keep the D term's velocity continuous
    }

    // ================= SEEK =================
    private boolean isHoming() {
        return homeState == HomeState.BACKING_OFF || homeState == HomeState.SEEKING;
    }

    private void startSeek() {
        if (!canHome()) {
            homeState = HomeState.FAILED;
            homeMessage = limit == null ? "no limit switch \"" + TURRET_LIMIT + "\" in config" : "LIMIT_ANGLE_DEG is NaN";
            return;
        }
        homeLegStartAngle = angle();
        if (magnet()) {
            homeState = HomeState.BACKING_OFF;
            backoffClearedAngle = Double.NaN;
            homeMessage = "on the magnet: backing off CCW";
        } else {
            homeState = HomeState.SEEKING;
            homeMessage = "spinning CW";
        }
    }

    private void cancelHoming() {
        if (isHoming()) {
            homeState = HomeState.IDLE;
            homeMessage = "cancelled";
        }
    }

    /** One step of seeking. Returns the angle-frame power (+ = CCW). Goes back to TRACK when it re-homes. */
    private double runHoming(boolean rehomedThisLoop, double angle) {
        if (homeState == HomeState.SEEKING) {
            if (rehomedThisLoop) {
                homeState = HomeState.DONE;
                homeMessage = String.format("homed (moved %.1f deg)", lastCorrectionDeg);
                tracking = true; // resume goal tracking
                return 0;
            }
            if (homeLegStartAngle - angle > MAX_SEEK_DEG) {
                homeState = HomeState.FAILED;
                homeMessage = String.format("no magnet after %.0f deg clockwise", MAX_SEEK_DEG);
                return 0;
            }
            return -Math.abs(SEEK_POWER); // CW = angle down
        }
        // BACKING_OFF (counter-clockwise, off the magnet)
        if (magnet()) {
            backoffClearedAngle = Double.NaN;
            if (angle - homeLegStartAngle > MAX_BACKOFF_DEG) {
                homeState = HomeState.FAILED;
                homeMessage = String.format("still on the magnet after %.0f deg CCW", MAX_BACKOFF_DEG);
                return 0;
            }
        } else {
            if (Double.isNaN(backoffClearedAngle)) backoffClearedAngle = angle;
            if (angle - backoffClearedAngle >= BACKOFF_DEG) {
                homeState = HomeState.SEEKING;
                homeLegStartAngle = angle;
                homeMessage = "spinning CW";
                return -Math.abs(SEEK_POWER);
            }
        }
        return Math.abs(BACKOFF_POWER); // CCW = angle up
    }

    // ================= ANGLE / GOAL =================
    private double angle() {
        return TURRET_START_ANGLE_DEG + (motor.getCurrentPosition() + offsetTicks) / TURRET_TICKS_PER_DEGREE;
    }

    /** Desired turret angle (0 = robot right, 90 = front, CCW positive), plus trim. Not wrapped. */
    private double targetAngle(Pose robot, boolean secondary) {
        double gx = secondary ? alliance.secondaryGoalX : alliance.primaryGoalX;
        double gy = secondary ? alliance.secondaryGoalY : alliance.primaryGoalY;
        double h = robot.heading();
        double tx = robot.x() + TURRET_OFFSET_FORWARD * Math.cos(h) - TURRET_OFFSET_LEFT * Math.sin(h);
        double ty = robot.y() + TURRET_OFFSET_FORWARD * Math.sin(h) + TURRET_OFFSET_LEFT * Math.cos(h);
        double fieldAngle = Math.toDegrees(Math.atan2(gy - ty, gx - tx));
        double frontRelative = wrap180(fieldAngle - Math.toDegrees(h)); // 0 = front
        return frontRelative + TURRET_FRONT_ANGLE_DEG + trimDeg;
    }

    private static double wrap180(double a) {
        a = a % 360.0;
        if (a > 180) a -= 360;
        if (a <= -180) a += 360;
        return a;
    }
}
