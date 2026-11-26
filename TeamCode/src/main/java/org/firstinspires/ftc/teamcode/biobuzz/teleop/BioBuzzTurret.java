package org.firstinspires.ftc.teamcode.biobuzz.teleop;

import static org.firstinspires.ftc.teamcode.biobuzz.teleop.BioBuzzConfig.*;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * Field-aware turret for the BioBuzz robot.
 *
 * Unlike the Decode turret (0..180 deg, clamped) this one is continuous: there are no angle limits
 * and no wrap-around handling. The angle is a running total from the encoder, and every update the
 * turret takes the shortest way to the target (so the error is always within +-180 deg).
 * Make sure the wiring is on a slip ring or otherwise able to spin freely.
 *
 * Turret angle convention: 0 = robot RIGHT, 90 = robot FRONT, 180 = left, 270 = back, positive = counter-clockwise.
 *
 * HOMING: the belt/gears skip, so  encoder angle drifts. A magnet on the turret passes a REV magnetic switch
 * (configured as a REV Touch Sensor). The switch reads over a ~4 deg band, so only its LEFT edge is used: the edge
 * the turret hits while spinning CLOCKWISE. Every clockwise entry re-homes the angle to TURRET_LIMIT_ANGLE_DEG
 * (counter-clockwise entries are ignored). startHoming() makes it go find the magnet, always finishing clockwise.
 *
 * LIMIT SWITCH DISABLED (commented out): the angle comes from the encoder only, no re-homing.
 */
public class BioBuzzTurret {

    private final DcMotorEx motor;
    private final BioBuzzAlliance alliance;

    private int encoderOffsetTicks = 0;
    private double trimDeg;
    private double targetAngleDeg = TURRET_START_ANGLE_DEG;

    private double goalX, goalY;
    private boolean targetingSecondaryGoal = false;

    private double previousAngleDeg;
    private long lastNanos;

    // LIMIT SWITCH DISABLED: everything for the magnet switch / re-homing is commented out below. Uncomment to bring it back.
//    // ================= LIMIT SWITCH / HOMING STATE =================
//    /** Magnet limit switch (REV Touch Sensor). Null if it isn't in the robot config. */
//    private final TouchSensor limit;

//    // Edge detection (motor ticks, without the offset, so re-homing doesn't disturb it)
//    private int prevMotorTicks;
//    private boolean prevPressed;
//    private int clearSamples = 0;     // consecutive "not detected" reads before the current one
//    private int lastMoveDir = 0;      // +1 = CCW (angle up), -1 = CW (angle down), sticky across still loops
//    private boolean edgePending = false;
//    private double pendingEdgeMotorTicks;

//    private int rehomeCount = 0;
//    private double lastRehomeCorrectionDeg = 0;

//    public enum HomeState { IDLE, BACKING_OFF, SEEKING, DONE, FAILED }

//    private HomeState homeState = HomeState.IDLE;
//    private String homeMessage = "not homed";
//    private double homeLegStartAngle;
//    private double backoffClearedAngle = Double.NaN;

    public BioBuzzTurret(HardwareMap hardwareMap, BioBuzzAlliance alliance) {
        this.alliance = alliance;
        this.trimDeg = alliance.startTrimDeg;
        this.goalX = alliance.primaryGoalX;
        this.goalY = alliance.primaryGoalY;

        motor = hardwareMap.get(DcMotorEx.class, TURRET);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(TURRET_DIR);

//        TouchSensor found;
//        try {
//            found = hardwareMap.get(TouchSensor.class, TURRET_LIMIT);
//        } catch (RuntimeException e) {
//            found = null; // not in this config: no homing, everything else still works
//        }
//        limit = found;

//        prevMotorTicks = motor.getCurrentPosition();
//        prevPressed = isLimitPressed();
        previousAngleDeg = getAngle();
        lastNanos = System.nanoTime();
    }

    // ================= GOAL TOGGLING =================
    public void toggleGoal() {
        targetingSecondaryGoal = !targetingSecondaryGoal;
        goalX = targetingSecondaryGoal ? alliance.secondaryGoalX : alliance.primaryGoalX;
        goalY = targetingSecondaryGoal ? alliance.secondaryGoalY : alliance.primaryGoalY;
    }

    /** Pick the goal directly (used by the teleop's automatic goal choice). */
    public void setSecondaryGoal(boolean secondary) {
        if (secondary != targetingSecondaryGoal) toggleGoal();
    }

    public boolean isTargetingSecondaryGoal() {
        return targetingSecondaryGoal;
    }

    // ================= STATE =================
    /** Continuous turret angle in degrees (can go past +-360). */
    public double getAngle() {
        return TURRET_START_ANGLE_DEG + getRawTicks() / TURRET_TICKS_PER_DEGREE;
    }

    /** Encoder ticks relative to the start position (includes any offset carried over from auto). */
    public int getRawTicks() {
        return motor.getCurrentPosition() + encoderOffsetTicks;
    }

    /** Carry the turret position over from auto: pass the ticks auto saved with BioBuzzPose. */
    public void setEncoderOffset(int ticks) {
        encoderOffsetTicks = ticks;
        previousAngleDeg = getAngle();
    }

    /** Declares the turret's current physical position to be {@code TURRET_START_ANGLE_DEG}. */
    public void resetEncoder() {
//        cancelHoming();
        encoderOffsetTicks = -motor.getCurrentPosition();
        targetAngleDeg = TURRET_START_ANGLE_DEG;
        previousAngleDeg = getAngle();
        motor.setPower(0);
    }

//    // ================= LIMIT SWITCH / HOMING =================
//    public boolean hasLimitSwitch() {
//        return limit != null;
//    }

//    /** True while the magnet is detected. A REV Touch Sensor's isPressed() already means "line LOW" = magnet there. */
//    public boolean isLimitPressed() {
//        return limit != null && limit.isPressed();
//    }

//    /** Homing / re-homing needs the switch and a measured TURRET_LIMIT_ANGLE_DEG. */
//    public boolean canHome() {
//        return limit != null && !Double.isNaN(TURRET_LIMIT_ANGLE_DEG);
//    }

//    /**
//     * Start homing: if already on the magnet, back off counter-clockwise until clear by TURRET_HOME_BACKOFF_DEG, then
//     * spin CLOCKWISE until it enters the magnet, which re-homes on the band's left edge. While homing, aimAtGoal
//     * drives the homing instead of aiming, so callers just keep calling aimAtGoal every loop.
//     */
//    public void startHoming() {
//        if (!canHome()) {
//            homeState = HomeState.FAILED;
//            homeMessage = limit == null ? "no limit switch \"" + TURRET_LIMIT + "\" in config" : "TURRET_LIMIT_ANGLE_DEG not set";
//            return;
//        }
//        homeLegStartAngle = getAngle();
//        if (isLimitPressed()) {
//            homeState = HomeState.BACKING_OFF;
//            backoffClearedAngle = Double.NaN;
//            homeMessage = "on the magnet: backing off CCW";
//        } else {
//            homeState = HomeState.SEEKING;
//            homeMessage = "seeking CW";
//        }
//    }

//    public boolean isHoming() {
//        return homeState == HomeState.BACKING_OFF || homeState == HomeState.SEEKING;
//    }

//    public void cancelHoming() {
//        if (isHoming()) {
//            homeState = HomeState.IDLE;
//            homeMessage = "cancelled";
//            motor.setPower(0);
//        }
//    }

//    public HomeState getHomeState() {
//        return homeState;
//    }

//    public String getHomeMessage() {
//        return homeMessage;
//    }

//    /** How many times the angle has been re-homed on the magnet. */
//    public int getRehomeCount() {
//        return rehomeCount;
//    }

//    /** How far the last re-home moved the angle (deg). This is roughly how much the belt/gears skipped since the one before. */
//    public double getLastRehomeCorrection() {
//        return lastRehomeCorrectionDeg;
//    }

//    /**
//     * Read the encoder and the switch once each, every loop. Re-homes when the turret enters the magnet while moving
//     * CLOCKWISE (that's the band's left edge). The edge is placed halfway between this read and the last one, and must
//     * still read "detected" on the next loop (so one noisy read can't re-home it). Returns true if it re-homed.
//     */
//    private boolean senseLimit() {
//        int ticks = motor.getCurrentPosition();
//        boolean pressed = isLimitPressed();
//        int step = ticks - prevMotorTicks;
//        if (step != 0) lastMoveDir = Integer.signum(step);

//        boolean rehomed = false;
//        boolean seeking = homeState == HomeState.SEEKING;
//        boolean enabled = canHome() && (TURRET_REHOME_ON_PASS || seeking);

//        if (edgePending) {
//            edgePending = false;
//            if (pressed && enabled) {
//                rehome(pendingEdgeMotorTicks);
//                rehomed = true;
//            }
//        }

//        boolean risingEdge = pressed && !prevPressed && clearSamples >= 2;
//        boolean clockwise = lastMoveDir < 0;
//        boolean slowEnough = seeking || Math.abs(step) / TURRET_TICKS_PER_DEGREE <= TURRET_REHOME_MAX_STEP_DEG;
//        if (enabled && risingEdge && clockwise && slowEnough) {
//            edgePending = true;
//            pendingEdgeMotorTicks = (prevMotorTicks + ticks) / 2.0;
//        }

//        clearSamples = pressed ? 0 : clearSamples + 1;
//        prevPressed = pressed;
//        prevMotorTicks = ticks;
//        return rehomed;
//    }

//    /** Shift the encoder offset so the given motor-tick position reads as the left edge (nearest equivalent angle). */
//    private void rehome(double edgeMotorTicks) {
//        double before = TURRET_START_ANGLE_DEG + (edgeMotorTicks + encoderOffsetTicks) / TURRET_TICKS_PER_DEGREE;
//        // Keep the same number of full turns, so an infinite turret's angle doesn't jump by 360.
//        double edgeAngle = TURRET_LIMIT_ANGLE_DEG + 360.0 * Math.round((before - TURRET_LIMIT_ANGLE_DEG) / 360.0);
//        encoderOffsetTicks = (int) Math.round((edgeAngle - TURRET_START_ANGLE_DEG) * TURRET_TICKS_PER_DEGREE - edgeMotorTicks);
//        double after = TURRET_START_ANGLE_DEG + (edgeMotorTicks + encoderOffsetTicks) / TURRET_TICKS_PER_DEGREE;

//        lastRehomeCorrectionDeg = after - before;
//        rehomeCount++;
//        previousAngleDeg += lastRehomeCorrectionDeg; // keep the D term's velocity continuous
//        targetAngleDeg += lastRehomeCorrectionDeg;
//    }

//    /** One step of start / button homing. */
//    private void runHoming(boolean rehomedThisLoop) {
//        double angle = getAngle();
//        if (homeState == HomeState.SEEKING) {
//            if (rehomedThisLoop) {
//                motor.setPower(0);
//                homeState = HomeState.DONE;
//                homeMessage = String.format("homed (moved %.1f deg)", lastRehomeCorrectionDeg);
//                targetAngleDeg = getAngle();
//                previousAngleDeg = getAngle();
//                lastNanos = System.nanoTime();
//                return;
//            }
//            if (homeLegStartAngle - angle > TURRET_HOME_MAX_SEEK_DEG) {
//                failHoming(String.format("no magnet after %.0f deg clockwise", TURRET_HOME_MAX_SEEK_DEG));
//                return;
//            }
//            setAnglePower(-TURRET_HOME_POWER); // CW = angle down
//            return;
//        }

//        // BACKING_OFF (counter-clockwise, off the magnet)
//        if (isLimitPressed()) {
//            backoffClearedAngle = Double.NaN;
//            if (angle - homeLegStartAngle > TURRET_HOME_MAX_BACKOFF_DEG) {
//                failHoming(String.format("still on the magnet after %.0f deg CCW", TURRET_HOME_MAX_BACKOFF_DEG));
//                return;
//            }
//        } else {
//            if (Double.isNaN(backoffClearedAngle)) backoffClearedAngle = angle;
//            if (angle - backoffClearedAngle >= TURRET_HOME_BACKOFF_DEG) {
//                homeState = HomeState.SEEKING;
//                homeLegStartAngle = angle;
//                homeMessage = "seeking CW";
//                setAnglePower(-TURRET_HOME_POWER);
//                return;
//            }
//        }
//        setAnglePower(TURRET_HOME_BACKOFF_POWER); // CCW = angle up
//    }

//    private void failHoming(String why) {
//        motor.setPower(0);
//        homeState = HomeState.FAILED;
//        homeMessage = "FAILED: " + why;
//    }

//    /** Power in the angle frame: positive = CCW (angle up). Applies TURRET_POWER_SIGN. */
//    private void setAnglePower(double power) {
//        motor.setPower(TURRET_POWER_SIGN * Range.clip(power, -1.0, 1.0));
//    }

    public double getTrim() {
        return trimDeg;
    }

    /** Nudge the aim trim. Call every loop with a signed rate (deg/s) and the loop's dt. */
    public void adjustTrim(double rateDegPerSec, double dtSeconds) {
        trimDeg += rateDegPerSec * dtSeconds;
    }

    // ================= GEOMETRY =================
    /** Turret pivot in field coordinates, accounting for robot heading. */
    private double[] turretWorldPosition(Pose robot) {
        double h = robot.heading();
        double cos = Math.cos(h), sin = Math.sin(h);
        return new double[]{
                robot.x() + TURRET_OFFSET_FORWARD * cos - TURRET_OFFSET_LEFT * sin,
                robot.y() + TURRET_OFFSET_FORWARD * sin + TURRET_OFFSET_LEFT * cos
        };
    }

    public double getDistanceToGoal(Pose robot) {
        double[] t = turretWorldPosition(robot);
        return Math.hypot(goalX - t[0], goalY - t[1]);
    }

    /**
     * Desired turret angle in degrees in the turret convention (0 = robot right, 90 = robot front, CCW positive),
     * plus trim. Trig: the turret's field direction is (heading - 90 + angle), so angle = fieldAngle - heading + 90.
     * The result is not wrapped; aimAtGoal unwraps it to the nearest equivalent of the current angle.
     */
    public double calculateTargetAngle(Pose robot) {
        double[] t = turretWorldPosition(robot);
        double fieldAngle = Math.toDegrees(Math.atan2(goalY - t[1], goalX - t[0]));
        double frontRelative = normalize180(fieldAngle - Math.toDegrees(robot.heading())); // 0 = front
        return frontRelative + TURRET_FRONT_ANGLE_DEG + trimDeg;
    }

    /**
     * Same as {@link #calculateTargetAngle} but aims from where the robot will be when the ball
     * lands, so the shot leads a moving robot. Velocity is in field frame, inches per second.
     */
    public double calculateTargetAngleWithPrediction(Pose robot, double velX, double velY) {
        if (Math.hypot(velX, velY) < MIN_SPEED_FOR_PREDICTION) {
            return calculateTargetAngle(robot);
        }
        double lead = getDistanceToGoal(robot) / PROJECTILE_SPEED_IN_PER_SEC + SYSTEM_DELAY_SECONDS;
        Pose predicted = new Pose(robot.x() + velX * lead, robot.y() + velY * lead, robot.heading());
        return calculateTargetAngle(predicted);
    }

    // ================= CONTROL =================
    /**
     * Call every loop. Checks the magnet first (re-homes on a clockwise pass), then either runs start / button
     * homing if it's active, or aims at the goal.
     */
    public void aimAtGoal(Pose robot, double velX, double velY) {
//        boolean rehomed = senseLimit();
//        if (isHoming()) {
//            runHoming(rehomed);
//            return;
//        }

        double desired = calculateTargetAngleWithPrediction(robot, velX, velY);
        double current = getAngle();

        // Unwrap the desired angle to the nearest equivalent of where the turret already is.
        // This is the whole "infinite turret" trick: never take the long way around.
        targetAngleDeg = current + normalize180(desired - current);
        update();
    }

    /** PD loop toward {@code targetAngleDeg}. */
    private void update() {
        double current = getAngle();
        double error = targetAngleDeg - current;

        long now = System.nanoTime();
        double dt = Math.max((now - lastNanos) / 1e9, 0.001);
        lastNanos = now;

        double velocity = (current - previousAngleDeg) / dt;
        double power = TURRET_KP * error - TURRET_KD * velocity;

        if (Math.abs(error) <= TURRET_DEADBAND_DEG) {
            power = 0;
        } else if (Math.abs(power) < TURRET_MIN_POWER) {
            power = Math.signum(error) * TURRET_MIN_POWER;
        }

        motor.setPower(TURRET_POWER_SIGN * Range.clip(power, -TURRET_MAX_POWER, TURRET_MAX_POWER));
        previousAngleDeg = current;
    }

    // ================= LOCK (manual fallback) =================
    /** Freeze the turret at the angle it is at right now. Then call {@link #hold()} every loop instead of aimAtGoal. */
    public void lockHere() {
//        cancelHoming();
        targetAngleDeg = getAngle();
        previousAngleDeg = targetAngleDeg;
        lastNanos = System.nanoTime();
    }

    /** Hold the angle saved by {@link #lockHere()} (stops aiming at the goal). Call every loop while locked. */
    public void hold() {
//        senseLimit(); // keep the edge detection's state current so re-homing still works after unlocking
        update();
    }

    public double getTargetAngle() {
        return targetAngleDeg;
    }

    public void stop() {
//        cancelHoming();
        motor.setPower(0);
    }

    private static double normalize180(double deg) {
        deg %= 360.0;
        if (deg > 180.0) deg -= 360.0;
        if (deg <= -180.0) deg += 360.0;
        return deg;
    }
}
