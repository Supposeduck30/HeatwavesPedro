package org.firstinspires.ftc.teamcode.biobuzz.teleop;

import static org.firstinspires.ftc.teamcode.biobuzz.teleop.BioBuzzConfig.*;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

/**
 * Field-aware turret for the BioBuzz robot.
 *
 * Unlike the Decode turret (0..180 deg, clamped) this one is continuous: there are no angle limits
 * and no wrap-around handling. The angle is a running total from the encoder, and every update the
 * turret takes the shortest way to the target (so the error is always within +-180 deg).
 * Make sure the wiring is on a slip ring or otherwise able to spin freely.
 *
 * Turret angle convention: 0 = robot front, positive = counter-clockwise (same as robot heading).
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

        previousAngleDeg = getAngle();
        lastNanos = System.nanoTime();
    }

    // ================= GOAL TOGGLING =================
    public void toggleGoal() {
        targetingSecondaryGoal = !targetingSecondaryGoal;
        goalX = targetingSecondaryGoal ? alliance.secondaryGoalX : alliance.primaryGoalX;
        goalY = targetingSecondaryGoal ? alliance.secondaryGoalY : alliance.primaryGoalY;
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
        encoderOffsetTicks = -motor.getCurrentPosition();
        targetAngleDeg = TURRET_START_ANGLE_DEG;
        previousAngleDeg = getAngle();
        motor.setPower(0);
    }

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

    /** Desired turret angle in degrees relative to the robot, normalized to (-180, 180] plus trim. */
    public double calculateTargetAngle(Pose robot) {
        double[] t = turretWorldPosition(robot);
        double fieldAngle = Math.toDegrees(Math.atan2(goalY - t[1], goalX - t[0]));
        return normalize180(fieldAngle - Math.toDegrees(robot.heading())) + trimDeg;
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
    public void aimAtGoal(Pose robot, double velX, double velY) {
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

        motor.setPower(Range.clip(power, -TURRET_MAX_POWER, TURRET_MAX_POWER));
        previousAngleDeg = current;
    }

    public double getTargetAngle() {
        return targetAngleDeg;
    }

    public void stop() {
        motor.setPower(0);
    }

    private static double normalize180(double deg) {
        deg %= 360.0;
        if (deg > 180.0) deg -= 360.0;
        if (deg <= -180.0) deg += 360.0;
        return deg;
    }
}
