package org.firstinspires.ftc.teamcode.SumoRobot;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TurretController {

    private DcMotorEx turretMotor;

    // ================= MOTOR + GEAR =================
    private static final double COUNTS_PER_MOTOR_REV = 383.6;
    private static final double GEAR_RATIO = 3.92;
    private static final double COUNTS_PER_DEGREE = (COUNTS_PER_MOTOR_REV * GEAR_RATIO) / 360.0;

    // ================= LIMITS =================
    private static final double MIN_ANGLE = 0.0;
    private static final double MAX_ANGLE = 180.0;

    // ================= GOAL =================
    private static final double GOAL_X = 2.3;
    private static final double GOAL_Y = 142.0;

    // ================= PID CONTROL =================
    private double targetAngle = 90.0;

    // PID State
    private double integralSum = 0.0;
    private double previousAngle = 0.0;
    private long lastTime = 0;

    // PID Coefficients - TUNED FOR SPEED AND SMOOTHNESS
    private static final double KP = 0.025;      // INCREASED - faster response
    private static final double KI = 0.0;        // DISABLED - causes jitter
    private static final double KD = 0.008;      // Low - minimal damping

    private static final double MAX_POWER = 0.70;  // INCREASED - faster movement
    private static final double ANGLE_TOLERANCE = 1.5; // Precise
    private static final double INTEGRAL_LIMIT = 25;
    private static final double MIN_POWER_THRESHOLD = 0.06; // INCREASED - overcome friction faster
    private static final double MAX_VELOCITY_DEG_PER_SEC = 150.0; // INCREASED - allow faster movement

    // Deadband - stops tiny corrections that cause jitter
    private static final double DEADBAND = 1.2; // Stop earlier to prevent jitter

    private int onTargetCounter = 0;
    private static final int ON_TARGET_LOOPS_REQUIRED = 3;

    // ================= CONSTRUCTOR =================
    public TurretController(HardwareMap hardwareMap, String motorName) {
        turretMotor = hardwareMap.get(DcMotorEx.class, motorName);

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        lastTime = System.currentTimeMillis();
        previousAngle = 0;
    }

    // ============================================================
    // SIMPLE ANGLE CONVERSION - NO NORMALIZATION
    // ============================================================

    /**
     * Get current turret angle in degrees
     * Formula: -1 * encoder / COUNTS_PER_DEGREE = degrees
     */
    public double getCurrentAngle() {
        return -1.0 * turretMotor.getCurrentPosition() / COUNTS_PER_DEGREE;
    }

    /**
     * Convert angle to encoder ticks
     */
    private int angleToTicks(double angleDegrees) {
        return (int)(-1.0 * angleDegrees * COUNTS_PER_DEGREE);
    }

    // ============================================================
    // ANGLE TO GOAL CALCULATION - NO NORMALIZATION
    // ============================================================

    public double calculateAbsoluteAngleToGoal(Pose currentPose) {
        double dx = GOAL_X - currentPose.getX();
        double dy = GOAL_Y - currentPose.getY();
        return Math.toDegrees(Math.atan2(dy, dx));
    }

    public double calculateTurretAngle(Pose currentPose) {
        double absAngle = calculateAbsoluteAngleToGoal(currentPose);
        double heading = Math.toDegrees(currentPose.getHeading());

        // Angle relative to robot forward
        double relativeAngle = absAngle - heading;

        // Convert to turret frame (add 90 because 0° = right, 90° = front)
        double turretAngle = relativeAngle + 90.0;

        // Clamp to limits
        if (turretAngle < MIN_ANGLE) turretAngle = MIN_ANGLE;
        if (turretAngle > MAX_ANGLE) turretAngle = MAX_ANGLE;

        return turretAngle;
    }

    // ============================================================
    // PID CONTROL
    // ============================================================

    public void setTargetAngle(double angleDegrees) {
        targetAngle = angleDegrees;
        if (targetAngle < MIN_ANGLE) targetAngle = MIN_ANGLE;
        if (targetAngle > MAX_ANGLE) targetAngle = MAX_ANGLE;
    }

    public void update() {
        double currentAngle = getCurrentAngle();
        double error = targetAngle - currentAngle;

        // DEADBAND - stop tiny movements that cause jitter
        if (Math.abs(error) <= DEADBAND) {
            turretMotor.setPower(0);
            integralSum = 0;
            onTargetCounter = 0;
            return;
        }

        // Stop when on target
        if (Math.abs(error) <= ANGLE_TOLERANCE) {
            onTargetCounter++;

            if (onTargetCounter >= ON_TARGET_LOOPS_REQUIRED) {
                turretMotor.setPower(0);
                integralSum = 0;
                return;
            }
        } else {
            onTargetCounter = 0;
        }

        // Time delta
        long now = System.currentTimeMillis();
        double dt = (now - lastTime) / 1000.0;
        if (dt <= 0 || dt > 0.1) dt = 0.02;
        lastTime = now;

        // ----- SIMPLIFIED PD CONTROL (NO INTEGRAL) -----

        // Proportional term
        double p = KP * error;

        // Derivative term (damping)
        double derivative = (currentAngle - previousAngle) / dt;
        double d = -KD * derivative;

        // Total power
        double power = p + d;

        // Dynamic max power based on distance
        double maxPowerNow = MAX_POWER;
        if (Math.abs(error) < 10.0) {
            // Slow down when close
            maxPowerNow = MAX_POWER * 0.6;
        }

        // Clamp power
        if (power > maxPowerNow) power = maxPowerNow;
        if (power < -maxPowerNow) power = -maxPowerNow;

        // Minimum power to overcome friction (only when far from target)
        if (Math.abs(error) > ANGLE_TOLERANCE * 2 && Math.abs(power) < MIN_POWER_THRESHOLD) {
            if (power > 0) {
                power = MIN_POWER_THRESHOLD;
            } else {
                power = -MIN_POWER_THRESHOLD;
            }
        }

        turretMotor.setPower(power);

        previousAngle = currentAngle;
    }

    public void aimAtGoal(Pose currentPose) {
        double target = calculateTurretAngle(currentPose);
        setTargetAngle(target);
        update();
    }

    public void stop() {
        turretMotor.setPower(0);
        integralSum = 0;
        onTargetCounter = 0;
    }

    // ============================================================
    // STATUS
    // ============================================================

    public int getRawTicks() {
        return turretMotor.getCurrentPosition();
    }

    public double getRawAngle() {
        return -1.0 * turretMotor.getCurrentPosition() / COUNTS_PER_DEGREE;
    }

    public boolean isOnTarget() {
        return Math.abs(targetAngle - getCurrentAngle()) <= ANGLE_TOLERANCE &&
                onTargetCounter >= ON_TARGET_LOOPS_REQUIRED;
    }

    public boolean isBusy() {
        return Math.abs(turretMotor.getPower()) > MIN_POWER_THRESHOLD;
    }

    public void resetEncoder() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        stop();
    }

    // ============================================================
    // DEBUG GETTERS
    // ============================================================

    public double getError() {
        return targetAngle - getCurrentAngle();
    }

    public double getIntegralSum() {
        return integralSum;
    }

    public int getOnTargetCounter() {
        return onTargetCounter;
    }

    public double getDistanceToGoal(Pose currentPose) {
        double dx = GOAL_X - currentPose.getX();
        double dy = GOAL_Y - currentPose.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }

    public double getMotorPower() {
        return turretMotor.getPower();
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public boolean isGoalReachable(Pose currentPose) {
        double angle = calculateTurretAngle(currentPose);
        return angle >= MIN_ANGLE && angle <= MAX_ANGLE;
    }
}