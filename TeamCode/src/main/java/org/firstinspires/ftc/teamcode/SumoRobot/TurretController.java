package org.firstinspires.ftc.teamcode.SumoRobot;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class TurretController {

    private DcMotorEx turretMotor;

    // ================= MOTOR + GEAR =================
    private static final double COUNTS_PER_MOTOR_REV = 384.5; // 5203 Series 435 RPM
    private static final double GEAR_RATIO = 110.0 / 28.0;
    private static final double COUNTS_PER_DEGREE = (COUNTS_PER_MOTOR_REV * GEAR_RATIO) / 360.0;

    // ================= LIMITS =================
    private static final double MIN_ANGLE = 0.0;
    private static final double MAX_ANGLE = 180.0;

    // ================= GOAL LOCATION =================
    private static final double GOAL_X = 2.3;
    private static final double GOAL_Y = 142.0;

    // ================= PID CONTROL =================
    private double targetAngle = 90.0; // Default center
    private double previousAngle = 0.0;
    private long lastTime = 0;

    // PID Coefficients
    private static final double KP = 0.032;
    private static final double KD = 0.006;
    private static final double MAX_POWER = 0.75;
    private static final double DEADBAND = 1.0;

    // ================= CONSTRUCTOR =================
    public TurretController(HardwareMap hardwareMap, String motorName) {
        turretMotor = hardwareMap.get(DcMotorEx.class, motorName);

        // --- CRITICAL DISCONNECT FIX ---
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Hub doesn't fight custom PID

        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        lastTime = System.currentTimeMillis();
    }

    // ================= GETTERS =================
    public double getCurrentAngle() {
        return -1.0 * turretMotor.getCurrentPosition() / COUNTS_PER_DEGREE;
    }

    public void resetEncoder() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        targetAngle = 0;
        turretMotor.setPower(0);
    }

    public double getDistanceToGoal(Pose currentPose) {
        double dx = GOAL_X - currentPose.getX();
        double dy = GOAL_Y - currentPose.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }

    // ================= CALCULATION LOGIC =================
    /**
     * Basic Aiming (Used by Telemetry and Fallback)
     */
    public double calculateTurretAngle(Pose currentPose) {
        // 1. Calculate Absolute Angle to Goal
        double dx = GOAL_X - currentPose.getX();
        double dy = GOAL_Y - currentPose.getY();
        double absTargetAngle = Math.toDegrees(Math.atan2(dy, dx));

        // 2. Get Robot Heading
        double robotHeading = Math.toDegrees(currentPose.getHeading());

        // 3. Calculate Relative Angle
        double relativeAngle = absTargetAngle - robotHeading;

        // Normalize (-180 to 180)
        while (relativeAngle > 180) relativeAngle -= 360;
        while (relativeAngle <= -180) relativeAngle += 360;

        // 4. Convert to Turret Frame (0 is Right, 90 is Front)
        double finalTurretAngle = relativeAngle + 90.0;

        return Range.clip(finalTurretAngle, MIN_ANGLE, MAX_ANGLE);
    }

    /**
     * Predictive Aiming (Used by Loop)
     */
    public double calculateTurretAngleWithPrediction(Pose currentPose, Pose velocity) {
        double XY_SCALAR = 0.4;

        // Predict Robot Position
        double predictedX = currentPose.getX() + (velocity.getX() * XY_SCALAR);
        double predictedY = currentPose.getY() + (velocity.getY() * XY_SCALAR);

        // Create a fake pose at predicted location to reuse logic
        Pose predictedPose = new Pose(predictedX, predictedY, currentPose.getHeading());

        return calculateTurretAngle(predictedPose);
    }

    // ================= PID LOOP =================
    public void setTargetAngle(double angle) {
        this.targetAngle = Range.clip(angle, MIN_ANGLE, MAX_ANGLE);
    }

    public void update() {
        double currentAngle = getCurrentAngle();
        double error = targetAngle - currentAngle;

        long now = System.currentTimeMillis();
        double dt = (now - lastTime) / 1000.0;
        lastTime = now;

        // Prevent divide by zero
        if (dt <= 0.001) dt = 0.001;

        if (Math.abs(error) < DEADBAND) {
            turretMotor.setPower(0);
            return;
        }

        double p = KP * error;
        double derivative = (currentAngle - previousAngle) / dt;
        double d = -KD * derivative;

        double power = p + d;

        // Safety Clamp
        power = Range.clip(power, -MAX_POWER, MAX_POWER);

        turretMotor.setPower(power);
        previousAngle = currentAngle;
    }

    public void aimAtGoalWithPrediction(Pose currentPose, Pose velocity) {
        double target = calculateTurretAngleWithPrediction(currentPose, velocity);
        setTargetAngle(target);
        update();
    }
}
