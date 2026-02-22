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
    private static final double COUNTS_PER_DEGREE = 366.0 / 90.0;

    // ================= LIMITS =================
    private static final double MIN_ANGLE = 0.0;
    private static final double MAX_ANGLE = 180.0;

    // ================= GOAL LOCATION =================
    private static final double GOAL_X = 2.7;
    private static final double GOAL_Y = 140.2;

    // ================= TURRET PIVOT OFFSET =================
    private static final double TURRET_OFFSET_FORWARD = -6.0;
    private static final double TURRET_OFFSET_STRAFE  =  0.0;

    // ================= FINE TRIM =================
    private static final double ANGLE_OFFSET = 3.5;

    // ================= PREDICTIVE AIMING =================
    private static final double XY_SCALAR                   = 0.4;
    private static final double MIN_VELOCITY_FOR_PREDICTION = 2.0;

    // ================= PID CONTROL =================
    private static final double KP        = 0.032;
    private static final double KD        = 0.006;
    private static final double MAX_POWER = 1;
    // Deadband removed — motor corrects continuously for tighter tracking.
    // If jitter/chatter becomes a problem, add back a small value (0.1 +r 0.2).

    private double targetAngle   = 90.0;
    private double previousAngle = 0.0;
    private long   lastTime      = 0;

    // ================= CONSTRUCTOR =================
    public TurretController(HardwareMap hardwareMap, String motorName) {
        turretMotor = hardwareMap.get(DcMotorEx.class, motorName);

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        lastTime = System.currentTimeMillis();
    }

    // ================= GETTERS =================
    public double getCurrentAngle() {
        return turretMotor.getCurrentPosition() / COUNTS_PER_DEGREE;
    }

    public int getRawTicks() {
        return turretMotor.getCurrentPosition();
    }

    public void resetEncoder() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        targetAngle = 0;
        turretMotor.setPower(0);
    }

    public double getDistanceToGoal(Pose currentPose) {
        double[] turretWorld = getTurretWorldPosition(currentPose);
        double dx = GOAL_X - turretWorld[0];
        double dy = GOAL_Y - turretWorld[1];
        return Math.sqrt(dx * dx + dy * dy);
    }

    // ================= TURRET WORLD POSITION =================
    private double[] getTurretWorldPosition(Pose robotPose) {
        double heading = robotPose.getHeading();

        double worldOffsetX = TURRET_OFFSET_FORWARD * Math.cos(heading)
                - TURRET_OFFSET_STRAFE  * Math.sin(heading);
        double worldOffsetY = TURRET_OFFSET_FORWARD * Math.sin(heading)
                + TURRET_OFFSET_STRAFE  * Math.cos(heading);

        return new double[]{
                robotPose.getX() + worldOffsetX,
                robotPose.getY() + worldOffsetY
        };
    }

    // ================= CALCULATION LOGIC =================
    public double calculateTurretAngle(Pose currentPose) {
        double[] turretWorld = getTurretWorldPosition(currentPose);

        double dx = GOAL_X - turretWorld[0];
        double dy = GOAL_Y - turretWorld[1];
        double absTargetAngle = Math.toDegrees(Math.atan2(dy, dx));

        double robotHeading = Math.toDegrees(currentPose.getHeading());

        double relativeAngle = absTargetAngle - robotHeading;
        while (relativeAngle > 180)   relativeAngle -= 360;
        while (relativeAngle <= -180) relativeAngle += 360;

        double finalTurretAngle = relativeAngle + 90.0 + ANGLE_OFFSET;
        return Range.clip(finalTurretAngle, MIN_ANGLE, MAX_ANGLE);
    }

    public double calculateTurretAngleWithPrediction(Pose currentPose, Pose velocity) {
        double speed = Math.sqrt(
                velocity.getX() * velocity.getX() +
                        velocity.getY() * velocity.getY()
        );

        if (speed < MIN_VELOCITY_FOR_PREDICTION) {
            return calculateTurretAngle(currentPose);
        }

        double predictedX = currentPose.getX() + (velocity.getX() * XY_SCALAR);
        double predictedY = currentPose.getY() + (velocity.getY() * XY_SCALAR);
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

        if (dt <= 0.001) dt = 0.001;

        double p = KP * error;
        double derivative = (currentAngle - previousAngle) / dt;
        double d = -KD * derivative;

        double power = p + d;

        double currentMaxPower = MAX_POWER;
        if ((currentAngle < 10.0 && power < 0) || (currentAngle > 170.0 && power > 0)) {
            currentMaxPower = 0.25;
        }

        power = Range.clip(power, -currentMaxPower, currentMaxPower);

        turretMotor.setPower(power);
        previousAngle = currentAngle;
    }

    public void aimAtGoalWithPrediction(Pose currentPose, Pose velocity) {
        double target = calculateTurretAngleWithPrediction(currentPose, velocity);
        setTargetAngle(target);
        update();
    }
}