package org.firstinspires.ftc.teamcode.SumoRobot;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class TurretControllerRED {

    private DcMotorEx turretMotor;

    private int encoderOffset=0;

    // ================= MOTOR + GEAR =================
    private static final double COUNTS_PER_DEGREE = 366.0 / 90.0;

    // ================= LIMITS =================
    private static final double MIN_ANGLE = 0.0;
    private static final double MAX_ANGLE = 180.0;

    // ================= GOAL LOCATION =================
    private static final double GOAL_X = 139.1;
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
    private static final double KP        = 0.06;
    private static final double KD        = 0.008;
    private static final double MAX_POWER = 1;
    private static final double DEADBAND = 0.8;
    // Deadband removed — motor corrects continuously for tighter tracking.
    // If jitter/chatter becomes a problem, add back a small value (0.1 +r 0.2).

    private double targetAngle   = 90.0;
    private double previousAngle = 0.0;
    private long   lastTime      = 0;

    public void setEncoderOffsetRED(int offset) {
        this.encoderOffset=offset;
    }

    // ================= CONSTRUCTOR =================
    public TurretControllerRED(HardwareMap hardwareMap, String motorName) {
        turretMotor = hardwareMap.get(DcMotorEx.class, motorName);

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        lastTime = System.currentTimeMillis();
    }

    // ================= GETTERS =================
    public double getCurrentAngleRED() {
        return (turretMotor.getCurrentPosition() +encoderOffset) / COUNTS_PER_DEGREE;
    }

    public int getRawTicksRED() {
        return turretMotor.getCurrentPosition() +encoderOffset;
    }

    public void resetEncoderRED() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        targetAngle = 0;
        encoderOffset=0;
        turretMotor.setPower(0);
    }

    public double getDistanceToGoalRED(Pose currentPose) {
        double[] turretWorld = getTurretWorldPositionRED(currentPose);
        double dx = GOAL_X - turretWorld[0];
        double dy = GOAL_Y - turretWorld[1];
        return Math.sqrt(dx * dx + dy * dy);
    }

    // ================= TURRET WORLD POSITION =================
    private double[] getTurretWorldPositionRED(Pose robotPose) {
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
    public double calculateTurretAngleRED(Pose currentPose) {
        double[] turretWorld = getTurretWorldPositionRED(currentPose);

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

    public double calculateTurretAngleWithPredictionRED(Pose currentPose, Pose velocity) {
        double speed = Math.sqrt(
                velocity.getX() * velocity.getX() +
                        velocity.getY() * velocity.getY()
        );

        if (speed < MIN_VELOCITY_FOR_PREDICTION) {
            return calculateTurretAngleRED(currentPose);
        }

        double predictedX = currentPose.getX() + (velocity.getX() * XY_SCALAR);
        double predictedY = currentPose.getY() + (velocity.getY() * XY_SCALAR);
        Pose predictedPose = new Pose(predictedX, predictedY, currentPose.getHeading());

        return calculateTurretAngleRED(predictedPose);
    }

    // ================= PID LOOP =================
    public void setTargetAngleRED(double angle) {
        this.targetAngle = Range.clip(angle, MIN_ANGLE, MAX_ANGLE);
    }

    public void updateRED() {
        double currentAngle = getCurrentAngleRED();
        double error = targetAngle - currentAngle;

        long now = System.currentTimeMillis();
        double dt = (now - lastTime) / 1000.0;
        lastTime = now;

        if (dt <= 0.001) dt = 0.001;

        if (Math.abs(error)<DEADBAND){
            turretMotor.setPower(0);
            previousAngle = currentAngle;
            return;
        }

        double p = KP * error;
        double derivative = (currentAngle - previousAngle) / dt;
        double d = -KD * derivative;

        double power = p + d;

        if (Math.abs(power) <0.05 && Math.abs(power) > 0.01){
            power=Math.signum(power) * 0.05;
        }

        double currentMaxPower = MAX_POWER;
        if ((currentAngle < 10.0 && power < 0) || (currentAngle > 170.0 && power > 0)) {
            currentMaxPower = 0.25;
        }

        power = Range.clip(power, -currentMaxPower, currentMaxPower);

        turretMotor.setPower(power);
        previousAngle = currentAngle;
    }

    public void aimAtGoalWithPredictionRED(Pose currentPose, Pose velocity) {
        double target = calculateTurretAngleWithPredictionRED(currentPose, velocity);
        setTargetAngleRED(target);
        updateRED();
    }
}