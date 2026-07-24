package org.firstinspires.ftc.teamcode.SumoRobot;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class TurretControllerRED {

    private DcMotorEx turretMotor;
    private int encoderOffset = 0;

    // ================= MOTOR + GEAR =================
    private static final double COUNTS_PER_DEGREE = 366.0 / 90.0;

    // ================= LIMITS =================
    private static final double MIN_ANGLE = 0.0;
    private static final double MAX_ANGLE = 180.0;

    // ================= GOAL LOCATIONS =================
    private static final double PRIMARY_GOAL_X = 180.1;
    private static final double PRIMARY_GOAL_Y = 187.2;

    private static final double SECONDARY_GOAL_X = 100.0;
    private static final double SECONDARY_GOAL_Y = 185.0;

    // Active tracking coordinates
    private double currentGoalX = PRIMARY_GOAL_X;
    private double currentGoalY = PRIMARY_GOAL_Y;
    private boolean targetingSecondaryGoal = false;

    // ================= TURRET PIVOT OFFSET =================
    private static final double TURRET_OFFSET_FORWARD = -6.0;
    private static final double TURRET_OFFSET_STRAFE  =  0.0;

    // ================= FINE TRIM =================
    public double ANGLE_OFFSETRED = 0;

    // ================= PREDICTIVE AIMING =================
    private static final double PROJECTILE_SPEED_IN_PER_SEC = 193.75;
    private static final double SYSTEM_DELAY_SECONDS = 0.0;
    private static final double MIN_VELOCITY_FOR_PREDICTION = 2.0;

    // ================= PID CONTROL =================
    private static final double KP        = 0.065;
    private static final double KD        = 0.004;
    private static final double MAX_POWER = 1.0;
    private static final double MIN_POWER = 0.08;

    private double targetAngle   = 90.0;
    private double previousAngle = 0.0;
    private long   lastTime      = 0;

    public void setEncoderOffsetRED(int offset) {
        this.encoderOffset = offset;
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

    // ================= GOAL TOGGLING =================
    public void toggleGoal() {
        targetingSecondaryGoal = !targetingSecondaryGoal;
        if (targetingSecondaryGoal) {
            currentGoalX = SECONDARY_GOAL_X;
            currentGoalY = SECONDARY_GOAL_Y;
        } else {
            currentGoalX = PRIMARY_GOAL_X;
            currentGoalY = PRIMARY_GOAL_Y;
        }
    }

    public boolean isTargetingSecondaryGoal() {
        return targetingSecondaryGoal;
    }

    // ================= GETTERS =================
    public double getCurrentAngleRED() {
        return (turretMotor.getCurrentPosition() + encoderOffset) / COUNTS_PER_DEGREE;
    }

    public int getRawTicks() {
        return turretMotor.getCurrentPosition() + encoderOffset;
    }

    public void resetEncoderRED() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        targetAngle = 0;
        encoderOffset = 0;
        turretMotor.setPower(0);
    }

    public double getDistanceToGoalRED(Pose currentPose) {
        double[] turretWorld = getTurretWorldPosition(currentPose);
        // Using dynamic goal coordinates here
        double dx = currentGoalX - turretWorld[0];
        double dy = currentGoalY - turretWorld[1];
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
    public double calculateTurretAngleRED(Pose currentPose) {
        double[] turretWorld = getTurretWorldPosition(currentPose);

        // Using dynamic goal coordinates here
        double dx = currentGoalX - turretWorld[0];
        double dy = currentGoalY - turretWorld[1];
        double absTargetAngle = Math.toDegrees(Math.atan2(dy, dx));

        double robotHeading = Math.toDegrees(currentPose.getHeading());

        double relativeAngle = absTargetAngle - robotHeading;
        while (relativeAngle > 180)   relativeAngle -= 360;
        while (relativeAngle <= -180) relativeAngle += 360;

        double finalTurretAngle = relativeAngle + 90.0 + ANGLE_OFFSETRED;
        return Range.clip(finalTurretAngle, MIN_ANGLE, MAX_ANGLE);
    }

    public double calculateTurretAngleWithPrediction(Pose currentPose, Pose velocity) {
        double speed = Math.sqrt(
                velocity.getX() * velocity.getX() +
                        velocity.getY() * velocity.getY()
        );

        if (speed < MIN_VELOCITY_FOR_PREDICTION) {
            return calculateTurretAngleRED(currentPose);
        }

        double distanceToGoal = getDistanceToGoalRED(currentPose);
        double timeOfFlight = distanceToGoal / PROJECTILE_SPEED_IN_PER_SEC;
        double dynamicTimeScalar = timeOfFlight + SYSTEM_DELAY_SECONDS;

        double predictedX = currentPose.getX() + (velocity.getX() * dynamicTimeScalar);
        double predictedY = currentPose.getY() + (velocity.getY() * dynamicTimeScalar);
        Pose predictedPose = new Pose(predictedX, predictedY, currentPose.getHeading());

        return calculateTurretAngleRED(predictedPose);
    }

    // ================= PID LOOP =================
    public void setTargetAngle(double angle) {
        this.targetAngle = Range.clip(angle, MIN_ANGLE, MAX_ANGLE);
    }

    public void update() {
        double currentAngle = getCurrentAngleRED();
        double error = targetAngle - currentAngle;

        long now = System.currentTimeMillis();
        double dt = (now - lastTime) / 1000.0;
        lastTime = now;

        if (dt <= 0.001) dt = 0.001;

        double p = KP * error;
        double derivative = (currentAngle - previousAngle) / dt;
        double d = -KD * derivative;

        double power = p + d;

        if (Math.abs(error) > 0.10 && Math.abs(power) < MIN_POWER) {
            power = Math.signum(error) * MIN_POWER;
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
        double target = calculateTurretAngleWithPrediction(currentPose, velocity);
        setTargetAngle(target);
        update();
    }
}