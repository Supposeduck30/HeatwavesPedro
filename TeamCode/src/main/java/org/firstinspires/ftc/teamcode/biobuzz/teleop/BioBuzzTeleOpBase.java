package org.firstinspires.ftc.teamcode.biobuzz.teleop;

import static org.firstinspires.ftc.teamcode.biobuzz.teleop.BioBuzzConfig.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * BioBuzz TeleOp, ported from the Decode BLUETELEOP / REDTELEOP to Pedro 3.
 * Use {@link BioBuzzBlueTeleOp} or {@link BioBuzzRedTeleOp}; they only pick the alliance.
 *
 * gamepad1: left stick = drive/strafe, right stick = turn, triangle = reset pose
 *           HOLD to drive to and hold a point: square = empty gate, circle = park,
 *           left bumper = shoot far, right bumper = shoot close
 * gamepad2: right bumper = intake in (intake + transfer, transfer stops after 2 balls via the sensor),
 *           left bumper = shoot feed (intake + transfer both run, ball count resets),
 *           cross = reverse both (ball count resets), triangle (hold) = open blocker,
 *           circle = toggle primary/secondary goal, dpad down = zero ball count
 *
 * The shooter (single motor) always spins at the speed the distance to the goal calls for (no idle speed; the
 * blocker and intake decide when balls fire).
 *
 * TURRET IS DISABLED for now: BioBuzzTurret is not created or called. To bring it back, construct
 * it in init(), call turret.aimAtGoal(pose, velX, velY) in loop(), and take the goal/distance from
 * it instead of the helpers below. The velocity estimate is already computed for that.
 *
 * Tunables: {@link BioBuzzConfig} and {@link BioBuzzAlliance}.
 * Auto handoff (start pose + turret position): {@link BioBuzzPose}.
 */
public abstract class BioBuzzTeleOpBase extends OpMode {

    private enum HoldTarget { NONE, EMPTY_GATE, PARK, SHOOT_FAR, SHOOT_CLOSE }

    private final BioBuzzAlliance alliance;

    private Follower follower;

    private DcMotorEx shooter;
    private DcMotor intake, transfer;
    private DigitalChannel ballSensor;
    private Servo blocker;

    private HoldTarget activeHold = HoldTarget.NONE;
    private boolean lastGamepad2Circle = false;
    private boolean secondaryGoal = false;

    // Ball counting (rising edges on the sensor, debounced)
    private int ballCount = 0;
    private boolean lastSensorBall = false;
    private long lastSensorChangeMs = 0;

    // Field-frame velocity from pose differences (used for shot leading)
    private Pose lastPose = null;
    private long lastNanos = 0;
    private double velX = 0, velY = 0;

    private boolean firstLoop = true;

    protected BioBuzzTeleOpBase(BioBuzzAlliance alliance) {
        this.alliance = alliance;
    }

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        if (BioBuzzPose.hasPoseFromAuto()) {
            follower.setPose(BioBuzzPose.getPose());
        } else {
            follower.setPose(alliance.startPose);
        }

        shooter    = hardwareMap.get(DcMotorEx.class, SHOOTER);
        intake     = hardwareMap.get(DcMotor.class, INTAKE);
        transfer   = hardwareMap.get(DcMotor.class, TRANSFER);
        ballSensor = hardwareMap.get(DigitalChannel.class, BALL_SENSOR);
        blocker     = hardwareMap.get(Servo.class, BLOCKER);

        shooter.setDirection(SHOOTER_DIR);
        intake.setDirection(INTAKE_DIR);
        transfer.setDirection(TRANSFER_DIR);
        ballSensor.setMode(DigitalChannel.Mode.INPUT);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F));

        blocker.setPosition(BLOCKER_BLOCK);

        telemetry.addData("Status", "Initialized (" + alliance + ")");
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();

        // The first update just lets the localizer settle.
        if (firstLoop) {
            firstLoop = false;
            return;
        }

        long nowNanos = System.nanoTime();

        if (gamepad1.triangle) {
            follower.setPose(RESET_POSE);
        }

        /* ---------- HOLD POINTS / DRIVE ---------- */
        updateHold();
        if (activeHold == HoldTarget.NONE) {
            // manual() takes the follower out of HOLD mode, so only call it when not holding.
            follower.manual(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        }

        /* ---------- VELOCITY ESTIMATE ---------- */
        Pose pose = follower.pose();
        if (lastPose != null) {
            double dt = (nowNanos - lastNanos) / 1e9;
            if (dt > 0.005) {
                velX = (pose.x() - lastPose.x()) / dt;
                velY = (pose.y() - lastPose.y()) / dt;
            }
        }
        lastPose = pose;
        lastNanos = nowNanos;

        /* ---------- GOAL TOGGLE ---------- */
        boolean circleNow = gamepad2.circle;
        if (circleNow && !lastGamepad2Circle) {
            secondaryGoal = !secondaryGoal;
        }
        lastGamepad2Circle = circleNow;

        /* ---------- BLOCKER ---------- */
        blocker.setPosition(gamepad2.triangle ? BLOCKER_OPEN : BLOCKER_BLOCK);

        /* ---------- SHOOTER ---------- */
        double distance = distanceToGoal(pose);
        double targetVelocity = USE_FIXED_SHOOTER_VELOCITY
                ? SHOOTER_FIXED_VELOCITY
                : Range.clip(alliance.shooterVelocity(distance) * SHOOTER_VELOCITY_SCALE, 0, MAX_SHOOTER_VELOCITY);
        shooter.setVelocity(targetVelocity);

        /* ---------- INTAKE ---------- */
        updateBallCount();
        if (gamepad2.dpad_down) {
            ballCount = 0;
        }

        double intakePower = 0, transferPower = 0;
        if (gamepad2.cross) {
            // Spit everything out
            intakePower = INTAKE_REVERSE_POWER;
            transferPower = TRANSFER_REVERSE_POWER;
            ballCount = 0;
        } else if (gamepad2.left_bumper) {
            // Shooting: both run no matter what the sensor says
            double feed = secondaryGoal ? SECONDARY_FEED_POWER : PRIMARY_FEED_POWER;
            intakePower = feed;
            transferPower = feed;
            ballCount = 0;
        } else if (gamepad2.right_bumper) {
            // Intaking: intake always runs, transfer stops once the robot is full
            intakePower = INTAKE_IN_POWER;
            transferPower = ballCount >= MAX_BALLS_STORED ? 0 : TRANSFER_IN_POWER;
        }
        intake.setPower(intakePower);
        transfer.setPower(transferPower);

        /* ---------- TELEMETRY ---------- */
        double shooterVelocity = shooter.getVelocity();
        boolean ready = Math.abs(targetVelocity - shooterVelocity) < SHOOTER_READY_TOLERANCE;

        telemetry.addData("Alliance", alliance);
        telemetry.addData("Active goal", secondaryGoal ? "SECONDARY" : "PRIMARY");
        telemetry.addData("Holding", activeHold);
        telemetry.addData("--- POSE ---", "");
        telemetry.addData("X / Y (in)", "%.2f / %.2f", pose.x(), pose.y());
        telemetry.addData("Heading (deg)", "%.2f", Math.toDegrees(pose.heading()));
        telemetry.addData("Speed (in/s)", "%.1f", Math.hypot(velX, velY));
        telemetry.addData("Distance to goal (in)", "%.2f", distance);
        telemetry.addData("--- SHOOTER ---", "");
        telemetry.addData("Target (ticks/s)", "%.0f", targetVelocity);
        telemetry.addData("Actual (ticks/s)", "%.0f", shooterVelocity);
        telemetry.addData("Velocity error", "%.0f", targetVelocity - shooterVelocity);
        telemetry.addData("Ready", ready ? "YES" : "no");
        telemetry.addData("--- FEED ---", "");
        telemetry.addData("Intake / Transfer power", "%.1f / %.1f", intakePower, transferPower);
        telemetry.addData("Balls stored", "%d / %d  (sensor sees ball: %s)", ballCount, MAX_BALLS_STORED, lastSensorBall);
        telemetry.addData("Gamepad2 assigned", gamepad2.getGamepadId() != Gamepad.ID_UNASSOCIATED);
        telemetry.addData("Blocker", gamepad2.triangle ? "OPEN" : "BLOCKING");
        telemetry.update();
    }

    /**
     * Counts balls entering as debounced rising edges of the sensor, but only while intaking
     * (right bumper). Shooting and reversing reset the count in loop().
     */
    private void updateBallCount() {
        boolean raw = ballSensor.getState() == BALL_SENSOR_ACTIVE_HIGH;
        long now = System.currentTimeMillis();
        if (raw == lastSensorBall || now - lastSensorChangeMs < BALL_SENSOR_DEBOUNCE_MS) {
            return;
        }
        lastSensorBall = raw;
        lastSensorChangeMs = now;
        if (raw && gamepad2.right_bumper && !gamepad2.left_bumper && !gamepad2.cross) {
            ballCount++;
        }
    }

    /** Distance from the robot center to the active goal (no turret, so no pivot offset). */
    private double distanceToGoal(Pose pose) {
        double gx = secondaryGoal ? alliance.secondaryGoalX : alliance.primaryGoalX;
        double gy = secondaryGoal ? alliance.secondaryGoalY : alliance.primaryGoalY;
        return Math.hypot(gx - pose.x(), gy - pose.y());
    }

    /**
     * Hold a button to go to and hold its point; let go to drive again. Same rules as Decode:
     * only one hold at a time, and a hold in progress is not interrupted by another button.
     */
    private void updateHold() {
        if (activeHold != HoldTarget.NONE && !isPressed(activeHold)) {
            activeHold = HoldTarget.NONE;
        }
        if (activeHold != HoldTarget.NONE) return;

        for (HoldTarget target : new HoldTarget[]{
                HoldTarget.EMPTY_GATE, HoldTarget.PARK, HoldTarget.SHOOT_FAR, HoldTarget.SHOOT_CLOSE}) {
            if (isPressed(target)) {
                follower.hold(poseFor(target));
                activeHold = target;
                return;
            }
        }
    }

    private boolean isPressed(HoldTarget target) {
        switch (target) {
            case EMPTY_GATE:  return gamepad1.square;
            case PARK:        return gamepad1.circle;
            case SHOOT_FAR:   return gamepad1.left_bumper;
            case SHOOT_CLOSE: return gamepad1.right_bumper;
            default:          return false;
        }
    }

    private Pose poseFor(HoldTarget target) {
        switch (target) {
            case EMPTY_GATE:  return EMPTY_GATE_POSE;
            case PARK:        return alliance.parkPose;
            case SHOOT_FAR:   return alliance.shootFarPose;
            default:          return alliance.shootClosePose;
        }
    }

    @Override
    public void stop() {
        shooter.setVelocity(0);
        intake.setPower(0);
        transfer.setPower(0);
    }
}
