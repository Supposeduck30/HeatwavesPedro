package org.firstinspires.ftc.teamcode.biobuzz.auto;

import static org.firstinspires.ftc.teamcode.biobuzz.auto.BioBuzzPIDFAutoBase.*;
import static org.firstinspires.ftc.teamcode.biobuzz.teleop.BioBuzzConfig.*;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.biobuzz.teleop.BioBuzzPose;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Same curve as {@link BioBuzzMoveAuto}, but: wait WAIT_SEC at the start pose, shoot the preload from there,
 * then follow the curve to the park pose and hold it.
 *
 * Shooting is the same as {@link BioBuzzPIDFAutoBase} (PIDF flywheel with voltage-compensated F, and the same
 * READY_HOLD_SEC / SPIN_UP_TIMEOUT_SEC / BLOCKER_OPEN_SEC / FEED_SECONDS). The flywheel starts spinning
 * SPIN_UP_LEAD_SEC before the wait ends so it is already at speed when the wait is over.
 */
@Autonomous(name = "Wait 15 + Shoot + Park (curve)", group = "BioBuzz Auto")
public class BioBuzzWaitShootParkAuto extends OpMode {

    private static final Pose START    = deg(57.8797, 134.3579, 270);
    private static final Pose CONTROL1 = deg(66.4981, 104.0545, 0);
    private static final Pose PARK     = deg(9.8428, 116.6185, 270);

    // ================= TIMING (TUNE) =================
    /** Sit still at the start pose this long before shooting. */
    public static final double WAIT_SEC = 15.0;
    /** Start the flywheel this long before the wait ends. */
    public static final double SPIN_UP_LEAD_SEC = 3.0;

    // ================= SHOOTER VOLTAGE COMPENSATION =================
    private static final double VOLTAGE_UPDATE_SEC = 0.5;
    private static final double VOLTAGE_ALPHA = 0.2;
    private static final double MIN_VALID_VOLTAGE = 9.0;

    private enum State { WAIT, SPIN_UP, OPEN_BLOCKER, FEED, DRIVE_TO_PARK, DONE }

    private Follower follower;
    private DcMotorEx shooter;
    private DcMotor intake, transfer;
    private Servo blocker;
    private VoltageSensor battery;

    private double voltage = SHOOTER_F_NOMINAL_VOLTAGE;
    private final ElapsedTime voltageTimer = new ElapsedTime();

    private Path toPark;

    private State state = State.WAIT;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime readyTimer = new ElapsedTime();
    private boolean wasReady = false;
    private boolean shooterEnabled = false;

    private static Pose deg(double x, double y, double headingDeg) {
        return new Pose(x, y, Math.toRadians(headingDeg));
    }

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);

        shooter  = hardwareMap.get(DcMotorEx.class, SHOOTER);
        intake   = hardwareMap.get(DcMotor.class, INTAKE);
        transfer = hardwareMap.get(DcMotor.class, TRANSFER);
        blocker  = hardwareMap.get(Servo.class, BLOCKER);

        shooter.setDirection(SHOOTER_DIR);
        intake.setDirection(INTAKE_DIR);
        transfer.setDirection(TRANSFER_DIR);

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (hardwareMap.voltageSensor.iterator().hasNext()) {
            battery = hardwareMap.voltageSensor.iterator().next();
            double v = battery.getVoltage();
            if (v >= MIN_VALID_VOLTAGE) voltage = v;
        }
        applyShooterCoefficients();

        blocker.setPosition(BLOCKER_BLOCK);

        toPark = Paths.curve(START, CONTROL1, PARK).linear(START, PARK);
        follower.setPose(START);

        telemetry.addLine("Place the robot on the start pose, then press play.");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.setPose(START);
        follower.update();
        follower.hold(START);
        setState(State.WAIT);
    }

    @Override
    public void loop() {
        follower.update();

        updateVoltage();
        shooter.setVelocity(shooterEnabled ? SHOOTER_FIXED_VELOCITY : 0);
        double shooterVelocity = shooter.getVelocity();
        boolean ready = Math.abs(SHOOTER_FIXED_VELOCITY - shooterVelocity) < SHOOTER_READY_TOLERANCE;

        switch (state) {
            case WAIT:
                if (stateTimer.seconds() >= WAIT_SEC - SPIN_UP_LEAD_SEC && !shooterEnabled) {
                    shooterEnabled = true;
                    intake.setPower(PRIMARY_FEED_POWER); // intake spins with the flywheel, transfer waits for FEED
                }
                if (stateTimer.seconds() >= WAIT_SEC) {
                    setState(State.SPIN_UP);
                }
                break;

            case SPIN_UP:
                if (ready && !wasReady) readyTimer.reset();
                wasReady = ready;
                if ((ready && readyTimer.seconds() >= READY_HOLD_SEC) || stateTimer.seconds() > SPIN_UP_TIMEOUT_SEC) {
                    // Intake must be stopped before the blocker opens.
                    intake.setPower(0);
                    transfer.setPower(0);
                    blocker.setPosition(BLOCKER_OPEN);
                    setState(State.OPEN_BLOCKER);
                }
                break;

            case OPEN_BLOCKER:
                if (stateTimer.seconds() >= BLOCKER_OPEN_SEC) {
                    intake.setPower(PRIMARY_FEED_POWER);
                    transfer.setPower(PRIMARY_FEED_POWER);
                    setState(State.FEED);
                }
                break;

            case FEED:
                if (stateTimer.seconds() >= FEED_SECONDS) {
                    blocker.setPosition(BLOCKER_BLOCK);
                    intake.setPower(0);
                    transfer.setPower(0);
                    shooterEnabled = false;
                    shooter.setVelocity(0);
                    follower.follow(toPark);
                    setState(State.DRIVE_TO_PARK);
                }
                break;

            case DRIVE_TO_PARK:
                if (follower.atParametricEnd() || stateTimer.seconds() > DRIVE_TIMEOUT_SEC) {
                    follower.hold(PARK);
                    BioBuzzPose.savePose(follower.pose());
                    setState(State.DONE);
                }
                break;

            case DONE:
                break;
        }

        Pose pose = follower.pose();
        telemetry.addData("State", "%s  (%.1f s)", state, stateTimer.seconds());
        if (state == State.WAIT) {
            telemetry.addData("Shooting in", "%.1f s", Math.max(0, WAIT_SEC - stateTimer.seconds()));
        }
        telemetry.addData("X / Y (in)", "%.2f / %.2f", pose.x(), pose.y());
        telemetry.addData("Heading (deg)", "%.2f", Math.toDegrees(pose.heading()));
        telemetry.addData("Shooter target / actual", "%.0f / %.0f  (%s)",
                shooterEnabled ? SHOOTER_FIXED_VELOCITY : 0, shooterVelocity, ready ? "READY" : "not ready");
        telemetry.addData("Shooter F", "%.3f applied at %.2f V%s",
                compensatedF(), voltage, battery == null ? ", NO VOLTAGE SENSOR" : "");
        telemetry.update();
    }

    @Override
    public void stop() {
        shooter.setVelocity(0);
        intake.setPower(0);
        transfer.setPower(0);
        BioBuzzPose.savePose(follower.pose());
    }

    private void setState(State next) {
        state = next;
        stateTimer.reset();
    }

    /** SHOOTER_F is defined at SHOOTER_F_NOMINAL_VOLTAGE; a lower battery needs proportionally more. */
    private double compensatedF() {
        return SHOOTER_F * SHOOTER_F_NOMINAL_VOLTAGE / voltage;
    }

    private void updateVoltage() {
        if (battery == null || voltageTimer.seconds() < VOLTAGE_UPDATE_SEC) return;
        double v = battery.getVoltage();
        if (v >= MIN_VALID_VOLTAGE) voltage += VOLTAGE_ALPHA * (v - voltage);
        applyShooterCoefficients();
    }

    private void applyShooterCoefficients() {
        voltageTimer.reset();
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, compensatedF()));
    }
}
