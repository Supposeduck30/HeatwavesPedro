package org.firstinspires.ftc.teamcode.biobuzz.auto;

import static org.firstinspires.ftc.teamcode.biobuzz.teleop.BioBuzzConfig.*;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
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
 * PIDF copy of {@link BioBuzzAutoBase}: identical route and sequence, but the flywheel runs on the hub's
 * velocity PIDF (RUN_USING_ENCODER + setVelocity) with SHOOTER_P / I / D / F from BioBuzzConfig instead of
 * bang-bang. Like the PIDF teleop and the tuners, F is scaled by SHOOTER_F_NOMINAL_VOLTAGE / battery voltage.
 * Use one of the two OpModes in {@link BioBuzzPIDFAutos} (PIDF Red 1 / Blue 1); they only pick the route.
 *
 * Sequence:
 *   1. Drive to the shoot pose while the flywheel spins up to SHOOTER_FIXED_VELOCITY.
 *   2. Hold the shoot pose until the flywheel is READY (or SPIN_UP_TIMEOUT_SEC passes).
 *   3. Stop the intake, open the blocker, wait BLOCKER_OPEN_SEC, then run intake + transfer for FEED_SECONDS
 *      (every ball takes about SECONDS_PER_BALL).
 *   4. Close the blocker, stop the flywheel, drive to the park pose and hold it.
 * The final pose is saved in {@link BioBuzzPose} (also in stop()) so the teleop can start from it.
 */
public abstract class BioBuzzPIDFAutoBase extends OpMode {

    // ================= SHOOTING (TUNE) =================
    /** Pollen preloaded on the robot. */
    public static final int    PRELOAD_BALLS = 4;
    /** Each ball leaves in about this long once the blocker is open and the feed is running. */
    public static final double SECONDS_PER_BALL = 0.2;
    /** Extra feed time so the last ball has fully left before the blocker closes. */
    public static final double FEED_MARGIN_SEC = 0.7;
    public static final double FEED_SECONDS = PRELOAD_BALLS * SECONDS_PER_BALL + FEED_MARGIN_SEC;
    /** Time for the blocker servo to get out of the way before the intake restarts. */
    public static final double BLOCKER_OPEN_SEC = 1.0;
    /** The flywheel must stay inside SHOOTER_READY_TOLERANCE this long before the first ball. */
    public static final double READY_HOLD_SEC = 0.5;  // was 0.15; longer so the wheel is fully at speed before the first ball
    /** Shoot anyway after this long so a slow flywheel cannot eat the whole auto. */
    public static final double SPIN_UP_TIMEOUT_SEC = 4.0;  // was 2.0

    // ================= DRIVING (TUNE) =================
    /** A segment counts as arrived when it is this close to the end pose... */
    public static final double POSITION_TOLERANCE_IN = 1.5;
    /** ...and within this many degrees of the end heading. */
    public static final double HEADING_TOLERANCE_DEG = 4.0;
    /** Move on after this long even if the tolerances were never met (the follower keeps holding the pose). */
    public static final double DRIVE_TIMEOUT_SEC = 5.0;

    /** After reaching the park pose, creep this far forward (along the park heading) to be sure it is inside the zone. */
    public static final double PARK_NUDGE_IN = 4.0;

    // ================= SHOOTER VOLTAGE COMPENSATION =================
    /** How often the battery voltage is read and the compensated F re-sent to the hub. */
    private static final double VOLTAGE_UPDATE_SEC = 0.5;
    /** Voltage low-pass filter (0..1, lower = smoother). The reading sags during every shot. */
    private static final double VOLTAGE_ALPHA = 0.2;
    /** Ignore readings below this (sensor glitch / brownout) so the compensation can't explode. */
    private static final double MIN_VALID_VOLTAGE = 9.0;

    private enum State { DRIVE_TO_SHOOT, SPIN_UP, OPEN_BLOCKER, FEED, DRIVE_TO_PARK, NUDGE_FORWARD, DONE }

    private final BioBuzzAutoRoute route;
    /** Shoot and park poses (default to the route's; a subclass can override the shoot pose). */
    private final Pose shoot, park;
    /** If non-null, the shoot -> park leg is a Bezier curve through this control point instead of a line. */
    private final Pose parkControl;

    private Follower follower;
    private DcMotorEx shooter;
    private DcMotor intake, transfer;
    private Servo blocker;
    private VoltageSensor battery;

    private double voltage = SHOOTER_F_NOMINAL_VOLTAGE;
    private final ElapsedTime voltageTimer = new ElapsedTime();

    private Path toShoot, toPark, toNudge;
    private Pose nudge;

    private State state = State.DRIVE_TO_SHOOT;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime readyTimer = new ElapsedTime();
    private boolean wasReady = false;
    /** The flywheel is being held at shooterTarget(); otherwise it is commanded to 0. */
    private boolean shooterEnabled = false;

    protected BioBuzzPIDFAutoBase(BioBuzzAutoRoute route) {
        this(route, null, null);
    }

    /**
     * @param shootOverride shoot pose to use instead of route.shoot (null = route's)
     * @param parkControl   Bezier control point for the shoot -> park curve (null = straight line)
     */
    protected BioBuzzPIDFAutoBase(BioBuzzAutoRoute route, Pose shootOverride, Pose parkControl) {
        this.route = route;
        this.shoot = shootOverride != null ? shootOverride : route.shoot;
        this.park = route.park;
        this.parkControl = parkControl;
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

        // Closed loop (same as BioBuzzPIDFTeleOpBase): the hub's velocity PIDF holds the target we pass to setVelocity().
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (hardwareMap.voltageSensor.iterator().hasNext()) {
            battery = hardwareMap.voltageSensor.iterator().next();
            double v = battery.getVoltage();
            if (v >= MIN_VALID_VOLTAGE) voltage = v;
        }
        applyShooterCoefficients();

        blocker.setPosition(BLOCKER_BLOCK);

        toShoot = Paths.line(route.start, shoot).linear(route.start.heading(), shoot.heading());
        toPark  = parkControl != null
                ? Paths.curve(shoot, parkControl, park).linear(shoot.heading(), park.heading())
                : Paths.line(shoot, park).linear(shoot.heading(), park.heading());

        // Short straight line forward from the park pose, same heading, so the robot ends up inside the zone.
        double parkHeading = park.heading();
        nudge = new Pose(park.x() + PARK_NUDGE_IN * Math.cos(parkHeading),
                park.y() + PARK_NUDGE_IN * Math.sin(parkHeading), parkHeading);
        toNudge = Paths.line(park, nudge).linear(parkHeading, parkHeading);

        follower.setPose(route.start);

        telemetry.addData("Route", route);
        telemetry.addData("Start (x, y, deg)", "%.1f, %.1f, %.1f",
                route.start.x(), route.start.y(), Math.toDegrees(route.start.heading()));
        telemetry.addLine("Place the robot on the start pose, then press play.");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.setPose(route.start);
        follower.update();
        follower.follow(toShoot);
        shooterEnabled = true;
        intake.setPower(PRIMARY_FEED_POWER);   // intake spins up with the flywheel, transfer waits for FEED
        setState(State.DRIVE_TO_SHOOT);
    }

    @Override
    public void loop() {
        follower.update();

        double shooterVelocity = shooter.getVelocity();
        boolean ready = Math.abs(shooterTarget() - shooterVelocity) < SHOOTER_READY_TOLERANCE;

        // PIDF: the hub holds the target velocity on its own; F follows the battery voltage.
        updateVoltage();
        shooter.setVelocity(shooterEnabled ? shooterTarget() : 0);

        switch (state) {
            case DRIVE_TO_SHOOT:
                if (arrived(shoot) || stateTimer.seconds() > DRIVE_TIMEOUT_SEC) {
                    follower.hold(shoot);
                    setState(State.SPIN_UP);
                }
                break;

            case SPIN_UP:
                // READY has to hold for a moment so one lucky reading on the way up does not count.
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
                // Blocker is open: now restart the intake and feed.
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
                if (arrived(park) || stateTimer.seconds() > DRIVE_TIMEOUT_SEC) {
                    follower.follow(toNudge);
                    setState(State.NUDGE_FORWARD);
                }
                break;

            case NUDGE_FORWARD:
                if (arrived(nudge) || stateTimer.seconds() > DRIVE_TIMEOUT_SEC) {
                    follower.hold(nudge);
                    BioBuzzPose.savePose(follower.pose());
                    setState(State.DONE);
                }
                break;

            case DONE:
                break;
        }

        Pose pose = follower.pose();
        telemetry.addData("Route", route);
        telemetry.addData("State", "%s  (%.1f s)", state, stateTimer.seconds());
        telemetry.addData("X / Y (in)", "%.2f / %.2f", pose.x(), pose.y());
        telemetry.addData("Heading (deg)", "%.2f", Math.toDegrees(pose.heading()));
        telemetry.addData("Shooter target / actual", "%.0f / %.0f  (%s)",
                shooterEnabled ? shooterTarget() : 0, shooterVelocity, ready ? "READY" : "not ready");
        telemetry.addData("Shooter PIDF", "P=%s F=%s (%.3f applied at %.2f V%s)", SHOOTER_P, SHOOTER_F,
                compensatedF(), voltage, battery == null ? ", NO VOLTAGE SENSOR" : "");
        telemetry.update();
    }

    @Override
    public void stop() {
        shooter.setVelocity(0);
        intake.setPower(0);
        transfer.setPower(0);
        // Hand the final pose to the teleop even if the auto ended before it finished.
        BioBuzzPose.savePose(follower.pose());
    }

    /** Flywheel target for this route: the shared fixed velocity plus the route's boost. */
    private double shooterTarget() {
        return SHOOTER_FIXED_VELOCITY + route.velocityBoost;
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

    private void setState(State next) {
        state = next;
        stateTimer.reset();
    }

    /** Close enough to the end pose of the current segment, in position and heading. */
    private boolean arrived(Pose target) {
        Pose pose = follower.pose();
        double distance = Math.hypot(target.x() - pose.x(), target.y() - pose.y());
        double headingError = target.heading() - pose.heading();
        headingError = Math.atan2(Math.sin(headingError), Math.cos(headingError));
        return follower.atParametricEnd()
                && distance < POSITION_TOLERANCE_IN
                && Math.abs(Math.toDegrees(headingError)) < HEADING_TOLERANCE_DEG;
    }
}
