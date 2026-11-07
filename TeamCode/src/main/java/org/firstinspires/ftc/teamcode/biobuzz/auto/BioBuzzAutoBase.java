package org.firstinspires.ftc.teamcode.biobuzz.auto;

import static org.firstinspires.ftc.teamcode.biobuzz.teleop.BioBuzzConfig.*;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.biobuzz.teleop.BioBuzzPose;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * BioBuzz autonomous: start -> shoot the preload -> park, all straight lines (see {@link BioBuzzAutoRoute}).
 * Use one of the two OpModes in {@link BioBuzzAutos} (Red 1 / Blue 1); they only pick the route.
 *
 * Sequence:
 *   1. Drive to the shoot pose while the flywheel spins up to SHOOTER_FIXED_VELOCITY.
 *   2. Hold the shoot pose until the flywheel is READY (or SPIN_UP_TIMEOUT_SEC passes).
 *   3. Stop the intake, open the blocker, wait BLOCKER_OPEN_SEC, then run intake + transfer for FEED_SECONDS
 *      (every ball takes about SECONDS_PER_BALL).
 *   4. Close the blocker, stop the flywheel, drive to the park pose and hold it.
 * The final pose is saved in {@link BioBuzzPose} (also in stop()) so the teleop can start from it.
 */
public abstract class BioBuzzAutoBase extends OpMode {

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
    public static final double READY_HOLD_SEC = 0.15;
    /** Shoot anyway after this long so a slow flywheel cannot eat the whole auto. */
    public static final double SPIN_UP_TIMEOUT_SEC = 2.0;

    // ================= DRIVING (TUNE) =================
    /** A segment counts as arrived when it is this close to the end pose... */
    public static final double POSITION_TOLERANCE_IN = 1.5;
    /** ...and within this many degrees of the end heading. */
    public static final double HEADING_TOLERANCE_DEG = 4.0;
    /** Move on after this long even if the tolerances were never met (the follower keeps holding the pose). */
    public static final double DRIVE_TIMEOUT_SEC = 5.0;

    /** After reaching the park pose, creep this far forward (along the park heading) to be sure it is inside the zone. */
    public static final double PARK_NUDGE_IN = 4.0;

    private enum State { DRIVE_TO_SHOOT, SPIN_UP, OPEN_BLOCKER, FEED, DRIVE_TO_PARK, NUDGE_FORWARD, DONE }

    private final BioBuzzAutoRoute route;

    private Follower follower;
    private DcMotorEx shooter;
    private DcMotor intake, transfer;
    private Servo blocker;

    private Path toShoot, toPark, toNudge;
    private Pose nudge;

    private State state = State.DRIVE_TO_SHOOT;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime readyTimer = new ElapsedTime();
    private boolean wasReady = false;
    /** Bang-bang: the flywheel is being regulated, and whether it is currently at full power. */
    private boolean shooterEnabled = false;
    private boolean shooterOn = false;

    protected BioBuzzAutoBase(BioBuzzAutoRoute route) {
        this.route = route;
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

        // Bang-bang (same as BangBangTeleOp): open loop, the encoder is only read for velocity.
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // BRAKE at zero power (what the teleop was originally tuned with), same as BangBangTuner.
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        blocker.setPosition(BLOCKER_BLOCK);

        toShoot = Paths.line(route.start, route.shoot).linear(route.start.heading(), route.shoot.heading());
        toPark  = Paths.line(route.shoot, route.park).linear(route.shoot.heading(), route.park.heading());

        // Short straight line forward from the park pose, same heading, so the robot ends up inside the zone.
        double parkHeading = route.park.heading();
        nudge = new Pose(route.park.x() + PARK_NUDGE_IN * Math.cos(parkHeading),
                route.park.y() + PARK_NUDGE_IN * Math.sin(parkHeading), parkHeading);
        toNudge = Paths.line(route.park, nudge).linear(parkHeading, parkHeading);

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

        // Bang-bang: full power below (target - H), off above (target + H), hold in between.
        if (shooterEnabled) {
            if (shooterVelocity < shooterTarget() - BANG_BANG_HYSTERESIS) {
                shooterOn = true;
            } else if (shooterVelocity > shooterTarget() + BANG_BANG_HYSTERESIS) {
                shooterOn = false;
            }
            shooter.setPower(shooterOn ? BANG_BANG_ON_POWER : BANG_BANG_OFF_POWER);
        }

        switch (state) {
            case DRIVE_TO_SHOOT:
                if (arrived(route.shoot) || stateTimer.seconds() > DRIVE_TIMEOUT_SEC) {
                    follower.hold(route.shoot);
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
                    shooterOn = false;
                    shooter.setPower(0);
                    follower.follow(toPark);
                    setState(State.DRIVE_TO_PARK);
                }
                break;

            case DRIVE_TO_PARK:
                if (arrived(route.park) || stateTimer.seconds() > DRIVE_TIMEOUT_SEC) {
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
        telemetry.addData("Shooter power", "%.1f", shooterEnabled && shooterOn ? BANG_BANG_ON_POWER : BANG_BANG_OFF_POWER);
        telemetry.update();
    }

    @Override
    public void stop() {
        shooter.setPower(0);
        intake.setPower(0);
        transfer.setPower(0);
        // Hand the final pose to the teleop even if the auto ended before it finished.
        BioBuzzPose.savePose(follower.pose());
    }

    /** Flywheel target for this route: the shared fixed velocity plus the route's boost. */
    private double shooterTarget() {
        return SHOOTER_FIXED_VELOCITY + route.velocityBoost;
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
