package org.firstinspires.ftc.teamcode.biobuzz.auto;

import static org.firstinspires.ftc.teamcode.biobuzz.teleop.BioBuzzConfig.*;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.biobuzz.teleop.BioBuzzAlliance;
import org.firstinspires.ftc.teamcode.biobuzz.teleop.BioBuzzPose;
import org.firstinspires.ftc.teamcode.biobuzz.teleop.BioBuzzTurret;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.ArrayList;
import java.util.Map;
import java.util.List;

/**
 * BioBuzz Solo 2, RED side. A state machine (no sleep): the follower updates every loop and each action starts at
 * the right moment.
 *
 *   START : shoot the preload (turret aims at the Red goal, flywheel at PIDF target, then blocker open + feed)
 *   1. intake ON   : intakepreset  -> shootpreset  -> SHOOT
 *   2. flower DOWN : intakeflower1 -> shootflower1 -> SHOOT   (intake ON)
 *   3. flower DOWN : gotoflower -> intakeflower2 -> shootflower2 -> SHOOT   (intake ON)
 *   4. flower UP   : path8 (final move / park); then intake and shooter stop.
 *
 * Reuses the BioBuzz hardware: shooter (hub velocity PIDF, SHOOTER_P/I/D/F at SHOOTER_FIXED_VELOCITY), intake,
 * transfer, blocker, flower servo (FLOWER: FLOWER_ACTIVE = down, FLOWER_REST = up) and {@link BioBuzzTurret}, which
 * tracks the Red goal every loop ((59.47, 65.3) when y <= 70, (57.47, 74.7) when y > 70, from BioBuzzAlliance). Point the
 * turret straight forward before init.
 *
 * The paths come from {@link AutoPathsRed}. If it hasn't been filled in, this auto does nothing and says so.
 * Guards: a timeout per state, and a 30 s auto timer that stops everything shortly before the end.
 */
@Autonomous
public class BioBuzzAutoSolo2Red extends OpMode {

    // ================= TUNE =================
    private static final BioBuzzAlliance ALLIANCE = BioBuzzAlliance.RED;
    /** Start pose (inches, degrees). */
    private static final double START_X = 54.6158, START_Y = 7.3815, START_HEADING_DEG = 180; // matches the start pose of the exported paths

    /** How long the blocker stays open and the feed runs for one shot sequence. */
    private static final double SHOOT_SEC = 1.5;
    /** Shoot anyway if the flywheel / turret aren't ready after this long. */
    private static final double SPINUP_TIMEOUT_SEC = 3.0;
    /** The flywheel must stay within SHOOTER_READY_TOLERANCE of the target this long before feeding. */
    private static final double READY_HOLD_SEC = 0.3;
    /** Turret must be within this many degrees of its target before feeding. */
    private static final double AIM_TOLERANCE_DEG = 3.0;
    /** Give up on a path after this long and go on to the next step (so a stuck robot doesn't freeze the auto). */
    private static final double DRIVE_TIMEOUT_SEC = 6.0;
    /** A new path can't count as finished before this long (the follower can still report the previous path's end). */
    private static final double MIN_DRIVE_SEC = 0.3;
    /** After an intake path ends, keep intaking (standing still) this long to pick up the game piece. */
    private static final double INTAKE_DWELL_SEC = 1.0;      // intakeflower1 and intakeflower2
    /** Drivetrain max power while driving intakepreset (slow so the balls don't scatter). */
    private static final double PRESET_MAX_POWER = 0.3;
    /** Max drive time for intakeflower1 / intakeflower2 (+ INTAKE_DWELL_SEC = about 3 s total per intake). */
    private static final double INTAKE_DRIVE_TIMEOUT_SEC = 2.0;
    private static final double PRESET_DWELL_SEC = 0.3;      // intakepreset (short, it hits the side wall)
    /** Whole auto is 30 s. */
    private static final double AUTO_SEC = 30.0;
    /** Stop everything this long before the 30 s are up. */
    private static final double END_MARGIN_SEC = 0.5;
    // =======================================

    private enum State { PRELOAD_SPINUP, PRELOAD_SHOOT, DRIVE, SHOOT_SPINUP, SHOOT, INTAKE_DWELL, DONE }

    /** One path of the route and what should be on while driving it. */
    private static final class Step {
        final String name;
        final Path path;
        final boolean flowerDown;
        final boolean shootAtEnd;

        Step(String name, Path path, boolean flowerDown, boolean shootAtEnd) {
            this.name = name;
            this.path = path;
            this.flowerDown = flowerDown;
            this.shootAtEnd = shootAtEnd;
        }
    }

    private Follower follower;
    private PowerLimiter powerLimiter;
    private AutoPathsRed paths;
    private DcMotorEx shooter;
    private DcMotor intake, transfer;
    private Servo blocker, flower;
    private BioBuzzTurret turret;

    private final List<Step> steps = new ArrayList<>();
    private int stepIndex = 0;
    private State state = State.DONE;
    private long stateStartNs, autoStartNs, readySinceNs = -1;
    private boolean pathsOk = false;
    private boolean intakeOn = false;
    private boolean flowerIsDown = false;

    @Override
    public void init() {
        // Same follower as Constants.create(), but the drivetrain goes through a power limiter (Pedro 3 has no setMaxPower).
        powerLimiter = new PowerLimiter(new Mecanum(hardwareMap, Constants.drivetrainConfig));
        follower = new Follower(new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
                powerLimiter, new Foresight(Constants.foresightConfig));
        follower.setPose(new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG)));

        shooter  = hardwareMap.get(DcMotorEx.class, SHOOTER);
        intake   = hardwareMap.get(DcMotor.class, INTAKE);
        transfer = hardwareMap.get(DcMotor.class, TRANSFER);
        blocker  = hardwareMap.get(Servo.class, BLOCKER);
        flower   = hardwareMap.get(Servo.class, FLOWER);

        shooter.setDirection(SHOOTER_DIR);
        intake.setDirection(INTAKE_DIR);
        transfer.setDirection(TRANSFER_DIR);

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F));

        blocker.setPosition(BLOCKER_BLOCK);
        flower.setPosition(FLOWER_REST);

        turret = new BioBuzzTurret(hardwareMap, ALLIANCE); // assumes it points straight forward now

        paths = new AutoPathsRed(follower);
        pathsOk = paths.isComplete();
        if (pathsOk) {
            steps.clear();
            steps.add(new Step("intakepreset",  paths.intakepreset,  false, false));
            steps.add(new Step("shootpreset",   paths.shootpreset,   false, true));
            steps.add(new Step("intakeflower1", paths.intakeflower1, true,  false));
            steps.add(new Step("shootflower1",  paths.shootflower1,  true,  true));
            steps.add(new Step("gotoflower",    paths.gotoflower,    true,  false));
            steps.add(new Step("intakeflower2", paths.intakeflower2, true,  false));
            steps.add(new Step("shootflower2",  paths.shootflower2,  true,  true));
            steps.add(new Step("path8",         paths.path8,         false, false));
        }

        telemetry.addData("Status", pathsOk ? "Ready (Red). Turret straight forward!" : "AutoPathsRed NOT FILLED IN: the auto will do nothing");
        telemetry.update();
    }

    @Override
    public void start() {
        autoStartNs = System.nanoTime();
        if (!pathsOk) {
            state = State.DONE;
            return;
        }
        shooter.setVelocity(SHOOTER_FIXED_VELOCITY + PIDF_VELOCITY_OFFSET); // spin up right away
        setState(State.PRELOAD_SPINUP);
    }

    @Override
    public void loop() {
        follower.update();
        long now = System.nanoTime();
        double t = (now - stateStartNs) / 1e9;
        double autoT = (now - autoStartNs) / 1e9;
        Pose pose = follower.pose();

        // Turret: keep tracking the Red goal the whole auto.
        if (pathsOk && state != State.DONE) {
            turret.setSecondaryGoal(ALLIANCE.useSecondaryGoalAt(pose.y()));
            turret.aimAtGoal(pose, 0, 0);
        }

        double target = SHOOTER_FIXED_VELOCITY + PIDF_VELOCITY_OFFSET;
        if (pathsOk && state != State.DONE) shooter.setVelocity(target); // never let the flywheel drop while running
        double velocity = shooter.getVelocity();
        boolean flywheelReady = Math.abs(target - velocity) <= SHOOTER_READY_TOLERANCE;
        boolean aimed = Math.abs(wrap180(turret.getTargetAngle() - turret.getAngle())) <= AIM_TOLERANCE_DEG;

        // 30 s timer: nearly out of time = stop everything.
        if (pathsOk && state != State.DONE && autoT >= AUTO_SEC - END_MARGIN_SEC) {
            finish();
        }

        switch (state) {
            case PRELOAD_SPINUP:
            case SHOOT_SPINUP:
                if (flywheelReady && aimed) {
                    if (readySinceNs < 0) readySinceNs = now;
                } else {
                    readySinceNs = -1;
                }
                boolean readyLongEnough = readySinceNs >= 0 && (now - readySinceNs) / 1e9 >= READY_HOLD_SEC;
                if (readyLongEnough || t >= SPINUP_TIMEOUT_SEC) {
                    openAndFeed();
                    setState(state == State.PRELOAD_SPINUP ? State.PRELOAD_SHOOT : State.SHOOT);
                }
                break;

            case PRELOAD_SHOOT:
                if (t >= SHOOT_SEC) {
                    closeAndStopFeed();
                    startStep(0); // intake ON before the first move
                }
                break;

            case DRIVE:
                // Same end-of-path check the other BioBuzz autos use (atParametricEnd), plus a timeout.
                Step s = steps.get(stepIndex);
                // Intake paths must actually finish driving (so the robot gets all the way in); others use atParametricEnd.
                boolean arrived = s.name.startsWith("intake") ? !follower.isBusy() : follower.atParametricEnd();
                if (t >= MIN_DRIVE_SEC && (arrived || t >= driveTimeout(s))) {
                    if (s.shootAtEnd) {
                        follower.hold(s.path.endPose()); // stand still on the shooting spot
                        setState(State.SHOOT_SPINUP);
                    } else if (s.name.startsWith("intake")) {
                        follower.hold(s.path.endPose()); // stay and keep intaking for a moment
                        setState(State.INTAKE_DWELL);
                    } else {
                        nextStep();
                    }
                }
                break;

            case INTAKE_DWELL:
                if (t >= (steps.get(stepIndex).name.equals("intakepreset") ? PRESET_DWELL_SEC : INTAKE_DWELL_SEC)) nextStep();
                break;

            case SHOOT:
                if (t >= SHOOT_SEC) {
                    closeAndStopFeed();
                    nextStep();
                }
                break;

            case DONE:
                break;
        }

        telemetry.addData("State", "%s  (%.1f s)", state, t);
        telemetry.addData("Step", pathsOk && stepIndex < steps.size() ? (stepIndex + 1) + "/" + steps.size() + " " + steps.get(stepIndex).name : "-");
        telemetry.addData("Path", follower.isBusy() ? "driving" : "idle / holding");
        telemetry.addData("Auto time", "%.1f / %.0f s", autoT, AUTO_SEC);
        telemetry.addData("X / Y / heading", "%.1f / %.1f / %.0f", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Intake", intakeOn ? "ON" : "off");
        telemetry.addData("Flower", flowerIsDown ? "DOWN" : "UP");
        telemetry.addData("Goal", turret.isTargetingSecondaryGoal() ? "SECONDARY" : "PRIMARY");
        telemetry.addData("Turret angle / target", "%.1f / %.1f %s", turret.getAngle(), turret.getTargetAngle(), aimed ? "AIMED" : "");
        telemetry.addData("Flywheel", "%.0f / %.0f %s", velocity, target, flywheelReady ? "READY" : "");
        if (!pathsOk) telemetry.addData("!!!", "AutoPathsRed not filled in (see AutoPathsRed.java)");
        telemetry.update();
    }

    @Override
    public void stop() {
        stopAll();
        // Hand the final pose and turret position to the teleop.
        BioBuzzPose.save(follower.pose(), turret.getRawTicks());    }

    // ================= STEPS =================
    /** Start driving step i: flower and intake are set BEFORE the robot moves. */
    private void startStep(int i) {
        stepIndex = i;
        Step s = steps.get(i);
        boolean lastStep = i == steps.size() - 1;
        setFlower(s.flowerDown);
        if (lastStep) {
            setIntake(false); // path8: flower up, no need to intake any more
        } else {
            setIntake(true);
        }
        powerLimiter.scale = s.name.equals("intakepreset") ? PRESET_MAX_POWER : 1.0;
        follower.follow(s.path);
        setState(State.DRIVE);
    }

    private static double driveTimeout(Step s) {
        return s.name.startsWith("intakeflower") ? INTAKE_DRIVE_TIMEOUT_SEC : DRIVE_TIMEOUT_SEC;
    }

    private void nextStep() {
        if (stepIndex + 1 >= steps.size()) {
            finish();
        } else {
            startStep(stepIndex + 1);
        }
    }

    private void finish() {
        if (pathsOk && !steps.isEmpty()) follower.hold(follower.pose()); // stay where we ended
        intake.setPower(0);
        transfer.setPower(0);
        blocker.setPosition(BLOCKER_BLOCK);
        intakeOn = false;
        setState(State.DONE); // flywheel stays on until the OpMode stops
    }

    // ================= HELPERS =================
    private void setState(State s) {
        state = s;
        stateStartNs = System.nanoTime();
        readySinceNs = -1;
    }

    private void setIntake(boolean on) {
        intakeOn = on;
        intake.setPower(on ? PIDF_AUTO_FEED_POWER : 0);
    }

    private void setFlower(boolean down) {
        flowerIsDown = down;
        flower.setPosition(down ? FLOWER_ACTIVE : FLOWER_REST);
    }

    /** Shoot: blocker open, intake + transfer feed. */
    private void openAndFeed() {
        blocker.setPosition(BLOCKER_OPEN);
        intake.setPower(PIDF_AUTO_FEED_POWER);
        transfer.setPower(PIDF_AUTO_FEED_POWER);
    }

    /** Close the blocker and stop the transfer; the intake goes back to whatever the step wants. */
    private void closeAndStopFeed() {
        blocker.setPosition(BLOCKER_BLOCK);
        transfer.setPower(0);
        intake.setPower(intakeOn ? PIDF_AUTO_FEED_POWER : 0);
    }

    private void stopAll() {
        shooter.setVelocity(0);
        intake.setPower(0);
        transfer.setPower(0);
        blocker.setPosition(BLOCKER_BLOCK);
        intakeOn = false;
        turret.stop();
    }

    private static double wrap180(double a) {
        a = a % 360.0;
        if (a > 180) a -= 360;
        if (a <= -180) a += 360;
        return a;
    }

    /** Wraps the drivetrain and scales the follower's drive powers (not manual) by {@link #scale}. */
    private static final class PowerLimiter implements Drivetrain {
        private final Drivetrain base;
        volatile double scale = 1.0;

        PowerLimiter(Drivetrain base) { this.base = base; }

        @Override public void drive(DrivePowers p, boolean manual) {
            base.drive(manual ? p : new DrivePowers(p.forward() * scale, p.strafe() * scale, p.turn() * scale), manual);
        }
        @Override public double maxScaling(DrivePowers current, DrivePowers delta) { return base.maxScaling(current, delta); }
        @Override public void stop() { base.stop(); }
        @Override public void stop(boolean brake) { base.stop(brake); }
        @Override public Map<String, Object> debug() { return base.debug(); }
        @Override public double interpolateVelocity(double xRadius, double yRadius, double theta) {
            return base.interpolateVelocity(xRadius, yRadius, theta);
        }
    }
}
