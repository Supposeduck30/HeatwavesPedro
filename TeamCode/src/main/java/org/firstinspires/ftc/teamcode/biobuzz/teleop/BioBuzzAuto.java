package org.firstinspires.ftc.teamcode.biobuzz.teleop;

import static org.firstinspires.ftc.teamcode.biobuzz.teleop.BioBuzzConfig.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * BioBuzz auto:
 *   1. Drive path1 (curve) to the shooting spot, spinning the flywheel up on the way.
 *   2. Wait until the flywheel is READY, then open the blocker and feed to shoot the preloads.
 *   3. Close the blocker and put the flower down.
 *   4. Turn on intake + transfer, then drive path2 forward to pick up balls.
 *   5. Stop everything.
 *
 * Shooter uses PIDF (RUN_USING_ENCODER + setVelocity) with SHOOTER_P/I/D/F from BioBuzzConfig.
 */
@Autonomous(name = "BioBuzz Auto", group = "BioBuzz")
public class BioBuzzAuto extends OpMode {

    // ================= TUNE =================
    /** How long to feed while shooting the preloads. */
    private static final double SHOOT_SEC = 2.5;
    /** Shoot anyway if the flywheel is not READY after this long. */
    private static final double SPINUP_TIMEOUT_SEC = 2.0;
    /** Flywheel must stay READY this long before feeding. */
    private static final double READY_HOLD_SEC = 0.3;
    /** Time for the flower servo to move down before driving. */
    private static final double FLOWER_SEC = 0.5;
    /** Which flower position is "down". Swap to FLOWER_REST if it moves the wrong way. */
    private static final double FLOWER_DOWN = FLOWER_ACTIVE;

    // ================= PATHS (from your path file) =================
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start          = poseFactory.of(53.5901, 7.5786, 90);
    private final Pose point1         = poseFactory.of(47.0565, 118.8103, 90);
    private final Pose point1Control1 = poseFactory.of(74.1178, 69.6332, 0);
    private final Pose point2Start    = poseFactory.of(47.0565, 118.8103, 90);
    private final Pose point2         = poseFactory.of(46.7315, 128.812, 90);

    private Path path1() {
        return com.pedropathing.api.Paths.curve(start, point1Control1, point1).constant(point1);
    }

    private Path path2() {
        return com.pedropathing.api.Paths.line(point2Start, point2).linear(point2Start, point2);
    }

    // ================= STATE =================
    private enum State { DRIVE_TO_SHOOT, SPIN_UP, SHOOT, FLOWER_DOWN, DRIVE_FORWARD, DONE }

    private State state = State.DRIVE_TO_SHOOT;
    private long stateStartNs;
    private long readySinceNs = -1;

    private Follower follower;
    private DcMotorEx shooter;
    private DcMotor intake, transfer;
    private Servo blocker, flower;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(start);

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
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F));

        blocker.setPosition(BLOCKER_BLOCK);
        flower.setPosition(FLOWER_REST);

        telemetry.addLine("BioBuzz Auto ready.");
        telemetry.update();
    }

    @Override
    public void start() {
        shooter.setVelocity(SHOOTER_FIXED_VELOCITY); // spin up while driving
        followPath(path1());
        setState(State.DRIVE_TO_SHOOT);
    }

    @Override
    public void loop() {
        follower.update();
        long now = System.nanoTime();
        double t = (now - stateStartNs) / 1e9;
        double velocity = shooter.getVelocity();
        boolean ready = Math.abs(SHOOTER_FIXED_VELOCITY - velocity) <= SHOOTER_READY_TOLERANCE;

        switch (state) {
            case DRIVE_TO_SHOOT:
                if (!pathBusy()) setState(State.SPIN_UP);
                break;

            case SPIN_UP:
                if (ready) {
                    if (readySinceNs < 0) readySinceNs = now;
                } else {
                    readySinceNs = -1;
                }
                boolean readyLongEnough = readySinceNs >= 0 && (now - readySinceNs) / 1e9 >= READY_HOLD_SEC;
                if (readyLongEnough || t >= SPINUP_TIMEOUT_SEC) {
                    blocker.setPosition(BLOCKER_OPEN);
                    intake.setPower(PRIMARY_FEED_POWER);
                    transfer.setPower(PRIMARY_FEED_POWER);
                    setState(State.SHOOT);
                }
                break;

            case SHOOT:
                if (t >= SHOOT_SEC) {
                    blocker.setPosition(BLOCKER_BLOCK);
                    intake.setPower(0);
                    transfer.setPower(0);
                    flower.setPosition(FLOWER_DOWN);
                    setState(State.FLOWER_DOWN);
                }
                break;

            case FLOWER_DOWN:
                if (t >= FLOWER_SEC) {
                    // Intake + transfer on BEFORE moving forward
                    intake.setPower(INTAKE_IN_POWER);
                    transfer.setPower(TRANSFER_IN_POWER);
                    followPath(path2());
                    setState(State.DRIVE_FORWARD);
                }
                break;

            case DRIVE_FORWARD:
                if (!pathBusy()) {
                    stopMechanisms();
                    setState(State.DONE);
                }
                break;

            case DONE:
                break;
        }

        Pose pose = follower.pose();
        telemetry.addData("State", "%s  (%.1f s)", state, t);
        telemetry.addData("X / Y / heading", "%.1f / %.1f / %.0f", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Flywheel", "%.0f / %.0f  %s", velocity, SHOOTER_FIXED_VELOCITY, ready ? "READY" : "");
        telemetry.update();
    }

    @Override
    public void stop() {
        stopMechanisms();
        // Hand the final pose to the teleop (BioBuzzPose) so it starts where this auto ended.
        BioBuzzPose.savePose(follower.pose());
    }

    // ================= HELPERS =================

    /** Pedro call to start a path. If this does not compile, check how TemplateAuto starts a path. */
    private void followPath(Path path) {
        follower.follow(path);
    }

    /** Pedro call for "still driving". If this does not compile, check TemplateAuto. */
    private boolean pathBusy() {
        return follower.isBusy();
    }

    private void setState(State s) {
        state = s;
        stateStartNs = System.nanoTime();
        readySinceNs = -1;
    }

    private void stopMechanisms() {
        shooter.setVelocity(0);
        intake.setPower(0);
        transfer.setPower(0);
        blocker.setPosition(BLOCKER_BLOCK);
    }
}
