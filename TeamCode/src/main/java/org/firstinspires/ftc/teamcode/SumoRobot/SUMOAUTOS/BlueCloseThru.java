package org.firstinspires.ftc.teamcode.SumoRobot.SUMOAUTOS;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
@Configurable
public class BlueCloseThru extends OpMode {
    private PanelsTelemetry panelsTelemetry;

    // Mechanisms
    private DcMotorEx shooter1;
    private DcMotorEx shooter2;
    private DcMotor intake;
    private Servo blocker; // Replaces kicker — controls ball throughput

    // Blocker positions (mirrored from TeleOp constants)
    private static final double BLOCKER_BLOCK = 0.15; // Holds balls back
    private static final double BLOCKER_OPEN  = 0.25; // Lets balls through

    public double shootVelocity = 1460;
    public double idleVelocity  = 1000;

    // How long blocker stays open per ball (tuned for throughput robot)
    private static final double FIRE_TIME = 0.41; // seconds
    // How long to spin up shooters before opening blocker
    private static final double SPIN_UP_TIME = 1.0; // seconds

    // Software
    public Follower follower;
    private Timer pathTimer, opModeTimer;

    // Shooting state
    private int ballsShot = 0;
    private boolean shotsFired = false;

    // States
    public enum PathState {
        DRIVE_PATH1,
        SHOOT_PATH_WAIT,
        SHOOT_PATH1,
        SPIN_UP,
        SHOOT,

        DRIVE_PATH2,
        DRIVE_PATH3,
        INTAKE_ON_PATH3,
        DRIVE_PATH4,
        DRIVE_PATH5,
        SHOOT_PATH5,
        SPIN_UP2,
        SHOOT2,

        DRIVE_PATH6,
        DRIVE_PATH7,
        INTAKE_ON_PATH7,
        DRIVE_PATH8,
        SHOOT_PATH8,
        SPIN_UP3,
        SHOOT3,

        DRIVE_PATH9,
        DRIVE_PATH10,
        DRIVE_PATH11,
        SHOOT_PATH11,
        SPIN_UP4,
        SHOOT4,

        DRIVE_PATH12,
        DONE,
    }

    PathState pathState;

    // Poses
    private final Pose startPose = new Pose(24.6, 126.7, Math.toRadians(135));

    private final Pose pose1End  = new Pose(55.8, 89.9,  Math.toRadians(138));
    private final Pose pose2End  = new Pose(48.3, 61.6,  Math.toRadians(180));
    private final Pose pose3End  = new Pose(15.8, 61.6,  Math.toRadians(180));

    private final Pose bezierIntakeToGate = new Pose(34.2, 67.1, Math.toRadians(180));

    private final Pose pose4End  = new Pose(16.2,  71.5, Math.toRadians(180));

    private final Pose bezierGateToShoot  = new Pose(61.4, 61.3, Math.toRadians(180));

    private final Pose pose5End  = new Pose(55.8, 89.9,  Math.toRadians(138));
    private final Pose pose6End  = new Pose(48,   83.5,  Math.toRadians(180));
    private final Pose pose7End  = new Pose(14.9, 83.5,  Math.toRadians(180));
    private final Pose pose8End  = new Pose(55.8, 89.9,  Math.toRadians(138));
    private final Pose pose9End  = new Pose(47.4, 36.5,  Math.toRadians(180));
    private final Pose pose10End = new Pose(14.9, 36.5,  Math.toRadians(180));
    private final Pose pose11End = new Pose(55.8, 89.9,  Math.toRadians(138));
    private final Pose pose12End = new Pose(57.3, 110.2, Math.toRadians(270));

    private PathChain path1, path2, path3, path4, path5, path6, path7,
            path8, path9, path10, path11, path12;

    public void buildPaths() {
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, pose1End))
                .setConstantHeadingInterpolation(pose1End.getHeading())
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierLine(pose1End, pose2End))
                .setLinearHeadingInterpolation(pose1End.getHeading(), pose2End.getHeading())
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierLine(pose2End, pose3End))
                .setLinearHeadingInterpolation(pose2End.getHeading(), pose3End.getHeading())
                .build();

        path4 = follower.pathBuilder()
                .addPath(new BezierCurve(pose3End, bezierIntakeToGate, pose4End))
                .setLinearHeadingInterpolation(pose3End.getHeading(), pose4End.getHeading())
                .build();

        path5 = follower.pathBuilder()
                .addPath(new BezierCurve(pose4End, bezierGateToShoot, pose5End))
                .setConstantHeadingInterpolation(pose5End.getHeading())
                .build();

        path6 = follower.pathBuilder()
                .addPath(new BezierLine(pose5End, pose6End))
                .setLinearHeadingInterpolation(pose5End.getHeading(), pose6End.getHeading())
                .build();

        path7 = follower.pathBuilder()
                .addPath(new BezierLine(pose6End, pose7End))
                .setLinearHeadingInterpolation(pose6End.getHeading(), pose7End.getHeading())
                .build();

        path8 = follower.pathBuilder()
                .addPath(new BezierLine(pose7End, pose8End))
                .setConstantHeadingInterpolation(pose8End.getHeading())
                .build();

        path9 = follower.pathBuilder()
                .addPath(new BezierLine(pose8End, pose9End))
                .setLinearHeadingInterpolation(pose8End.getHeading(), pose9End.getHeading())
                .build();

        path10 = follower.pathBuilder()
                .addPath(new BezierLine(pose9End, pose10End))
                .setLinearHeadingInterpolation(pose9End.getHeading(), pose10End.getHeading())
                .build();

        path11 = follower.pathBuilder()
                .addPath(new BezierLine(pose10End, pose11End))
                .setConstantHeadingInterpolation(pose11End.getHeading())
                .build();

        path12 = follower.pathBuilder()
                .addPath(new BezierLine(pose11End, pose12End))
                .setLinearHeadingInterpolation(pose11End.getHeading(), pose12End.getHeading())
                .build();
    }

    // ================= BLOCKER SHOOTING LOGIC =================
    /**
     * Throughput shooting: blocker opens for FIRE_TIME seconds to let one ball through,
     * then closes. Call this every loop while in a SHOOT state.
     * Returns true when all 3 balls have been fired.
     */
    private boolean updateShoot() {
        if (ballsShot >= 3) return true;

        if (pathTimer.getElapsedTimeSeconds() >= FIRE_TIME) {
            // Close blocker between shots (brief pause handled by timer reset in next call)
            blocker.setPosition(BLOCKER_BLOCK);
            ballsShot++;

            if (ballsShot >= 3) {
                // All done — keep blocker closed
                return true;
            } else {
                // Reset timer to time the next shot's open window
                pathTimer.resetTimer();
                blocker.setPosition(BLOCKER_OPEN);
            }
        }
        return false;
    }

    // ================= STATE MACHINE =================
    public void statePathUpdate() {
        switch (pathState) {

            // ---- SHOT GROUP 1 ----
            case DRIVE_PATH1:
                follower.followPath(path1, true);
                setPathState(PathState.SHOOT_PATH_WAIT);
                break;

            case SHOOT_PATH_WAIT:
                if (!follower.isBusy() && Math.toDegrees(follower.getHeadingError()) < 3) {
                    setPathState(PathState.SHOOT_PATH1);
                }
                break;

            case SHOOT_PATH1:
                // Spin up shooters and start spin-up timer
                shooter1.setVelocity(shootVelocity);
                shooter2.setVelocity(shootVelocity);
                setPathState(PathState.SPIN_UP);
                break;

            case SPIN_UP:
                if (pathTimer.getElapsedTimeSeconds() > SPIN_UP_TIME) {
                    // Open blocker for first shot and start fire timer
                    blocker.setPosition(BLOCKER_OPEN);
                    pathTimer.resetTimer();
                    setPathState(PathState.SHOOT);
                }
                break;

            case SHOOT:
                if (updateShoot()) {
                    ballsShot = 0;
                    shooter1.setVelocity(idleVelocity);
                    shooter2.setVelocity(idleVelocity);
                    follower.followPath(path2, true);
                    setPathState(PathState.DRIVE_PATH2);
                }
                break;

            // ---- INTAKE RUN 1 ----
            case DRIVE_PATH2:
                if (!follower.isBusy()) {
                    follower.followPath(path3, true);
                    setPathState(PathState.DRIVE_PATH3);
                }
                break;

            case DRIVE_PATH3:
                if (!follower.isBusy()) {
                    intake.setPower(1);
                    setPathState(PathState.INTAKE_ON_PATH3);
                }
                break;

            case INTAKE_ON_PATH3:
                follower.followPath(path4, true);
                setPathState(PathState.DRIVE_PATH4);
                break;

            case DRIVE_PATH4:
                if (!follower.isBusy()) {
                    follower.followPath(path5, true);
                    setPathState(PathState.DRIVE_PATH5);
                }
                break;

            // ---- SHOT GROUP 2 ----
            case DRIVE_PATH5:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_PATH5);
                }
                break;

            case SHOOT_PATH5:
                shooter1.setVelocity(shootVelocity);
                shooter2.setVelocity(shootVelocity);
                setPathState(PathState.SPIN_UP2);
                break;

            case SPIN_UP2:
                if (pathTimer.getElapsedTimeSeconds() > SPIN_UP_TIME) {
                    blocker.setPosition(BLOCKER_OPEN);
                    pathTimer.resetTimer();
                    setPathState(PathState.SHOOT2);
                }
                break;

            case SHOOT2:
                if (updateShoot()) {
                    ballsShot = 0;
                    shooter1.setVelocity(idleVelocity);
                    shooter2.setVelocity(idleVelocity);
                    follower.followPath(path6, true);
                    setPathState(PathState.DRIVE_PATH6);
                }
                break;

            // ---- INTAKE RUN 2 ----
            case DRIVE_PATH6:
                if (!follower.isBusy()) {
                    follower.followPath(path7, true);
                    setPathState(PathState.DRIVE_PATH7);
                }
                break;

            case DRIVE_PATH7:
                if (!follower.isBusy()) {
                    intake.setPower(1);
                    setPathState(PathState.INTAKE_ON_PATH7);
                }
                break;

            case INTAKE_ON_PATH7:
                follower.followPath(path8, true);
                setPathState(PathState.DRIVE_PATH8);
                break;

            // ---- SHOT GROUP 3 ----
            case DRIVE_PATH8:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_PATH8);
                }
                break;

            case SHOOT_PATH8:
                shooter1.setVelocity(shootVelocity);
                shooter2.setVelocity(shootVelocity);
                setPathState(PathState.SPIN_UP3);
                break;

            case SPIN_UP3:
                if (pathTimer.getElapsedTimeSeconds() > SPIN_UP_TIME) {
                    blocker.setPosition(BLOCKER_OPEN);
                    pathTimer.resetTimer();
                    setPathState(PathState.SHOOT3);
                }
                break;

            case SHOOT3:
                if (updateShoot()) {
                    ballsShot = 0;
                    shooter1.setVelocity(idleVelocity);
                    shooter2.setVelocity(idleVelocity);
                    follower.followPath(path9, true);
                    setPathState(PathState.DRIVE_PATH9);
                }
                break;

            // ---- INTAKE RUN 3 ----
            case DRIVE_PATH9:
                if (!follower.isBusy()) {
                    follower.followPath(path10, true);
                    setPathState(PathState.DRIVE_PATH10);
                }
                break;

            case DRIVE_PATH10:
                if (!follower.isBusy()) {
                    follower.followPath(path11, true);
                    setPathState(PathState.DRIVE_PATH11);
                }
                break;

            // ---- SHOT GROUP 4 ----
            case DRIVE_PATH11:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_PATH11);
                }
                break;

            case SHOOT_PATH11:
                shooter1.setVelocity(shootVelocity);
                shooter2.setVelocity(shootVelocity);
                setPathState(PathState.SPIN_UP4);
                break;

            case SPIN_UP4:
                if (pathTimer.getElapsedTimeSeconds() > SPIN_UP_TIME) {
                    blocker.setPosition(BLOCKER_OPEN);
                    pathTimer.resetTimer();
                    setPathState(PathState.SHOOT4);
                }
                break;

            case SHOOT4:
                if (updateShoot()) {
                    ballsShot = 0;
                    shooter1.setVelocity(0);
                    shooter2.setVelocity(0);
                    intake.setPower(0);
                    follower.followPath(path12, true);
                    setPathState(PathState.DRIVE_PATH12);
                }
                break;

            // ---- PARK ----
            case DRIVE_PATH12:
                if (!follower.isBusy()) {
                    setPathState(PathState.DONE);
                }
                break;

            case DONE:
                //panelsTelemetry.debug("Status", "Done all Paths");
                break;

            default:
                //panelsTelemetry.debug("Status", "DONE :)");
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
        shotsFired = false;
    }

    @Override
    public void init() {
        //panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        pathState = PathState.DRIVE_PATH1;
        pathTimer   = new Timer();
        opModeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);

        shooter1 = hardwareMap.get(DcMotorEx.class, "Shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "Shooter2");
        intake   = hardwareMap.get(DcMotor.class,   "Intake");
        blocker  = hardwareMap.get(Servo.class,      "Kicker"); // Same hardware name, new role

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pidf = new PIDFCoefficients(100, 0, 0, 15);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        // Start with blocker closed
        blocker.setPosition(BLOCKER_BLOCK);

        buildPaths();
        follower.setPose(startPose);


    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);

        shooter1.setVelocity(idleVelocity);
        shooter2.setVelocity(idleVelocity);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();


    }

    @Override
    public void stop() {
        Pose finalPose = follower.getPose();
        org.firstinspires.ftc.teamcode.pedroPathing.mechanisms.PedroPose.saveCurrentPose(finalPose);
        super.stop();
    }
}