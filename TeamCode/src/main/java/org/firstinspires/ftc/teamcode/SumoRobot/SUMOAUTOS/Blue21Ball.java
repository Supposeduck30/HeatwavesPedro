package org.firstinspires.ftc.teamcode.SumoRobot.SUMOAUTOS;

import static java.lang.Thread.sleep;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
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
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
@Configurable
public class Blue21Ball extends OpMode {
    private PanelsTelemetry panelsTelemetry;

    // Mechanisms (using your team's hardware)
    private DcMotorEx shooter1;
    private DcMotorEx shooter2;
    private DcMotor intake;
    private Servo kicker;

    // Shooting configuration
    public double shootVelocity = 1460;
    public double idleVelocity = 1000;
    public double firstSpinUpTime = 0.55;

    // Software
    public Follower follower;
    private Timer pathTimer, opModeTimer, shootTimer;

    // Shooting state
    private int shotsRemaining = 0;
    private boolean isKicking = false;
    private boolean isFirstShot = true;
    private int ballsShot = 0;

    // Intake power constants
    private static final double INTAKE_POWER_FULL = 1.0;
    private static final double INTAKE_POWER_OFF = 0.0;

    // States
    public enum PathState {
        DRIVE_TO_SHOOT_PRELOAD,
        SHOOT_PRELOAD,
        DRIVE_TO_MIDDLE_SET,
        DRIVE_MIDDLE_FORWARD,
        DRIVE_TO_OPEN_GATE,
        DRIVE_TO_SHOOT_MIDDLE,
        SHOOT_MIDDLE,

        // Gate cycle 1
        DRIVE_TO_GATE_1,
        INTAKE_AT_GATE_1,
        DRIVE_TO_SHOOT_GATE_1,
        SHOOT_GATE_1,

        // Gate cycle 2
        DRIVE_TO_GATE_2,
        INTAKE_AT_GATE_2,
        DRIVE_TO_SHOOT_GATE_2,
        SHOOT_GATE_2,

        // Gate cycle 3
        DRIVE_TO_GATE_3,
        INTAKE_AT_GATE_3,
        DRIVE_TO_SHOOT_GATE_3,
        SHOOT_GATE_3,

        // Gate cycle 4
        DRIVE_TO_GATE_4,
        INTAKE_AT_GATE_4,
        DRIVE_TO_SHOOT_GATE_4,
        SHOOT_GATE_4,

        // Top set
        DRIVE_TO_TOP_SET,
        DRIVE_TOP_FORWARD,
        DRIVE_TO_SHOOT_TOP,
        SHOOT_TOP,

        DONE
    }

    PathState pathState;

    // Poses from the original left document
    private final Pose STARTING_POSE = new Pose(24.6, 126.7, Math.toRadians(135));
    private final Pose SHOOT_PRELOAD_POSE = new Pose(58.5, 85, Math.toRadians(140));

    // Middle Set
    private final Pose MIDDLE_SET_INTAKE_POSE = new Pose(53.11, 58, Math.toRadians(180));
    private final Pose MIDDLE_SET_FORWARD_POSE = new Pose(17, 55, Math.toRadians(180));

    // Gate opening pose
    private final Pose GATE_OPEN_POSE = new Pose(21, 67.5, Math.toRadians(180));
    private final Pose GATE_OPEN_CONTROL = new Pose(30, 64, Math.toRadians(180));
    private final Pose MIDDLE_SET_SHOOT_POSE = new Pose(57.5, 85.5, Math.toRadians(140));

    // Gate cycle 1
    private final Pose GATE_CYCLE_1_INTAKE_POSE = new Pose(17, 59, Math.toRadians(154));
    private final Pose GATE_CYCLE_1_SHOOT_POSE = new Pose(57.5, 85.5, Math.toRadians(140));

    // Gate cycle 2
    private final Pose GATE_CYCLE_2_INTAKE_POSE = new Pose(17, 59, Math.toRadians(154));
    private final Pose GATE_CYCLE_2_SHOOT_POSE = new Pose(58.5, 88, Math.toRadians(140));

    // Gate cycle 3
    private final Pose GATE_CYCLE_3_INTAKE_POSE = new Pose(17, 59, Math.toRadians(154));
    private final Pose GATE_CYCLE_3_SHOOT_POSE = new Pose(59.5, 88.5, Math.toRadians(140));

    // Gate cycle 4
    private final Pose GATE_CYCLE_4_INTAKE_POSE = new Pose(17, 59, Math.toRadians(154));
    private final Pose GATE_CYCLE_4_SHOOT_POSE = new Pose(60.5, 89, Math.toRadians(140));

    // Top set
    private final Pose TOP_SET_INTAKE_POSE = new Pose(48, 82, Math.toRadians(180));
    private final Pose TOP_SET_FORWARD_POSE = new Pose(20, 82, Math.toRadians(180));
    private final Pose TOP_SET_SHOOT_POSE = new Pose(61.5, 85.5, Math.toRadians(138));

    // Path chains
    private PathChain driveToShootPreload, driveToMiddleSet, driveMiddleForward, driveToOpenGate;
    private PathChain driveToShootMiddle;
    private PathChain driveToGate1, driveToShootGate1;
    private PathChain driveToGate2, driveToShootGate2;
    private PathChain driveToGate3, driveToShootGate3;
    private PathChain driveToGate4, driveToShootGate4;
    private PathChain driveToTopSet, driveTopForward, driveToShootTop;

    public void buildPaths() {
        driveToShootPreload = follower.pathBuilder()
                .addPath(new BezierLine(STARTING_POSE, SHOOT_PRELOAD_POSE))
                .setConstantHeadingInterpolation(SHOOT_PRELOAD_POSE.getHeading())
                .setTValueConstraint(0.95)
                .setTimeoutConstraint(150)
                .build();

        driveToMiddleSet = follower.pathBuilder()
                .addPath(new BezierLine(SHOOT_PRELOAD_POSE, MIDDLE_SET_INTAKE_POSE))
                .setLinearHeadingInterpolation(SHOOT_PRELOAD_POSE.getHeading(), MIDDLE_SET_INTAKE_POSE.getHeading())
                .setTValueConstraint(0.95)
                .setTimeoutConstraint(50)
                .build();

        driveMiddleForward = follower.pathBuilder()
                .addPath(new BezierLine(MIDDLE_SET_INTAKE_POSE, MIDDLE_SET_FORWARD_POSE))
                .setLinearHeadingInterpolation(MIDDLE_SET_INTAKE_POSE.getHeading(), MIDDLE_SET_FORWARD_POSE.getHeading())
                .setTValueConstraint(0.95)
                .setTimeoutConstraint(50)
                .build();

        driveToOpenGate = follower.pathBuilder()
                .addPath(new BezierCurve(MIDDLE_SET_FORWARD_POSE, GATE_OPEN_CONTROL, GATE_OPEN_POSE))
                .setLinearHeadingInterpolation(MIDDLE_SET_FORWARD_POSE.getHeading(), GATE_OPEN_POSE.getHeading(), 0.8)
                .setTValueConstraint(0.90)
                .setTimeoutConstraint(100)
                .build();

        driveToShootMiddle = follower.pathBuilder()
                .addPath(new BezierCurve(GATE_OPEN_POSE, GATE_OPEN_CONTROL, MIDDLE_SET_SHOOT_POSE))
                .setLinearHeadingInterpolation(GATE_OPEN_POSE.getHeading(), MIDDLE_SET_SHOOT_POSE.getHeading())
                .setTValueConstraint(0.95)
                .setTimeoutConstraint(150)
                .build();

        // Gate cycle 1
        driveToGate1 = follower.pathBuilder()
                .addPath(new BezierLine(MIDDLE_SET_SHOOT_POSE, GATE_CYCLE_1_INTAKE_POSE))
                .setLinearHeadingInterpolation(MIDDLE_SET_SHOOT_POSE.getHeading(), GATE_CYCLE_1_INTAKE_POSE.getHeading())
                .build();

        driveToShootGate1 = follower.pathBuilder()
                .addPath(new BezierLine(GATE_CYCLE_1_INTAKE_POSE, GATE_CYCLE_1_SHOOT_POSE))
                .setLinearHeadingInterpolation(GATE_CYCLE_1_INTAKE_POSE.getHeading(), GATE_CYCLE_1_SHOOT_POSE.getHeading())
                .setTValueConstraint(0.95)
                .setTimeoutConstraint(150)
                .build();

        // Gate cycle 2
        driveToGate2 = follower.pathBuilder()
                .addPath(new BezierLine(GATE_CYCLE_1_SHOOT_POSE, GATE_CYCLE_2_INTAKE_POSE))
                .setLinearHeadingInterpolation(GATE_CYCLE_1_SHOOT_POSE.getHeading(), GATE_CYCLE_2_INTAKE_POSE.getHeading())
                .build();

        driveToShootGate2 = follower.pathBuilder()
                .addPath(new BezierLine(GATE_CYCLE_2_INTAKE_POSE, GATE_CYCLE_2_SHOOT_POSE))
                .setLinearHeadingInterpolation(GATE_CYCLE_2_INTAKE_POSE.getHeading(), GATE_CYCLE_2_SHOOT_POSE.getHeading())
                .setTValueConstraint(0.95)
                .setTimeoutConstraint(150)
                .build();

        // Gate cycle 3
        driveToGate3 = follower.pathBuilder()
                .addPath(new BezierLine(GATE_CYCLE_2_SHOOT_POSE, GATE_CYCLE_3_INTAKE_POSE))
                .setLinearHeadingInterpolation(GATE_CYCLE_2_SHOOT_POSE.getHeading(), GATE_CYCLE_3_INTAKE_POSE.getHeading())
                .build();

        driveToShootGate3 = follower.pathBuilder()
                .addPath(new BezierLine(GATE_CYCLE_3_INTAKE_POSE, GATE_CYCLE_3_SHOOT_POSE))
                .setLinearHeadingInterpolation(GATE_CYCLE_3_INTAKE_POSE.getHeading(), GATE_CYCLE_3_SHOOT_POSE.getHeading())
                .setTValueConstraint(0.95)
                .setTimeoutConstraint(150)
                .build();

        // Gate cycle 4
        driveToGate4 = follower.pathBuilder()
                .addPath(new BezierLine(GATE_CYCLE_3_SHOOT_POSE, GATE_CYCLE_4_INTAKE_POSE))
                .setLinearHeadingInterpolation(GATE_CYCLE_3_SHOOT_POSE.getHeading(), GATE_CYCLE_4_INTAKE_POSE.getHeading())
                .build();

        driveToShootGate4 = follower.pathBuilder()
                .addPath(new BezierLine(GATE_CYCLE_4_INTAKE_POSE, GATE_CYCLE_4_SHOOT_POSE))
                .setLinearHeadingInterpolation(GATE_CYCLE_4_INTAKE_POSE.getHeading(), GATE_CYCLE_4_SHOOT_POSE.getHeading())
                .setTValueConstraint(0.95)
                .setTimeoutConstraint(150)
                .build();

        // Top set
        driveToTopSet = follower.pathBuilder()
                .addPath(new BezierLine(GATE_CYCLE_4_SHOOT_POSE, TOP_SET_INTAKE_POSE))
                .setLinearHeadingInterpolation(GATE_CYCLE_4_SHOOT_POSE.getHeading(), TOP_SET_INTAKE_POSE.getHeading())
                .setTValueConstraint(0.9)
                .setTimeoutConstraint(50)
                .build();

        driveTopForward = follower.pathBuilder()
                .addPath(new BezierLine(TOP_SET_INTAKE_POSE, TOP_SET_FORWARD_POSE))
                .setLinearHeadingInterpolation(TOP_SET_INTAKE_POSE.getHeading(), TOP_SET_FORWARD_POSE.getHeading())
                .setTValueConstraint(0.9)
                .setTimeoutConstraint(50)
                .build();

        driveToShootTop = follower.pathBuilder()
                .addPath(new BezierLine(TOP_SET_FORWARD_POSE, TOP_SET_SHOOT_POSE))
                .setLinearHeadingInterpolation(TOP_SET_FORWARD_POSE.getHeading(), TOP_SET_SHOOT_POSE.getHeading())
                .setTValueConstraint(0.9)
                .setTimeoutConstraint(150)
                .build();
    }

    public void statePathUpdate() throws InterruptedException {
        boolean followerBusy = follower.isBusy();

        switch (pathState) {
            case DRIVE_TO_SHOOT_PRELOAD:
                follower.followPath(driveToShootPreload, true);
                setPathState(PathState.SHOOT_PRELOAD);
                break;

            case SHOOT_PRELOAD:
                handleShootingState(PathState.DRIVE_TO_MIDDLE_SET, driveToMiddleSet);
                break;

            case DRIVE_TO_MIDDLE_SET:
                setIntakePower(INTAKE_POWER_FULL);
                if (!followerBusy) {
                    follower.followPath(driveMiddleForward, true);
                    setPathState(PathState.DRIVE_MIDDLE_FORWARD);
                }
                break;

            case DRIVE_MIDDLE_FORWARD:
                setIntakePower(INTAKE_POWER_FULL);
                if (!followerBusy) {
                    follower.followPath(driveToOpenGate, true);
                    setPathState(PathState.DRIVE_TO_OPEN_GATE);
                }
                break;

            case DRIVE_TO_OPEN_GATE:
                setIntakePower(INTAKE_POWER_FULL);
                if (!followerBusy) {
                    follower.followPath(driveToShootMiddle, true);
                    setPathState(PathState.DRIVE_TO_SHOOT_MIDDLE);
                }
                break;

            case DRIVE_TO_SHOOT_MIDDLE:
                checkEarlySpinup(MIDDLE_SET_SHOOT_POSE);
                if (!followerBusy) {
                    setIntakePower(INTAKE_POWER_OFF);
                    setPathState(PathState.SHOOT_MIDDLE);
                }
                break;

            case SHOOT_MIDDLE:
                handleShootingState(PathState.DRIVE_TO_GATE_1, driveToGate1);
                break;

            // Gate cycle 1
            case DRIVE_TO_GATE_1:
                setIntakePower(INTAKE_POWER_FULL);
                if (!followerBusy) setPathState(PathState.INTAKE_AT_GATE_1);
                break;

            case INTAKE_AT_GATE_1:
                setIntakePower(INTAKE_POWER_FULL);
                if (pathTimer.getElapsedTimeSeconds() > 1.35) {
                    follower.followPath(driveToShootGate1, true);
                    setPathState(PathState.DRIVE_TO_SHOOT_GATE_1);
                }
                break;

            case DRIVE_TO_SHOOT_GATE_1:
                checkEarlySpinup(GATE_CYCLE_1_SHOOT_POSE);
                if (!followerBusy) {
                    setIntakePower(INTAKE_POWER_OFF);
                    setPathState(PathState.SHOOT_GATE_1);
                }
                break;

            case SHOOT_GATE_1:
                handleShootingState(PathState.DRIVE_TO_GATE_2, driveToGate2);
                break;

            // Gate cycle 2
            case DRIVE_TO_GATE_2:
                setIntakePower(INTAKE_POWER_FULL);
                if (!followerBusy) setPathState(PathState.INTAKE_AT_GATE_2);
                break;

            case INTAKE_AT_GATE_2:
                setIntakePower(INTAKE_POWER_FULL);
                if (pathTimer.getElapsedTimeSeconds() > 1.35) {
                    follower.followPath(driveToShootGate2, true);
                    setPathState(PathState.DRIVE_TO_SHOOT_GATE_2);
                }
                break;

            case DRIVE_TO_SHOOT_GATE_2:
                checkEarlySpinup(GATE_CYCLE_2_SHOOT_POSE);
                if (!followerBusy) {
                    setIntakePower(INTAKE_POWER_OFF);
                    setPathState(PathState.SHOOT_GATE_2);
                }
                break;

            case SHOOT_GATE_2:
                handleShootingState(PathState.DRIVE_TO_GATE_3, driveToGate3);
                break;

            // Gate cycle 3
            case DRIVE_TO_GATE_3:
                setIntakePower(INTAKE_POWER_FULL);
                if (!followerBusy) setPathState(PathState.INTAKE_AT_GATE_3);
                break;

            case INTAKE_AT_GATE_3:
                setIntakePower(INTAKE_POWER_FULL);
                if (pathTimer.getElapsedTimeSeconds() > 1.35) {
                    follower.followPath(driveToShootGate3, true);
                    setPathState(PathState.DRIVE_TO_SHOOT_GATE_3);
                }
                break;

            case DRIVE_TO_SHOOT_GATE_3:
                checkEarlySpinup(GATE_CYCLE_3_SHOOT_POSE);
                if (!followerBusy) {
                    setIntakePower(INTAKE_POWER_OFF);
                    setPathState(PathState.SHOOT_GATE_3);
                }
                break;

            case SHOOT_GATE_3:
                handleShootingState(PathState.DRIVE_TO_GATE_4, driveToGate4);
                break;

            // Gate cycle 4
            case DRIVE_TO_GATE_4:
                setIntakePower(INTAKE_POWER_FULL);
                if (!followerBusy) setPathState(PathState.INTAKE_AT_GATE_4);
                break;

            case INTAKE_AT_GATE_4:
                setIntakePower(INTAKE_POWER_FULL);
                if (pathTimer.getElapsedTimeSeconds() > 1.35) {
                    follower.followPath(driveToShootGate4, true);
                    setPathState(PathState.DRIVE_TO_SHOOT_GATE_4);
                }
                break;

            case DRIVE_TO_SHOOT_GATE_4:
                checkEarlySpinup(GATE_CYCLE_4_SHOOT_POSE);
                if (!followerBusy) {
                    setIntakePower(INTAKE_POWER_OFF);
                    setPathState(PathState.SHOOT_GATE_4);
                }
                break;

            case SHOOT_GATE_4:
                handleShootingState(PathState.DRIVE_TO_TOP_SET, driveToTopSet);
                break;

            // Top set
            case DRIVE_TO_TOP_SET:
                setIntakePower(INTAKE_POWER_FULL);
                if (!followerBusy) {
                    follower.followPath(driveTopForward, true);
                    setPathState(PathState.DRIVE_TOP_FORWARD);
                }
                break;

            case DRIVE_TOP_FORWARD:
                setIntakePower(INTAKE_POWER_FULL);
                if (!followerBusy) {
                    follower.followPath(driveToShootTop, true);
                    setPathState(PathState.DRIVE_TO_SHOOT_TOP);
                }
                break;

            case DRIVE_TO_SHOOT_TOP:
                checkEarlySpinup(TOP_SET_SHOOT_POSE);
                if (!followerBusy) {
                    setIntakePower(INTAKE_POWER_OFF);
                    setPathState(PathState.SHOOT_TOP);
                }
                break;

            case SHOOT_TOP:
                handleShootingState(PathState.DONE, null);
                break;

            case DONE:
                telemetry.addLine("Auto Complete!");
                break;

            default:
                telemetry.addLine("Unknown state");
                break;
        }
    }

    // Helper method for shooting logic
    private void handleShootingState(PathState nextState, PathChain nextPath) throws InterruptedException {
        if (pathTimer.getElapsedTimeSeconds() > 0.5) {
            // Spin up shooters
            shooter1.setVelocity(shootVelocity);
            shooter2.setVelocity(shootVelocity);

            if (pathTimer.getElapsedTimeSeconds() > 1.0) {
                // Shoot
                shoot();
                ballsShot++;

                if (ballsShot >= 3) {
                    ballsShot = 0;
                    shooter1.setVelocity(idleVelocity);
                    shooter2.setVelocity(idleVelocity);

                    if (nextPath != null) {
                        follower.followPath(nextPath, true);
                    }
                    setPathState(nextState);
                }
            }
        }
    }

    // Helper method for early shooter spinup
    private void checkEarlySpinup(Pose targetPose) {
        double distanceToTarget = Math.hypot(
                follower.getPose().getX() - targetPose.getX(),
                follower.getPose().getY() - targetPose.getY()
        );

        // Start spinning up when within 20 inches of target
        if (distanceToTarget < 20) {
            shooter1.setVelocity(shootVelocity);
            shooter2.setVelocity(shootVelocity);
        }
    }

    // Helper method to set intake power
    private void setIntakePower(double power) {
        intake.setPower(power);
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {

        pathState = PathState.DRIVE_TO_SHOOT_PRELOAD;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        shootTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);

        // Initialize mechanisms using your team's hardware names
        shooter1 = hardwareMap.get(DcMotorEx.class, "Shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "Shooter2");
        intake = hardwareMap.get(DcMotor.class, "Intake");
        kicker = hardwareMap.get(Servo.class, "Kicker");

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(100, 0, 0, 15);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        kicker.setPosition(0.31);

        buildPaths();
        follower.setPose(STARTING_POSE);


    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);

        // Start shooters at idle velocity
        shooter1.setVelocity(idleVelocity);
        shooter2.setVelocity(idleVelocity);
    }

    @Override
    public void loop() {
        follower.update();

        try {
            statePathUpdate();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        // Log values
    }

    public void shoot() throws InterruptedException {
        intake.setPower(1);
        kicker.setPosition(0.6);
        sleep(160);
        kicker.setPosition(0.31);
    }
}