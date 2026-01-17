package org.firstinspires.ftc.teamcode.SumoRobot.SUMOAUTOS;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;


@Autonomous
@Configurable
public class Blue12BallFar extends OpMode {
    private TelemetryManager panelsTelemetry;

    // Mechanisms
    private DcMotorEx shooter1;
    private DcMotorEx shooter2;
    private DcMotor intake;
    private Servo kicker;

    public double shootVelocity = 1875; // Adjust as needed

    // Software
    public Follower follower;
    private Timer pathTimer, opModeTimer;

    // States
    public enum PathState {
        DRIVE_PATH1,
        SHOOT_PATH1,
        DRIVE_PATH2,
        DRIVE_PATH3,
        INTAKE_ON_PATH3,
        DRIVE_PATH4,
        DRIVE_PATH5,
        SHOOT_PATH5,
        DRIVE_PATH6,
        DRIVE_PATH7,
        INTAKE_ON_PATH7,
        DRIVE_PATH8,
        SHOOT_PATH8,
        DRIVE_PATH9,
        DRIVE_PATH10,
        DRIVE_PATH11,
        SHOOT_PATH11,
        DRIVE_PATH12,
        DONE
    }

    PathState pathState;

    // Poses from your original paths
    private final Pose startPose = new Pose(54.5, 8, Math.toRadians(90));
    private final Pose pose1End = new Pose(60.270, 15.611, Math.toRadians(119));
    private final Pose pose2End = new Pose(47.684, 60.257, Math.toRadians(180));
    private final Pose pose3End = new Pose(14.982, 60.130, Math.toRadians(180));
    private final Pose pose4End = new Pose(14.982, 70.002, Math.toRadians(270));
    private final Pose pose5End = new Pose(60.040, 15.611, Math.toRadians(115));
    private final Pose pose6End = new Pose(47.853, 34.755, Math.toRadians(180));
    private final Pose pose7End = new Pose(11.720, 35.761, Math.toRadians(180));
    private final Pose pose8End = new Pose(60.040, 15.611, Math.toRadians(117));
    private final Pose pose9End = new Pose(6.900, 31.000, Math.toRadians(270));
    private final Pose pose10End = new Pose(6.900, 10.000, Math.toRadians(270));
    private final Pose pose11End = new Pose(60.040, 15.611, Math.toRadians(117));
    private final Pose pose12End = new Pose(38.712, 32.526, Math.toRadians(90));

    private PathChain path1, path2, path3, path4, path5, path6, path7, path8, path9, path10, path11, path12;

    public void buildPaths() {
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, pose1End))
                .setLinearHeadingInterpolation(startPose.getHeading(), pose1End.getHeading())
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
                .addPath(new BezierLine(pose3End, pose4End))
                .setLinearHeadingInterpolation(pose3End.getHeading(), pose4End.getHeading())
                .build();

        path5 = follower.pathBuilder()
                .addPath(new BezierLine(pose4End, pose5End))
                .setLinearHeadingInterpolation(pose4End.getHeading(), pose5End.getHeading())
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
                .setLinearHeadingInterpolation(pose7End.getHeading(), pose8End.getHeading())
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
                .setLinearHeadingInterpolation(pose10End.getHeading(), pose11End.getHeading())
                .build();

        path12 = follower.pathBuilder()
                .addPath(new BezierLine(pose11End, pose12End))
                .setLinearHeadingInterpolation(pose11End.getHeading(), pose12End.getHeading())
                .build();
    }

    public void statePathUpdate() throws InterruptedException {
        switch (pathState) {
            case DRIVE_PATH1:
                follower.followPath(path1, true);
                setPathState(PathState.SHOOT_PATH1);
                break;

            case SHOOT_PATH1:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5) {
                    sleep(300);
                    shoot();
                    sleep(150);
                    shoot();
                    sleep(160);
                    shoot();
                    setPathState(PathState.DRIVE_PATH2);
                }
                break;

            case DRIVE_PATH2:
                follower.followPath(path2, true);
                setPathState(PathState.DRIVE_PATH3);
                break;

            case DRIVE_PATH3:
                if (!follower.isBusy()) {
                    follower.followPath(path3, true);
                    setPathState(PathState.INTAKE_ON_PATH3);
                }
                break;

            case INTAKE_ON_PATH3:
                intake.setPower(1);
                setPathState(PathState.DRIVE_PATH4);
                break;

            case DRIVE_PATH4:
                if (!follower.isBusy()) {
                    follower.followPath(path4, true);
                    setPathState(PathState.DRIVE_PATH5);
                }
                break;

            case DRIVE_PATH5:
                if (!follower.isBusy()) {
                    follower.followPath(path5, true);
                    setPathState(PathState.SHOOT_PATH5);
                }
                break;

            case SHOOT_PATH5:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5) {
                    shoot();
                    sleep(150);
                    shoot();
                    sleep(160);
                    shoot();
                    setPathState(PathState.DRIVE_PATH6);
                }
                break;

            case DRIVE_PATH6:
                follower.followPath(path6, true);
                setPathState(PathState.DRIVE_PATH7);
                break;

            case DRIVE_PATH7:
                if (!follower.isBusy()) {
                    follower.followPath(path7, true);
                    setPathState(PathState.INTAKE_ON_PATH7);
                }
                break;

            case INTAKE_ON_PATH7:
                intake.setPower(1);
                setPathState(PathState.DRIVE_PATH8);
                break;

            case DRIVE_PATH8:
                if (!follower.isBusy()) {
                    follower.followPath(path8, true);
                    setPathState(PathState.SHOOT_PATH8);
                }
                break;

            case SHOOT_PATH8:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5) {
                    shoot();
                    sleep(150);
                    shoot();
                    sleep(160);
                    shoot();
                    setPathState(PathState.DRIVE_PATH9);
                }
                break;

            case DRIVE_PATH9:
                follower.followPath(path9, true);
                setPathState(PathState.DRIVE_PATH10);
                break;

            case DRIVE_PATH10:
                if (!follower.isBusy()) {
                    follower.followPath(path10, true);
                    setPathState(PathState.DRIVE_PATH11);
                }
                break;

            case DRIVE_PATH11:
                if (!follower.isBusy()) {
                    follower.followPath(path11, true);
                    setPathState(PathState.SHOOT_PATH11);
                }
                break;

            case SHOOT_PATH11:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5) {
                    shoot();
                    sleep(150);
                    shoot();
                    sleep(160);
                    shoot();
                    setPathState(PathState.DRIVE_PATH12);
                }
                break;

            case DRIVE_PATH12:
                follower.followPath(path12, true);
                setPathState(PathState.DONE);
                break;

            case DONE:
                if (!follower.isBusy()) {
                    telemetry.addLine("Done all Paths");
                }
                break;

            default:
                telemetry.addLine("DONE :)");
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        pathState = PathState.DRIVE_PATH1;
        pathTimer = new Timer();
        opModeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);

        // Initialize mechanisms - CHANGE THESE TO YOUR ACTUAL HARDWARE NAMES
        shooter1 = hardwareMap.get(DcMotorEx.class, "Shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "Shooter2");
        intake = hardwareMap.get(DcMotor.class, "Intake");
        kicker = hardwareMap.get(Servo.class, "Kicker");

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(110, 0, 0, 20);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        kicker.setPosition(0.31);

        buildPaths();
        follower.setPose(startPose);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);

        // Start shooters running at the beginning and keep them running throughout
        shooter1.setVelocity(shootVelocity);
        shooter2.setVelocity(shootVelocity);
    }

    @Override
    public void loop() {
        follower.update();

        // Keep shooters running constantly throughout the auto
        shooter1.setVelocity(shootVelocity);
        shooter2.setVelocity(shootVelocity);

        try {
            statePathUpdate();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.debug("Shooter1 Velocity", shooter1.getVelocity());
        panelsTelemetry.debug("Shooter2 Velocity", shooter2.getVelocity());
        panelsTelemetry.update(telemetry);
    }

    public void shoot() throws InterruptedException {
        // Shooters are already running, just kick and intake
        intake.setPower(1);
        kicker.setPosition(0.6);  // Kick out
        sleep(140);                // Wait for kick (matching your teleop KICK_TIME)
        kicker.setPosition(0.31);  // Retract
    }
}