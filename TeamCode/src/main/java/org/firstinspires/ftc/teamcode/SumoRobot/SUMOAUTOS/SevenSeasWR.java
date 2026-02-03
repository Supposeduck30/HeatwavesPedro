package org.firstinspires.ftc.teamcode.SumoRobot.SUMOAUTOS;

import static java.lang.Thread.sleep;

import com.pedropathing.geometry.BezierCurve;
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
public class SevenSeasWR extends OpMode {
    private TelemetryManager panelsTelemetry;

    // Mechanisms
    private DcMotorEx shooter1;
    private DcMotorEx shooter2;
    private DcMotor intake;
    private Servo kicker;

    public double shootVelocity = 1840;
    public double idleVelocity = 1000;

    // Software
    public Follower follower;
    private Timer pathTimer, opModeTimer;

    // Shooting state
    private int ballsShot = 0;

    // States
    public enum PathState {
        DRIVE_TO_SHOOT1,
        SHOOT1_WAIT,
        SHOOT1,

        DRIVE_INTAKE_ROW1,
        DRIVE_FINISH_ROW1,

        DRIVE_ROW1_TO_SHOOT2,
        SHOOT2_WAIT,
        SHOOT2,

        DRIVE_SHOOT2_TO_HP_INTAKE,
        DRIVE_FINISH_HP,

        DRIVE_HP_TO_SHOOT3,
        SHOOT3_WAIT,
        SHOOT3,

        DRIVE_SHOOT3_TO_CHANCE1,
        DRIVE_CHANCE1_TO_SHOOT4,
        SHOOT4_WAIT,
        SHOOT4,

        DRIVE_SHOOT4_TO_CHANCE2,
        DRIVE_CHANCE2_TO_SHOOT5,
        SHOOT5_WAIT,
        SHOOT5,

        DRIVE_SHOOT5_TO_CHANCE3,
        DRIVE_CHANCE3_TO_SHOOT6,
        SHOOT6_WAIT,
        SHOOT6,

        DONE
    }

    PathState pathState;

    // Poses
    private final Pose startPose = new Pose(54.454, 8.000, Math.toRadians(90));
    private final Pose shoot1Pose = new Pose(59.600, 19.000, Math.toRadians(116));
    private final Pose intakeRow1Pose = new Pose(45.700, 35.400, Math.toRadians(180));
    private final Pose finishRow1Pose = new Pose(15.000, 35.400, Math.toRadians(180));
    private final Pose shoot2Pose = new Pose(59.600, 19.000, Math.toRadians(116));
    private final Pose hpIntakePose = new Pose(8.300, 40.000, Math.toRadians(270));
    private final Pose finishHPPose = new Pose(8.300, 17.000, Math.toRadians(270));
    private final Pose shoot3Pose = new Pose(59.600, 19.000, Math.toRadians(116));
    private final Pose bezierChance1Control = new Pose(2.500, 6.900, Math.toRadians(116));
    private final Pose chanceGrab1Pose = new Pose(4.900, 40.100, Math.toRadians(90));
    private final Pose shoot4Pose = new Pose(59.600, 19.000, Math.toRadians(116));
    private final Pose bezierChance2Control = new Pose(5.800, 85.700, Math.toRadians(116));
    private final Pose chanceGrab2Pose = new Pose(8.500, 13.700, Math.toRadians(270));
    private final Pose shoot5Pose = new Pose(59.600, 19.000, Math.toRadians(116));
    private final Pose bezierChance3Control = new Pose(43.700, 32.700, Math.toRadians(116));
    private final Pose chanceGrab3Pose = new Pose(9.800, 7.200, Math.toRadians(225));
    private final Pose shoot6Pose = new Pose(59.600, 19.000, Math.toRadians(116));

    private PathChain pathShoot1, pathIntakeRow1, pathFinishRow1, pathRow1ToShoot2;
    private PathChain pathShoot2ToHP, pathFinishHP, pathHPToShoot3;
    private PathChain pathShoot3ToChance1, pathChance1ToShoot4;
    private PathChain pathShoot4ToChance2, pathChance2ToShoot5;
    private PathChain pathShoot5ToChance3, pathChance3ToShoot6;

    public void buildPaths() {
        // Path to first shooting position
        pathShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shoot1Pose))
                .setConstantHeadingInterpolation(shoot1Pose.getHeading())
                .build();

        // Intake row 1
        pathIntakeRow1 = follower.pathBuilder()
                .addPath(new BezierLine(shoot1Pose, intakeRow1Pose))
                .setLinearHeadingInterpolation(shoot1Pose.getHeading(), intakeRow1Pose.getHeading())
                .build();

        pathFinishRow1 = follower.pathBuilder()
                .addPath(new BezierLine(intakeRow1Pose, finishRow1Pose))
                .setConstantHeadingInterpolation(finishRow1Pose.getHeading())
                .build();

        // Row 1 to shoot 2
        pathRow1ToShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(finishRow1Pose, shoot2Pose))
                .setConstantHeadingInterpolation(shoot2Pose.getHeading())
                .build();

        // Shoot 2 to human player
        pathShoot2ToHP = follower.pathBuilder()
                .addPath(new BezierLine(shoot2Pose, hpIntakePose))
                .setLinearHeadingInterpolation(shoot2Pose.getHeading(), hpIntakePose.getHeading())
                .build();

        pathFinishHP = follower.pathBuilder()
                .addPath(new BezierLine(hpIntakePose, finishHPPose))
                .setConstantHeadingInterpolation(finishHPPose.getHeading())
                .build();

        // HP to shoot 3
        pathHPToShoot3 = follower.pathBuilder()
                .addPath(new BezierLine(finishHPPose, shoot3Pose))
                .setConstantHeadingInterpolation(shoot3Pose.getHeading())
                .build();

        // Shoot 3 to chance grab 1
        pathShoot3ToChance1 = follower.pathBuilder()
                .addPath(new BezierCurve(shoot3Pose, bezierChance1Control, chanceGrab1Pose))
                .setLinearHeadingInterpolation(shoot3Pose.getHeading(), chanceGrab1Pose.getHeading())
                .build();

        pathChance1ToShoot4 = follower.pathBuilder()
                .addPath(new BezierLine(chanceGrab1Pose, shoot4Pose))
                .setConstantHeadingInterpolation(shoot4Pose.getHeading())
                .build();

        // Shoot 4 to chance grab 2
        pathShoot4ToChance2 = follower.pathBuilder()
                .addPath(new BezierCurve(shoot4Pose, bezierChance2Control, chanceGrab2Pose))
                .setLinearHeadingInterpolation(shoot4Pose.getHeading(), chanceGrab2Pose.getHeading())
                .build();

        pathChance2ToShoot5 = follower.pathBuilder()
                .addPath(new BezierLine(chanceGrab2Pose, shoot5Pose))
                .setConstantHeadingInterpolation(shoot5Pose.getHeading())
                .build();

        // Shoot 5 to chance grab 3
        pathShoot5ToChance3 = follower.pathBuilder()
                .addPath(new BezierCurve(shoot5Pose, bezierChance3Control, chanceGrab3Pose))
                .setLinearHeadingInterpolation(shoot5Pose.getHeading(), chanceGrab3Pose.getHeading())
                .build();

        pathChance3ToShoot6 = follower.pathBuilder()
                .addPath(new BezierLine(chanceGrab3Pose, shoot6Pose))
                .setConstantHeadingInterpolation(shoot6Pose.getHeading())
                .build();
    }

    public void statePathUpdate() throws InterruptedException {
        switch (pathState) {
            case DRIVE_TO_SHOOT1:
                follower.followPath(pathShoot1, true);
                setPathState(PathState.SHOOT1_WAIT);
                break;

            case SHOOT1_WAIT:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT1);
                }
                break;

            case SHOOT1:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    shoot();
                    ballsShot += 1;

                    if (ballsShot >= 3) {
                        ballsShot = 0;
                        follower.followPath(pathIntakeRow1, true);
                        setPathState(PathState.DRIVE_INTAKE_ROW1);
                    } else {
                        setPathState(PathState.SHOOT1);
                    }
                }
                break;

            case DRIVE_INTAKE_ROW1:
                if (!follower.isBusy()) {
                    follower.followPath(pathFinishRow1, true);
                    setPathState(PathState.DRIVE_FINISH_ROW1);
                }
                break;

            case DRIVE_FINISH_ROW1:
                if (!follower.isBusy()) {
                    follower.followPath(pathRow1ToShoot2, true);
                    setPathState(PathState.DRIVE_ROW1_TO_SHOOT2);
                }
                break;

            case DRIVE_ROW1_TO_SHOOT2:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT2_WAIT);
                }
                break;

            case SHOOT2_WAIT:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT2);
                }
                break;

            case SHOOT2:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    shoot();
                    ballsShot += 1;

                    if (ballsShot >= 3) {
                        ballsShot = 0;
                        follower.followPath(pathShoot2ToHP, true);
                        setPathState(PathState.DRIVE_SHOOT2_TO_HP_INTAKE);
                    } else {
                        setPathState(PathState.SHOOT2);
                    }
                }
                break;

            case DRIVE_SHOOT2_TO_HP_INTAKE:
                if (!follower.isBusy()) {
                    follower.followPath(pathFinishHP, true);
                    setPathState(PathState.DRIVE_FINISH_HP);
                }
                break;

            case DRIVE_FINISH_HP:
                if (!follower.isBusy()) {
                    follower.followPath(pathHPToShoot3, true);
                    setPathState(PathState.DRIVE_HP_TO_SHOOT3);
                }
                break;

            case DRIVE_HP_TO_SHOOT3:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT3_WAIT);
                }
                break;

            case SHOOT3_WAIT:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT3);
                }
                break;

            case SHOOT3:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    shoot();
                    ballsShot += 1;

                    if (ballsShot >= 3) {
                        ballsShot = 0;
                        follower.followPath(pathShoot3ToChance1, true);
                        setPathState(PathState.DRIVE_SHOOT3_TO_CHANCE1);
                    } else {
                        setPathState(PathState.SHOOT3);
                    }
                }
                break;

            case DRIVE_SHOOT3_TO_CHANCE1:
                if (!follower.isBusy()) {
                    follower.followPath(pathChance1ToShoot4, true);
                    setPathState(PathState.DRIVE_CHANCE1_TO_SHOOT4);
                }
                break;

            case DRIVE_CHANCE1_TO_SHOOT4:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT4_WAIT);
                }
                break;

            case SHOOT4_WAIT:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT4);
                }
                break;

            case SHOOT4:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    shoot();
                    ballsShot += 1;

                    if (ballsShot >= 3) {
                        ballsShot = 0;
                        follower.followPath(pathShoot4ToChance2, true);
                        setPathState(PathState.DRIVE_SHOOT4_TO_CHANCE2);
                    } else {
                        setPathState(PathState.SHOOT4);
                    }
                }
                break;

            case DRIVE_SHOOT4_TO_CHANCE2:
                if (!follower.isBusy()) {
                    follower.followPath(pathChance2ToShoot5, true);
                    setPathState(PathState.DRIVE_CHANCE2_TO_SHOOT5);
                }
                break;

            case DRIVE_CHANCE2_TO_SHOOT5:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT5_WAIT);
                }
                break;

            case SHOOT5_WAIT:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT5);
                }
                break;

            case SHOOT5:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    shoot();
                    ballsShot += 1;

                    if (ballsShot >= 3) {
                        ballsShot = 0;
                        follower.followPath(pathShoot5ToChance3, true);
                        setPathState(PathState.DRIVE_SHOOT5_TO_CHANCE3);
                    } else {
                        setPathState(PathState.SHOOT5);
                    }
                }
                break;

            case DRIVE_SHOOT5_TO_CHANCE3:
                if (!follower.isBusy()) {
                    follower.followPath(pathChance3ToShoot6, true);
                    setPathState(PathState.DONE);
                }
                break;


            case DONE:
                telemetry.addLine("Auto Complete - 18 Balls Shot");
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

        pathState = PathState.DRIVE_TO_SHOOT1;
        pathTimer = new Timer();
        opModeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(1.0);

        // Initialize mechanisms
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
        follower.setPose(startPose);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);

        // Start shooters and intake at max velocity - they stay spinning the whole auto
        shooter1.setVelocity(shootVelocity);
        shooter2.setVelocity(shootVelocity);
        intake.setPower(1);
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
        panelsTelemetry.debug("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("Follower Busy", follower.isBusy());
        panelsTelemetry.debug("Balls Shot", ballsShot);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Shooter1 Velocity", shooter1.getVelocity());
        panelsTelemetry.debug("Shooter2 Velocity", shooter2.getVelocity());
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void stop() {
        Pose finalPose = follower.getPose();
        org.firstinspires.ftc.teamcode.pedroPathing.mechanisms.PedroPose.saveCurrentPose(finalPose);
        super.stop();
    }

    public void shoot() throws InterruptedException {
        kicker.setPosition(0.6);
        sleep(160);
        kicker.setPosition(0.31);
    }
}