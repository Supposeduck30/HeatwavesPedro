package org.firstinspires.ftc.teamcode.SumoRobot.SUMOAUTOS;
import static java.lang.Thread.sleep;

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
public class Blue18BallCloseTurret extends OpMode {
    private TelemetryManager panelsTelemetry;
    // Mechanisms
    private DcMotorEx shooter1, shooter2;
    private DcMotor intake;
    private Servo kicker;
    private DcMotorEx turret;
    public double shootVelocity = 1650;
    // Fixed turret position in ticks
    private static final int TURRET_TARGET_TICKS = 219;
    public Follower follower;
    private Timer pathTimer, opModeTimer, kickTimer;
    private boolean shooting = false;
    private int ballsShot = 0;
    public enum PathState {
        DRIVE_PRELOAD_TO_SHOOT1,
        SPIN_UP1,
        SHOOT1,
        INTAKECLOSE1,
        DRIVE_SHOOT1_TO_SPIKE2,
        DRIVE_SPIKE2_TO_SHOOT2,
        SPIN_UP2,
        SHOOT2,
        INTAKECLOSE2,
        DRIVE_SHOOT2_TO_GATE,
        INTAKE1,
        GATE_TO_SHOOT3,
        SPIN_UP3,
        SHOOT3,
        DONE
    }
    PathState pathState;
    // ================= POSES =================
    private final Pose startPose = new Pose(16.800, 114.400, Math.toRadians(180));
    private final Pose shoot1Pose = new Pose(48.600, 93.400, Math.toRadians(180));
    private final Pose intakeSpike2Pose = new Pose(16.500, 57.900, Math.toRadians(180));
    private final Pose shoot2Pose = new Pose(48.700, 93.400, Math.toRadians(180));
    private final Pose gateIntakePose = new Pose(10, 58, Math.toRadians(140.58));
    private final Pose shoot3Pose = new Pose(48.7,93.4,Math.toRadians(180));

    private final Pose bezierShoot1ToSpike = new Pose(87.200, 52.400, Math.toRadians(180));
    private final Pose bezierSpikeToShoot2 = new Pose(50.100, 66.500, Math.toRadians(180));
    private final Pose bezierShoot2ToGate = new Pose(39.800, 45.600, Math.toRadians(180));
    private final Pose bezierGateToShoot3 = new Pose(64.7,65.6, Math.toRadians(180));

    private PathChain preloadToShoot1;
    private PathChain shoot1ToSpike2;
    private PathChain spike2ToShoot2;
    private PathChain shoot2ToGate;
    private PathChain gateToShoot3;

    public void buildPaths() {
        preloadToShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shoot1Pose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shoot1Pose.getHeading())
                .build();
        shoot1ToSpike2 = follower.pathBuilder()
                .addPath(new BezierCurve(shoot1Pose, bezierShoot1ToSpike, intakeSpike2Pose))
                .setLinearHeadingInterpolation(shoot1Pose.getHeading(), intakeSpike2Pose.getHeading())
                .build();
        spike2ToShoot2 = follower.pathBuilder()
                .addPath(new BezierCurve(intakeSpike2Pose, bezierSpikeToShoot2, shoot2Pose))
                .setLinearHeadingInterpolation(intakeSpike2Pose.getHeading(), shoot2Pose.getHeading())
                .build();
        shoot2ToGate = follower.pathBuilder()
                .addPath(new BezierCurve(shoot2Pose, bezierShoot2ToGate, gateIntakePose))
                .setConstantHeadingInterpolation(gateIntakePose.getHeading())
                .build();
        gateToShoot3 = follower.pathBuilder()
                .addPath(new BezierCurve(gateIntakePose, bezierGateToShoot3, shoot3Pose))
                .setLinearHeadingInterpolation(gateIntakePose.getHeading(),shoot3Pose.getHeading())
                .build();

    }
    // Only resets timer — does NOT reset ballsShot
    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }
    public void statePathUpdate() throws InterruptedException {
        switch (pathState) {
            case DRIVE_PRELOAD_TO_SHOOT1:
                follower.followPath(preloadToShoot1, true);
                setPathState(PathState.SPIN_UP1);
                break;
            // CHANGE 1: removed setVelocity calls here — shooter runs continuously in loop()
            // CHANGE 3: increased spin-up wait from 1.0s to 2.5s
            case SPIN_UP1:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3.5) {
                    ballsShot = 0;
                    setPathState(PathState.SHOOT1);
                }
                break;
            case SHOOT1:
                if (!shooting && pathTimer.getElapsedTimeSeconds() > 0.4) {
                    intake.setPower(1);
                    kicker.setPosition(0.25);
                    shooting = true;
                    kickTimer.resetTimer();
                } else if (shooting && kickTimer.getElapsedTimeSeconds() > 0.14) {
                    shooting = false;
                    ballsShot++;
                    if (ballsShot >= 3) {
                        intake.setPower(1);
                        follower.followPath(shoot1ToSpike2, 0.8, true); // Slower speed for this path
                        setPathState(PathState.INTAKECLOSE1);
                    } else {
                        pathTimer.resetTimer();
                    }
                }
                break;
            case INTAKECLOSE1:
                kicker.setPosition(0.15);
                setPathState(PathState.DRIVE_SHOOT1_TO_SPIKE2);
                break;
            case DRIVE_SHOOT1_TO_SPIKE2:
                if (!follower.isBusy()) {
                    follower.followPath(spike2ToShoot2, true);
                    setPathState(PathState.DRIVE_SPIKE2_TO_SHOOT2);
                }
                break;
            // CHANGE 1: removed setVelocity calls here — shooter already running
            case DRIVE_SPIKE2_TO_SHOOT2:
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    setPathState(PathState.SPIN_UP2);
                }
                break;
            case SPIN_UP2:
                if (pathTimer.getElapsedTimeSeconds() > 1.0) {
                    ballsShot = 0;
                    setPathState(PathState.SHOOT2);
                }
                break;
            case SHOOT2:
                if (!shooting && pathTimer.getElapsedTimeSeconds() > 0.1) {
                    intake.setPower(1);
                    kicker.setPosition(0.25);
                    shooting = true;
                    kickTimer.resetTimer();
                } else if (shooting && kickTimer.getElapsedTimeSeconds() > 0.14) {
                    shooting = false;
                    ballsShot++;
                    if (ballsShot >= 3) {
                        intake.setPower(1); // Ensure intake is full speed
                        follower.followPath(shoot2ToGate, 0.8, true); // Slower speed for going to gate
                        setPathState(PathState.INTAKECLOSE2);
                    } else {
                        pathTimer.resetTimer();
                    }
                }
                break;
            case INTAKECLOSE2:
                kicker.setPosition(0.15);
                setPathState(PathState.DRIVE_SHOOT2_TO_GATE);
                break;
            case DRIVE_SHOOT2_TO_GATE:
                if (!follower.isBusy()) {
                    setPathState(PathState.INTAKE1);
                }
                break;
            case INTAKE1:
                intake.setPower(1.0);
                if (pathTimer.getElapsedTimeSeconds() > 0.4) {
                    setPathState(PathState.GATE_TO_SHOOT3);
                }

                break;
            case GATE_TO_SHOOT3:
                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(gateToShoot3, true);
                    setPathState(PathState.SHOOT3);
                }
                break;
            case SHOOT3:
                if (!shooting && pathTimer.getElapsedTimeSeconds() > 0.1) {
                    intake.setPower(1);
                    kicker.setPosition(0.25);
                    shooting = true;
                    kickTimer.resetTimer();
                } else if (shooting && kickTimer.getElapsedTimeSeconds() > 0.14) {
                    shooting = false;
                    ballsShot++;
                    if (ballsShot >= 3) {
                        intake.setPower(1); // Ensure intake is full speed
                        setPathState(PathState.DONE);
                    } else {
                        pathTimer.resetTimer();
                    }
                }
                break;
            case DONE:
                shooter1.setVelocity(0);
                shooter2.setVelocity(0);
                intake.setPower(0);
                kicker.setPosition(0.31);
                break;
        }
    }
    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        pathTimer = new Timer();
        opModeTimer = new Timer();
        kickTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);
        shooter1 = hardwareMap.get(DcMotorEx.class, "Shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "Shooter2");
        intake = hardwareMap.get(DcMotor.class, "Intake");
        kicker = hardwareMap.get(Servo.class, "Kicker");
        turret = hardwareMap.get(DcMotorEx.class, "Turret");
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);
        PIDFCoefficients pidf = new PIDFCoefficients(400, 0, 0, 25);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(TURRET_TARGET_TICKS);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(0.5);
        kicker.setPosition(0.15);
        buildPaths();
        pathState = PathState.DRIVE_PRELOAD_TO_SHOOT1;
        shooting = false;
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }
    @Override
    public void start() {
        opModeTimer.resetTimer();
        pathTimer.resetTimer();
        // Shooter starts immediately at full velocity
        shooter1.setVelocity(shootVelocity);
        shooter2.setVelocity(shootVelocity);
    }
    @Override
    public void loop() {
        follower.update();
        // CHANGE 1: shooter velocity re-set every loop so it never drops,
        // even when driving between positions
        shooter1.setVelocity(shootVelocity);
        shooter2.setVelocity(shootVelocity);
        try {
            statePathUpdate();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("Follower Busy", follower.isBusy());
        panelsTelemetry.debug("Balls Shot", ballsShot);
        panelsTelemetry.debug("Turret Pos", turret.getCurrentPosition());
        panelsTelemetry.debug("Turret Target", TURRET_TARGET_TICKS);
        panelsTelemetry.debug("Shooter1 Velocity", shooter1.getVelocity());
        panelsTelemetry.debug("Shooter2 Velocity", shooter2.getVelocity());
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.update(telemetry);
    }
    @Override
    public void stop() {
        turret.setPower(0);
        shooter1.setVelocity(0);
        shooter2.setVelocity(0);
        intake.setPower(0);
        kicker.setPosition(0.31);
    }
}