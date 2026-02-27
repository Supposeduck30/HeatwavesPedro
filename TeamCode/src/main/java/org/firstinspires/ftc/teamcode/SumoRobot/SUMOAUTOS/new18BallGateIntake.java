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
public class new18BallGateIntake extends OpMode {

    private TelemetryManager panelsTelemetry;

    // Mechanisms
    private DcMotorEx shooter1;
    private DcMotorEx shooter2;
    private DcMotor intake;
    private Servo kicker;
    private DcMotorEx turret;

    public double shootVelocity = 1530;
    public int TURRET_TARGET_TICKS = 219;


    // Software
    public Follower follower;
    private Timer pathTimer, opModeTimer, kickTimer;

    // Shooting state
    private boolean shooting = false;
    private int ballsShot = 0;

    public enum PathState {
        DRIVE_SHOOT_PRELOAD1,
        SPINUP1,
        SHOOT1,
        INTAKE_CLOSE1,

        DRIVE_INTAKE_SPIKE2,
        DRIVE_SPIKE2_TO_SHOOT2,
        SHOOT2,
        INTAKE_CLOSE2,

        DRIVE_SHOOT2_TO_GATE,
        DRIVE_GATE_TO_INTAKE,
        DRIVE_INTAKE_GATE1_TO_SHOOT3,
        SHOOT3,

        DONE,
    }

    PathState pathState;

    // ── Poses ─────────────────────────────────────────────────────────────────
    // Robot physical start pose
    private final Pose startPose           = new Pose(24.600, 126.700, Math.toRadians(135));

    // Each path has its own shoot spot (same coordinates, separate poses)
    private final Pose shootSpot1          = new Pose(60.600,  84.000, Math.toRadians(180));
    private final Pose shootSpot2          = new Pose(60.600,  84.000, Math.toRadians(180));
    private final Pose shootSpot3          = new Pose(60.600,  84.000, Math.toRadians(180));

    // IntakeSpike2
    private final Pose intakeSpike2CP      = new Pose(69.000,  55.000, Math.toRadians(180));
    private final Pose intakeSpike2End     = new Pose(15.500,  60.200, Math.toRadians(180));

    // Spike2ToShoot2
    private final Pose spike2ToShoot2CP    = new Pose(45.500,  67.200, Math.toRadians(180));

    // Shoot2ToGate
    private final Pose shoot2ToGateEnd     = new Pose(13.380,  60.245, Math.toRadians(135));

    // GateToIntake
    private final Pose gateToIntakeEnd     = new Pose( 9.129,  54.264, Math.toRadians(100));

    // ── Path chains ────────────────────────────────────────────────────────────
    private PathChain shootPreload1, intakeSpike2, spike2ToShoot2,
            shoot2ToGate, gateToIntake, intakeGate1ToShoot3;

    public void buildPaths() {
        // ShootPreload1: startPose → shootSpot1, heading 135°→180°
        shootPreload1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootSpot1))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

        // IntakeSpike2: curve shootSpot1 → CP(69,55) → (15.5,60.2), constant 180°
        intakeSpike2 = follower.pathBuilder()
                .addPath(new BezierCurve(shootSpot1, intakeSpike2CP, intakeSpike2End))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        // Spike2ToShoot2: curve (15.5,60.2) → CP(45.5,67.2) → shootSpot2, constant 180°
        spike2ToShoot2 = follower.pathBuilder()
                .addPath(new BezierCurve(intakeSpike2End, spike2ToShoot2CP, shootSpot2))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        // Shoot2ToGate: shootSpot2 → (13.38,60.245), heading 180°→135°
        shoot2ToGate = follower.pathBuilder()
                .addPath(new BezierLine(shootSpot2, shoot2ToGateEnd))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();

        // GateToIntake: (13.38,60.245) → (9.129,54.264), heading 135°→100°
        gateToIntake = follower.pathBuilder()
                .addPath(new BezierLine(shoot2ToGateEnd, gateToIntakeEnd))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(100))
                .build();

        // IntakeGate1ToShoot3: (9.129,54.264) → shootSpot3, heading 100°→180°
        intakeGate1ToShoot3 = follower.pathBuilder()
                .addPath(new BezierLine(gateToIntakeEnd, shootSpot3))
                .setLinearHeadingInterpolation(Math.toRadians(100), Math.toRadians(180))
                .build();
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    public void statePathUpdate() {
        switch (pathState) {

            // ── ShootPreload1: startPose → shootSpot1 → shoot ─────────────────
            case DRIVE_SHOOT_PRELOAD1:
                follower.followPath(shootPreload1, true);
                setPathState(PathState.SPINUP1);
                break;
            case SPINUP1:
                if (pathTimer.getElapsedTimeSeconds() > 2.5){
                    setPathState(PathState.SHOOT1);
                }
            case SHOOT1:
                if (!follower.isBusy()&& pathTimer.getElapsedTimeSeconds() > 2.5) {
                    if (!shooting ) {
                        kicker.setPosition(0.25);
                        shooting = true;
                        kickTimer.resetTimer();
                    } else if (shooting && kickTimer.getElapsedTimeSeconds() > 0.16) {
                        shooting = false;
                        ballsShot++;
                        if (ballsShot >= 3) {
                            ballsShot = 0;
                            follower.followPath(intakeSpike2, true);
                            setPathState(PathState.INTAKE_CLOSE1);
                        } else {
                            pathTimer.resetTimer();
                        }
                    }
                }
                break;

            case INTAKE_CLOSE1:
                kicker.setPosition(0.15);
                setPathState(PathState.DRIVE_INTAKE_SPIKE2);
                break;

            // ── IntakeSpike2 running → wait → Spike2ToShoot2 → shootSpot2 ─────
            case DRIVE_INTAKE_SPIKE2:
                if (!follower.isBusy()) {
                    follower.followPath(spike2ToShoot2, true);
                    setPathState(PathState.DRIVE_SPIKE2_TO_SHOOT2);
                }
                break;

            case DRIVE_SPIKE2_TO_SHOOT2:
                if (!follower.isBusy()) {
                    ballsShot = 0;
                    setPathState(PathState.SHOOT2);
                }
                break;

            case SHOOT2:
                kicker.setPosition(0.25);
                if (!shooting && pathTimer.getElapsedTimeSeconds() > 0.2) {
                    shooting = true;
                    kickTimer.resetTimer();
                } else if (shooting && kickTimer.getElapsedTimeSeconds() > 0.16) {
                    shooting = false;
                    ballsShot++;
                    if (ballsShot >= 3) {
                        ballsShot = 0;
                        follower.followPath(shoot2ToGate, true);
                        setPathState(PathState.INTAKE_CLOSE2);
                    } else {
                        pathTimer.resetTimer();
                    }
                }
                break;

            case INTAKE_CLOSE2:
                kicker.setPosition(0.15);
                setPathState(PathState.DRIVE_SHOOT2_TO_GATE);
                break;

            // ── Shoot2ToGate running → GateToIntake → IntakeGate1ToShoot3 → shootSpot3
            case DRIVE_SHOOT2_TO_GATE:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.0) {
                    follower.followPath(gateToIntake, true);
                    setPathState(PathState.DRIVE_GATE_TO_INTAKE);
                }
                break;

            case DRIVE_GATE_TO_INTAKE:
                if (!follower.isBusy()) {
                    follower.followPath(intakeGate1ToShoot3, true);
                    setPathState(PathState.DRIVE_INTAKE_GATE1_TO_SHOOT3);
                }
                break;

            case DRIVE_INTAKE_GATE1_TO_SHOOT3:
                if (!follower.isBusy()) {
                    ballsShot = 0;
                    setPathState(PathState.SHOOT3);
                }
                break;

            case SHOOT3:
                if (!shooting && pathTimer.getElapsedTimeSeconds() > 0.2) {
                    kicker.setPosition(0.25);
                    shooting = true;
                    kickTimer.resetTimer();
                } else if (shooting && kickTimer.getElapsedTimeSeconds() > 0.16) {
                    kicker.setPosition(0.31);
                    shooting = false;
                    ballsShot++;
                    if (ballsShot >= 3) {
                        ballsShot = 0;
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
                telemetry.addLine("Done all Paths");
                break;
        }
    }

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        pathTimer   = new Timer();
        opModeTimer = new Timer();
        kickTimer   = new Timer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        shooter1 = hardwareMap.get(DcMotorEx.class, "Shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "Shooter2");
        intake   = hardwareMap.get(DcMotor.class,   "Intake");
        kicker   = hardwareMap.get(Servo.class,     "Kicker");
        turret  = hardwareMap.get(DcMotorEx.class,"Turret");

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(TURRET_TARGET_TICKS);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(0.5);
        kicker.setPosition(0.15);
        PIDFCoefficients pidf = new PIDFCoefficients(120, 0, 0, 25);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        kicker.setPosition(0.15);

        buildPaths();
        pathState = PathState.DRIVE_SHOOT_PRELOAD1;
        shooting  = false;

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        pathTimer.resetTimer();

        shooter1.setVelocity(shootVelocity);
        shooter2.setVelocity(shootVelocity);
        intake.setPower(1);
    }

    @Override
    public void loop() {
        follower.update();

        shooter1.setVelocity(shootVelocity);
        shooter2.setVelocity(shootVelocity);
        intake.setPower(1);

        statePathUpdate();

        panelsTelemetry.debug("Path State",        pathState);
        panelsTelemetry.debug("X",                 follower.getPose().getX());
        panelsTelemetry.debug("Y",                 follower.getPose().getY());
        panelsTelemetry.debug("Heading (deg)",     Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.debug("Follower Busy",     follower.isBusy());
        panelsTelemetry.debug("Balls Shot",        ballsShot);
        panelsTelemetry.debug("Shooter1 Velocity", shooter1.getVelocity());
        panelsTelemetry.debug("Shooter2 Velocity", shooter2.getVelocity());
        panelsTelemetry.update(telemetry);
    }
}