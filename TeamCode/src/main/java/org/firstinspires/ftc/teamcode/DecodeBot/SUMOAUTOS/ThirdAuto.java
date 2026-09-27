package org.firstinspires.ftc.teamcode.DecodeBot.SUMOAUTOS;

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
import org.firstinspires.ftc.teamcode.DecodeBot.TurretControllerRED;

@Autonomous(name="Third Team Auto Red CRI", group="Autonomous")
@Configurable
public class ThirdAuto extends OpMode {
    private TelemetryManager panelsTelemetry;

    // Hardware Mechanisms
    private DcMotorEx shooter1;
    private DcMotorEx shooter2;
    private DcMotor intake;
    private Servo kicker;

    // Live Turret Controller
    private TurretControllerRED turretController;
    public double shootVelocity = 1510;

    // Software
    public Follower follower;
    private Timer pathTimer, opModeTimer, kickTimer;

    // Pose and Velocity Tracking for Turret Prediction
    private Pose lastPose = null;
    private long lastTime = 0;

    // Shooting state
    private boolean shooting = false;
    private int ballsShot = 0;

    // Explicit State Machine
    public enum PathState {
        DRIVE_PATH1,
        SPIN_UP1, SHOOT1, INTAKECLOSE1,
        DRIVE_PATH2, WAIT_INTAKE1, DRIVE_PATH3,
        SPIN_UP2, SHOOT2, INTAKECLOSE2,
        DRIVE_PATH4, WAIT_INTAKE2, DRIVE_PATH5,
        SPIN_UP3, SHOOT3, INTAKECLOSE3,
        DRIVE_PATH6, WAIT_INTAKE3, DRIVE_PATH7,
        SPIN_UP4, SHOOT4, INTAKECLOSE4,
        DRIVE_PATH8, WAIT_INTAKE4, DRIVE_PATH9,
        DONE
    }

    PathState pathState;
    private final Pose startPose = new Pose(111.308, 160.041, Math.toRadians(0));
    private PathChain path1, path2, path3, path4, path5, path6, path7, path8, path9;

    public void buildPaths() {
        // Path 1: Drive to first shoot position (Preloads)
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(111.308, 160.041), new Pose(113.324, 117.288)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // Path 2: Sweep/Intake 1
        path2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(113.324, 117.288), new Pose(117.500, 82.100), new Pose(172.700, 83.100)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // Path 3: Return to shoot 1
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(172.700, 83.100), new Pose(112.400, 102.100)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // Path 4: Sweep/Intake 2
        path4 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(112.400, 102.100), new Pose(175.500, 80.000)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(40))
                .build();

        // Path 5: Return to shoot 2
        path5 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(175.500, 80.000), new Pose(112.400, 102.100)))
                .setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(0))
                .build();

        // Path 6: Sweep/Intake 3
        path6 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(112.400, 102.100), new Pose(175.500, 80.000)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(40))
                .build();

        // Path 7: Return to shoot 3
        path7 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(175.500, 80.000), new Pose(112.400, 102.100)))
                .setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(0))
                .build();

        // Path 8: Sweep/Intake 4
        path8 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(112.400, 102.100), new Pose(175.500, 80.000)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(40))
                .build();

        // Path 9: Final Park
        path9 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(175.500, 80.000), new Pose(106.658, 125.009)))
                .setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(0))
                .build();
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    public void statePathUpdate() {
        switch (pathState) {
            // ================== PRELOAD CYCLE ==================
            case DRIVE_PATH1:
                follower.followPath(path1, true);
                setPathState(PathState.SPIN_UP1);
                break;
            case SPIN_UP1:
                // Wait for the robot to finish moving to preload position
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3.0) {
                    ballsShot = 0;
                    setPathState(PathState.SHOOT1);
                }
                break;
            case SHOOT1:
                // Shoots all 3 preload balls
                double fireDelay = (ballsShot == 0) ? 0.0 : 0.45;
                if (!shooting && pathTimer.getElapsedTimeSeconds() > fireDelay && ballsShot < 3) {
                    intake.setPower(1);
                    kicker.setPosition(0.25);
                    shooting = true;
                    kickTimer.resetTimer();
                }
                if (shooting && kickTimer.getElapsedTimeSeconds() > 0.16) {
                    kicker.setPosition(0.31);
                    shooting = false;
                    ballsShot++;
                    pathTimer.resetTimer();
                }
                if (ballsShot >= 3) {
                    if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                        ballsShot = 0;
                        intake.setPower(1);
                        follower.followPath(path2, true);
                        setPathState(PathState.INTAKECLOSE1);
                    }
                }
                break;
            case INTAKECLOSE1:
                kicker.setPosition(0.15);
                setPathState(PathState.DRIVE_PATH2);
                break;

            // ================== CYCLE 1 ==================
            case DRIVE_PATH2:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 2.5) {
                    intake.setPower(1);
                    setPathState(PathState.WAIT_INTAKE1);
                }
                break;
            case WAIT_INTAKE1:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(path3, true);
                    setPathState(PathState.DRIVE_PATH3);
                }
                break;
            case DRIVE_PATH3:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 2.5) {
                    intake.setPower(0);
                    setPathState(PathState.SPIN_UP2);
                }
                break;
            case SPIN_UP2:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    ballsShot = 0;
                    setPathState(PathState.SHOOT2);
                }
                break;
            case SHOOT2:
                if (!shooting && ballsShot == 0) {
                    intake.setPower(1);
                    kicker.setPosition(0.25);
                    shooting = true;
                    kickTimer.resetTimer();
                    ballsShot = 1;
                }
                if (shooting && kickTimer.getElapsedTimeSeconds() > 0.16) {
                    kicker.setPosition(0.31);
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.56) {
                    shooting = false;
                    ballsShot = 0;
                    intake.setPower(1);
                    follower.followPath(path4, true);
                    setPathState(PathState.INTAKECLOSE2);
                }
                break;
            case INTAKECLOSE2:
                kicker.setPosition(0.15);
                setPathState(PathState.DRIVE_PATH4);
                break;

            // ================== CYCLE 2 ==================
            case DRIVE_PATH4:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 2.5) {
                    intake.setPower(1);
                    setPathState(PathState.WAIT_INTAKE2);
                }
                break;
            case WAIT_INTAKE2:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(path5, true);
                    setPathState(PathState.DRIVE_PATH5);
                }
                break;
            case DRIVE_PATH5:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 2.5) {
                    intake.setPower(0);
                    setPathState(PathState.SPIN_UP3);
                }
                break;
            case SPIN_UP3:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    ballsShot = 0;
                    setPathState(PathState.SHOOT3);
                }
                break;
            case SHOOT3:
                if (!shooting && ballsShot == 0) {
                    intake.setPower(1);
                    kicker.setPosition(0.25);
                    shooting = true;
                    kickTimer.resetTimer();
                    ballsShot = 1;
                }
                if (shooting && kickTimer.getElapsedTimeSeconds() > 0.16) {
                    kicker.setPosition(0.31);
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.56) {
                    shooting = false;
                    ballsShot = 0;
                    intake.setPower(1);
                    follower.followPath(path6, true);
                    setPathState(PathState.INTAKECLOSE3);
                }
                break;
            case INTAKECLOSE3:
                kicker.setPosition(0.15);
                setPathState(PathState.DRIVE_PATH6);
                break;

            // ================== CYCLE 3 ==================
            case DRIVE_PATH6:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 2.5) {
                    intake.setPower(1);
                    setPathState(PathState.WAIT_INTAKE3);
                }
                break;
            case WAIT_INTAKE3:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(path7, true);
                    setPathState(PathState.DRIVE_PATH7);
                }
                break;
            case DRIVE_PATH7:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 2.5) {
                    intake.setPower(0);
                    setPathState(PathState.SPIN_UP4);
                }
                break;
            case SPIN_UP4:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    ballsShot = 0;
                    setPathState(PathState.SHOOT4);
                }
                break;
            case SHOOT4:
                if (!shooting && ballsShot == 0) {
                    intake.setPower(1);
                    kicker.setPosition(0.25);
                    shooting = true;
                    kickTimer.resetTimer();
                    ballsShot = 1;
                }
                if (shooting && kickTimer.getElapsedTimeSeconds() > 0.16) {
                    kicker.setPosition(0.31);
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.56) {
                    shooting = false;
                    ballsShot = 0;
                    intake.setPower(1);
                    follower.followPath(path8, true);
                    setPathState(PathState.INTAKECLOSE4);
                }
                break;
            case INTAKECLOSE4:
                kicker.setPosition(0.15);
                setPathState(PathState.DRIVE_PATH8);
                break;

            // ================== FINAL OUT & PARK ==================
            case DRIVE_PATH8:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 2.5) {
                    intake.setPower(1);
                    setPathState(PathState.WAIT_INTAKE4);
                }
                break;
            case WAIT_INTAKE4:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(path9, true);
                    setPathState(PathState.DRIVE_PATH9);
                }
                break;
            case DRIVE_PATH9:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 2.5) {
                    setPathState(PathState.DONE);
                }
                break;

            // ================== DONE ==================
            case DONE:
                shooter1.setVelocity(0);
                shooter2.setVelocity(0);
                intake.setPower(0);
                kicker.setPosition(0.31);
                turretController.setTargetAngle(90.0);
                turretController.update();
                telemetry.addLine("Done all Paths");
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
        follower.setMaxPower(1.0);

        shooter1 = hardwareMap.get(DcMotorEx.class, "Shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "Shooter2");
        intake = hardwareMap.get(DcMotor.class, "Intake");
        kicker = hardwareMap.get(Servo.class, "Kicker");
        turretController = new TurretControllerRED(hardwareMap, "Turret");

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);

        PIDFCoefficients pidf = new PIDFCoefficients(125, 0, 0, 26);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        kicker.setPosition(0.15); // Retracted
        buildPaths();
        pathState = PathState.DRIVE_PATH1;
        shooting = false;
        lastTime = System.currentTimeMillis();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        pathTimer.resetTimer();
        shooter1.setVelocity(shootVelocity);
        shooter2.setVelocity(shootVelocity);
    }

    @Override
    public void loop() {
        follower.update();

        // 1. Maintain Flywheels constantly
        shooter1.setVelocity(shootVelocity);
        shooter2.setVelocity(shootVelocity);

        // 2. Turret Vector Prediction
        Pose currentPose = follower.getPose();
        long currentTime = System.currentTimeMillis();
        Pose velocity;

        if (lastPose == null || lastTime == 0) {
            velocity = new Pose(0, 0, 0);
        } else {
            double dt = (currentTime - lastTime) / 1000.0;
            if (dt > 0.005) {
                double velX = (currentPose.getX() - lastPose.getX()) / dt;
                double velY = (currentPose.getY() - lastPose.getY()) / dt;
                double diffHeading = currentPose.getHeading() - lastPose.getHeading();
                while (diffHeading > Math.PI)  diffHeading -= 2 * Math.PI;
                while (diffHeading < -Math.PI) diffHeading += 2 * Math.PI;
                double velHeading = diffHeading / dt;
                velocity = new Pose(velX, velY, velHeading);
            } else {
                velocity = new Pose(0, 0, 0);
            }
        }

        lastPose = new Pose(currentPose.getX(), currentPose.getY(), currentPose.getHeading());
        lastTime = currentTime;

        // 3. Actively Aim Turret
        if (pathState != PathState.DONE) {
            turretController.aimAtGoalWithPredictionRED(currentPose, velocity);
        }

        // 4. Update the Drivetrain and Shooter State Machine
        statePathUpdate();

        // 5. Telemetry Logs
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("Shooter 1 Vel", shooter1.getVelocity());
        panelsTelemetry.debug("Shooter 2 Vel", shooter2.getVelocity());
        panelsTelemetry.debug("Balls Shot Phase", ballsShot);
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void stop() {
        Pose finalPose = follower.getPose();
        org.firstinspires.ftc.teamcode.DecodeBot.PedroPose.saveCurrentPose(finalPose);
        org.firstinspires.ftc.teamcode.DecodeBot.PedroPose.saveTurretTicks(turretController.getRawTicks());
        super.stop();
    }
}