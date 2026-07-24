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

import org.firstinspires.ftc.teamcode.SumoRobot.TurretControllerRED;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name="CRIFarRed", group="Autonomous")
@Configurable
public class CRIAutoFarBlue extends OpMode {
    private TelemetryManager panelsTelemetry;

    // Hardware Mechanisms
    private DcMotorEx shooter1;
    private DcMotorEx shooter2;
    private DcMotor intake;
    private Servo kicker;

    // Live Turret Controller
    private TurretControllerRED turretController;
    public double shootVelocity = 2000; // Updated to 1900

    // Software & Tracking
    public Follower follower;
    private Timer stateTimer, pauseTimer, globalTimer;
    private Paths paths;

    // Pose and Velocity Tracking for Turret Prediction
    private Pose lastPose = null;
    private long lastTime = 0;

    // State Machine Architecture
    private int pathIndex = 0;
    private int ballsShot = 0;

    private enum AutoState {
        START,
        DRIVING,
        SHOOTING,
        DONE
    }
    private AutoState currentState = AutoState.START;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        stateTimer = new Timer();
        pauseTimer = new Timer();
        globalTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(111.600, 8.700, Math.toRadians(90)));

        // Set max power directly on the follower for the new PedroPathing API
        follower.setMaxPower(1.0);

        paths = new Paths(follower);

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

        ballsShot = 0;
        pathIndex = 0;
        lastTime = System.currentTimeMillis();
        currentState = AutoState.START;

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        stateTimer.resetTimer();
        globalTimer.resetTimer();

        // Turn on flywheels immediately upon pressing start
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
        if (currentState != AutoState.DONE) {
            turretController.aimAtGoalWithPredictionRED(currentPose, velocity);
        }

        // 4. Update Route Sequence & Intakes/Shooters
        autonomousPathUpdate();

        // 5. Telemetry Logs
        panelsTelemetry.debug("State Machine", currentState);
        panelsTelemetry.debug("Path Index", pathIndex);
        panelsTelemetry.debug("Shooter 1 Vel", shooter1.getVelocity());
        panelsTelemetry.debug("Shooter 2 Vel", shooter2.getVelocity());
        panelsTelemetry.debug("Balls Fired Sequences", ballsShot);
        panelsTelemetry.update(telemetry);
    }

    private void autonomousPathUpdate() {
        switch (currentState) {
            case START:
                follower.followPath(paths.autoPaths[pathIndex], true);
                currentState = AutoState.DRIVING;
                break;

            case DRIVING:
                if (pathIndex % 2 == 1) {
                    intake.setPower(1.0);
                }

                if (!follower.isBusy()) {
                    if (pathIndex == 10) {
                        currentState = AutoState.DONE;
                    } else if (pathIndex % 2 == 0) {
                        currentState = AutoState.SHOOTING;
                        pauseTimer.resetTimer();
                    } else {
                        pathIndex++;
                        follower.followPath(paths.autoPaths[pathIndex], true);
                    }
                }
                break;

            case SHOOTING:
                // SAFETY CHECK: DYNAMIC VELOCITY VERIFICATION
                if (ballsShot == 0) {
                    double currentVel1 = Math.abs(shooter1.getVelocity());
                    double currentVel2 = Math.abs(shooter2.getVelocity());

                    boolean notUpToSpeed = (currentVel1 < shootVelocity - 100) || (currentVel2 < shootVelocity - 100);

                    if (notUpToSpeed && globalTimer.getElapsedTimeSeconds() < 3.0) {
                        intake.setPower(0.0);
                        kicker.setPosition(0.15); // Closed
                        pauseTimer.resetTimer();
                        break;
                    }
                }

                double elapsedPauseTime = pauseTimer.getElapsedTimeSeconds();

                // Increased time to 1.20s to ensure all 3 balls feed and clear properly
                if (elapsedPauseTime < 1.20) {
                    // Open kicker IMMEDIATELY, run intake to feed all 3 balls
                    kicker.setPosition(0.25); // Open
                    intake.setPower(1.0);
                }
                else {
                    // Close the blocker cleanly at the direct end of the shooting cycle
                    kicker.setPosition(0.15); // Closed

                    ballsShot++;
                    pathIndex++;
                    follower.followPath(paths.autoPaths[pathIndex], true);
                    currentState = AutoState.DRIVING;
                }
                break;

            case DONE:
                shooter1.setVelocity(0);
                shooter2.setVelocity(0);
                intake.setPower(0);
                kicker.setPosition(0.31);
                turretController.setTargetAngle(90.0);
                turretController.update();
                break;
        }
    }

    @Override
    public void stop() {
        Pose finalPose = follower.getPose();
        org.firstinspires.ftc.teamcode.SumoRobot.PedroPose.saveCurrentPose(finalPose);
        org.firstinspires.ftc.teamcode.SumoRobot.PedroPose.saveTurretTicks(turretController.getRawTicks());
        super.stop();
    }

    // Path array construction
    public static class Paths {
        public PathChain[] autoPaths = new PathChain[11];

        public Paths(Follower follower) {
            autoPaths[0] = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(111.600, 8.700), new Pose(113.000, 43.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                    .build();

            autoPaths[1] = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(113.000, 43.000), new Pose(116.900, 62.100), new Pose(170.300, 58.700)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            autoPaths[2] = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(170.300, 58.700), new Pose(113.000, 43.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            autoPaths[3] = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(113.000, 43.000), new Pose(178.700, 31.400), new Pose(181.000, 76.500)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                    .build();

            autoPaths[4] = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(181.000, 76.500), new Pose(113.000, 43.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                    .build();

            autoPaths[5] = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(113.000, 43.000), new Pose(178.700, 31.400), new Pose(181.000, 76.500)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                    .build();

            autoPaths[6] = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(181.000, 76.500), new Pose(113.000, 43.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                    .build();

            autoPaths[7] = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(113.000, 43.000), new Pose(178.700, 31.200), new Pose(181.000, 76.500)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                    .build();

            autoPaths[8] = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(181.000, 76.500), new Pose(113.000, 43.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                    .build();

            autoPaths[9] = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(113.000, 43.000), new Pose(178.700, 31.200), new Pose(181.000, 76.500)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                    .build();

            autoPaths[10] = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(181.000, 76.500), new Pose(105.900, 38.400)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                    .build();
        }
    }
}