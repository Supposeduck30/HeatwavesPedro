package org.firstinspires.ftc.teamcode.SumoRobot.SUMOAUTOS;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
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
public class NBlue9BallFar extends OpMode {

    // Mechanisms
    private DcMotorEx shooter1, shooter2;
    private DcMotor intake;
    private Servo kicker;
    private DcMotorEx turret;

    private static final int TURRET_TARGET_TICKS = 99;
    public double shootVelocity = 1765;

    // Pathing
    private Follower follower;
    private Timer pathTimer, kickTimer;

    private boolean shooting = false;
    private int ballsShot = 0;

    public enum PathState {
        DRIVE_TO_SHOOT1,
        SPIN_UP1,
        SHOOT1,
        INTAKECLOSE1,
        DRIVE_TO_STACK,
        INTAKING_STACK,
        WAIT_FOR_INTAKE_STACK,
        DRIVE_TO_SHOOT2,
        SPIN_UP2,
        SHOOT2,
        INTAKECLOSE2,
        DRIVE_TO_HP,
        INTAKING_HP,
        WAIT_FOR_INTAKE_HP,
        DRIVE_TO_SHOOT3,
        SPIN_UP3,
        SHOOT3,
        INTAKECLOSE3,
        DRIVE_TO_END,
        DONE
    }

    private PathState pathState;

    // Poses
    private final Pose startPose = new Pose(53.4,7.7,Math.toRadians(180));
    private final Pose shootPose1 = new Pose(55.1,15.9, Math.toRadians(180));
    private final Pose collectRow1 = new Pose(56,36, Math.toRadians(180));
    private final Pose takeRow1 = new Pose(14.4,35.7, Math.toRadians(180));
    private final Pose shootPose2 = new Pose(55.1,15.9, Math.toRadians(180));
    private final Pose collectRow2 = new Pose(7.2,41.3, Math.toRadians(270));
    private final Pose takeRow2 = new Pose(7,9.7, Math.toRadians(270));
    private final Pose shootPose3 = new Pose(55.1,15.9, Math.toRadians(180));
    private final Pose endPose = new Pose(17.2,16, Math.toRadians(180));

    private PathChain path1, pathCollect1, pathTake1, path2,
            pathCollect2, pathTake2, path3, pathEnd;

    private void buildPaths() {
        // TURN 90 DEGREES TO 180
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose1.getHeading())
                .build();

        // DRIVE TO STACK LINE (Hold 180)
        pathCollect1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose1, collectRow1))
                .setConstantHeadingInterpolation(shootPose1.getHeading())
                .build();

        // SWEEP THROUGH STACK (Hold 180)
        pathTake1 = follower.pathBuilder()
                .addPath(new BezierLine(collectRow1, takeRow1))
                .setConstantHeadingInterpolation(collectRow1.getHeading())
                .build();

        // RETURN TO SHOOT (Hold 180)
        path2 = follower.pathBuilder()
                .addPath(new BezierLine(takeRow1, shootPose2))
                .setConstantHeadingInterpolation(takeRow1.getHeading())
                .build();

        // DRIVE TO HUMAN PLAYER (Turn 180 to 270)
        pathCollect2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose2, collectRow2))
                .setLinearHeadingInterpolation(shootPose2.getHeading(), collectRow2.getHeading())
                .build();

        // SWEEP HP (Hold 270)
        pathTake2 = follower.pathBuilder()
                .addPath(new BezierLine(collectRow2, takeRow2))
                .setConstantHeadingInterpolation(collectRow2.getHeading())
                .build();

        // RETURN TO SHOOT (Turn 270 to 180)
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(takeRow2, shootPose3))
                .setLinearHeadingInterpolation(takeRow2.getHeading(), shootPose3.getHeading())
                .build();

        // PARK (Hold 180)
        pathEnd = follower.pathBuilder()
                .addPath(new BezierLine(shootPose3, endPose))
                .setConstantHeadingInterpolation(shootPose3.getHeading())
                .build();
    }

    private void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    private void stateMachine() {
        switch (pathState) {
            case DRIVE_TO_SHOOT1:
                follower.followPath(path1, true);
                setPathState(PathState.SPIN_UP1);
                break;

            case SPIN_UP1:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3.5) {
                    ballsShot = 0;
                    setPathState(PathState.SHOOT1);
                }
                break;

            case SHOOT1:
                handleShooting(PathState.INTAKECLOSE1, pathCollect1);
                break;
            case INTAKECLOSE1:
                kicker.setPosition(0.15);
                setPathState(PathState.DRIVE_TO_STACK);
                break;
            case DRIVE_TO_STACK:
                if (!follower.isBusy()) {
                    // Start Intake and follow the sweep path SLOWLY (0.6 speed) so it actually grabs them
                    intake.setPower(1);
                    follower.followPath(pathTake1, 0.6, true);
                    setPathState(PathState.INTAKING_STACK);
                }
                break;

            case INTAKING_STACK:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 5) {
                    // Done driving over balls. Wait 0.8 seconds to let them travel up the ramp!
                    setPathState(PathState.WAIT_FOR_INTAKE_STACK);
                }
                break;

            case WAIT_FOR_INTAKE_STACK:
                if (pathTimer.getElapsedTimeSeconds() > 0.8) {
                    intake.setPower(0);
                    follower.followPath(path2, false);
                    setPathState(PathState.DRIVE_TO_SHOOT2);
                }
                break;

            case DRIVE_TO_SHOOT2:
                if (!follower.isBusy()) {
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
                handleShooting(PathState.INTAKECLOSE2, pathCollect2);
                break;
            case INTAKECLOSE2:
                kicker.setPosition(0.15);
                setPathState(PathState.DRIVE_TO_HP);
                break;
            case DRIVE_TO_HP:
                if (!follower.isBusy()) {
                    // Start Intake and follow HP sweep path SLOWLY
                    intake.setPower(1);
                    follower.followPath(pathTake2, 0.8, false);
                    setPathState(PathState.INTAKING_HP);
                }
                break;

            case INTAKING_HP:
                if (!follower.isBusy()) {
                    // Wait for balls to travel up ramp
                    setPathState(PathState.WAIT_FOR_INTAKE_HP);
                }
                break;

            case WAIT_FOR_INTAKE_HP:
                if (pathTimer.getElapsedTimeSeconds() > 0.8) {
                    intake.setPower(0);
                    follower.followPath(path3, false);
                    setPathState(PathState.DRIVE_TO_SHOOT3);
                }
                break;

            case DRIVE_TO_SHOOT3:
                if (!follower.isBusy()) {
                    setPathState(PathState.SPIN_UP3);
                }
                break;

            case SPIN_UP3:
                if (pathTimer.getElapsedTimeSeconds() > 1.0) {
                    ballsShot = 0;
                    setPathState(PathState.SHOOT3);
                }
                break;

            case SHOOT3:
                handleShooting(PathState.INTAKECLOSE3, pathEnd);
                break;
            case INTAKECLOSE3:
                kicker.setPosition(0.15);

                setPathState(PathState.DRIVE_TO_END);
                break;
            case DRIVE_TO_END:
                if (!follower.isBusy()) {
                    setPathState(PathState.DONE);
                }
                break;

            case DONE:
                intake.setPower(0);
                shooter1.setVelocity(0);
                shooter2.setVelocity(0);
                kicker.setPosition(0.31);
                turret.setPower(0.5);
                telemetry.addLine("Auto Complete");
                break;
        }
    }

    private void handleShooting(PathState nextState, PathChain nextPath) {
        if (!shooting && pathTimer.getElapsedTimeSeconds() > 0.3) {
            intake.setPower(1);
            kicker.setPosition(0.25);
            shooting = true;
            kickTimer.resetTimer();
        } else if (shooting && kickTimer.getElapsedTimeSeconds() > 0.16) {
            kicker.setPosition(0.31);
            shooting = false;
            ballsShot++;

            if (ballsShot >= 3) {
                ballsShot = 0;
                intake.setPower(1);
                follower.followPath(nextPath, true);
                setPathState(nextState);
            } else {
                pathTimer.resetTimer();
            }
        }
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);

        shooter1 = hardwareMap.get(DcMotorEx.class, "Shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "Shooter2");
        intake = hardwareMap.get(DcMotor.class, "Intake");
        kicker = hardwareMap.get(Servo.class, "Kicker");
        turret = hardwareMap.get(DcMotorEx.class, "Turret");

        turret.setDirection(DcMotorSimple.Direction.REVERSE);

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(TURRET_TARGET_TICKS);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(0.5);

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);

        PIDFCoefficients pidf = new PIDFCoefficients(120,0,0,25);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        pathTimer = new Timer();
        kickTimer = new Timer();

        kicker.setPosition(0.15);

        buildPaths();
        pathState = PathState.DRIVE_TO_SHOOT1;
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        shooter1.setVelocity(shootVelocity);
        shooter2.setVelocity(shootVelocity);
    }

    @Override
    public void loop() {
        follower.update();

        // Keep velocity held high during auto
        shooter1.setVelocity(shootVelocity);
        shooter2.setVelocity(shootVelocity);

        stateMachine();

        telemetry.addData("State", pathState);
        telemetry.addData("Balls Shot", ballsShot);
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void stop() {
        // --- ADDED CRITICAL MEMORY SAVES FOR TELEOP ---
        Pose finalPose = follower.getPose();
        org.firstinspires.ftc.teamcode.SumoRobot.PedroPose.saveCurrentPose(finalPose);
        org.firstinspires.ftc.teamcode.SumoRobot.PedroPose.saveTurretTicks(turret.getCurrentPosition());
        super.stop();
    }
}