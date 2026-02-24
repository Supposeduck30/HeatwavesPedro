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
public class Blue9BallFar extends OpMode {

    // Mechanisms
    private DcMotorEx shooter1, shooter2;
    private DcMotor intake;
    private Servo kicker;
    private DcMotorEx turret;

    private static final int TURRET_TARGET_TICKS = 90;
    public double shootVelocity = 2200;

    // Pathing
    private Follower follower;
    private Timer pathTimer, kickTimer;

    private boolean shooting = false;
    private int ballsShot = 0;

    public enum PathState {
        DRIVE_TO_SHOOT1,
        SPIN_UP1,
        SHOOT1,
        INTAKE_PATH1,
        DRIVE_TO_SHOOT2,
        SPIN_UP2,
        SHOOT2,
        INTAKE_PATH2,
        DRIVE_TO_SHOOT3,
        SPIN_UP3,
        SHOOT3,
        DRIVE_TO_END,
        DONE
    }

    private PathState pathState;

    // Poses
    private final Pose startPose = new Pose(53,6.6,Math.toRadians(180));
    private final Pose shootPose1 = new Pose(44.4,35.3, Math.toRadians(180));
    private final Pose collectRow1 = new Pose(12.5,35.7, Math.toRadians(180));
    private final Pose takeRow1 = new Pose(55.4,9.8, Math.toRadians(270));
    private final Pose shootPose2 = new Pose(7.7,27, Math.toRadians(270));
    private final Pose collectRow2 = new Pose(99.86,58.32, Math.toRadians(180));
    private final Pose takeRow2 = new Pose(7.4,10.9, Math.toRadians(180));
    private final Pose shootPose3 = new Pose(55.4,9.8, Math.toRadians(180));
    private final Pose endPose = new Pose(31.3,10.2, Math.toRadians(180));

    private PathChain path1, pathCollect1, pathTake1, path2,
            pathCollect2, pathTake2, path3, pathEnd;

    private void buildPaths() {

        path1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose1))
                .build();

        pathCollect1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose1, collectRow1))
                .build();

        pathTake1 = follower.pathBuilder()
                .addPath(new BezierLine(collectRow1, takeRow1))
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierLine(takeRow1, shootPose2))
                .build();

        pathCollect2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose2, collectRow2))
                .build();

        pathTake2 = follower.pathBuilder()
                .addPath(new BezierLine(collectRow2, takeRow2))
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierLine(takeRow2, shootPose3))
                .build();

        pathEnd = follower.pathBuilder()
                .addPath(new BezierLine(shootPose3, endPose))
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
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.0) {
                    ballsShot = 0;
                    setPathState(PathState.SHOOT1);
                }
                break;

            case SHOOT1:
                handleShooting(PathState.INTAKE_PATH1, pathCollect1);
                break;

            case INTAKE_PATH1:
                intake.setPower(1);
                if (!follower.isBusy()) {
                    follower.followPath(pathTake1, true);
                    setPathState(PathState.DRIVE_TO_SHOOT2);
                }
                break;

            case DRIVE_TO_SHOOT2:
                if (!follower.isBusy()) {
                    follower.followPath(path2, true);
                    setPathState(PathState.SPIN_UP2);
                }
                break;

            case SPIN_UP2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.0) {
                    ballsShot = 0;
                    setPathState(PathState.SHOOT2);
                }
                break;

            case SHOOT2:
                handleShooting(PathState.INTAKE_PATH2, pathCollect2);
                break;

            case INTAKE_PATH2:
                intake.setPower(1);
                if (!follower.isBusy()) {
                    follower.followPath(pathTake2, true);
                    setPathState(PathState.DRIVE_TO_SHOOT3);
                }
                break;

            case DRIVE_TO_SHOOT3:
                if (!follower.isBusy()) {
                    follower.followPath(path3, true);
                    setPathState(PathState.SPIN_UP3);
                }
                break;

            case SPIN_UP3:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.0) {
                    ballsShot = 0;
                    setPathState(PathState.SHOOT3);
                }
                break;

            case SHOOT3:
                handleShooting(PathState.DRIVE_TO_END, pathEnd);
                break;

            case DRIVE_TO_END:
                if (!follower.isBusy()) {
                    setPathState(PathState.DONE);
                }
                break;

            case DONE:
                intake.setPower(0);
                kicker.setPosition(0.31);
                telemetry.addLine("Auto Complete");
                break;
        }
    }

    private void handleShooting(PathState nextState, PathChain nextPath) {

        if (!shooting && pathTimer.getElapsedTimeSeconds() > 0.3) {

            intake.setPower(1);
            kicker.setPosition(0.6);
            shooting = true;
            kickTimer.resetTimer();

        } else if (shooting && kickTimer.getElapsedTimeSeconds() > 0.16) {

            kicker.setPosition(0.31);
            shooting = false;
            ballsShot++;

            if (ballsShot >= 3) {
                ballsShot = 0;
                intake.setPower(0);
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

        PIDFCoefficients pidf = new PIDFCoefficients(400,0,0,25);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        pathTimer = new Timer();
        kickTimer = new Timer();

        kicker.setPosition(0.31);

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

        // Blue18 style velocity hold
        shooter1.setVelocity(shootVelocity);
        shooter2.setVelocity(shootVelocity);

        stateMachine();

        telemetry.addData("State", pathState);
        telemetry.addData("Balls Shot", ballsShot);
        telemetry.update();
    }
}
