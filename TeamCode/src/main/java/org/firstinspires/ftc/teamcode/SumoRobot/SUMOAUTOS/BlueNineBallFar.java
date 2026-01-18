package org.firstinspires.ftc.teamcode.SumoRobot.SUMOAUTOS;

import static java.lang.Thread.sleep;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Disabled
@Autonomous
@Configurable
public class BlueNineBallFar extends OpMode {

    private TelemetryManager panelsTelemetry;
    //mechanisms
    private DcMotorEx shooter1;
    private DcMotorEx shooter2;
    private DcMotor intake;
    private Servo kicker;

    public double lowVelocity = 2250;//2100

    //software
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    //States
    public enum PathState{
        //START POSTION_END POSTION
        //DRIVE > MOVEMENT STATE
        //SHOOT > ATTEMPT TO SCORE THE ARTIFACT

        DRIVE_START_SHOOT_POS,
        SHOOT_PRELOAD,
        ROW1COLLECT,
        INTAKEON,
        TAKEROW1,
        SHOOTROW1POSE,
        SHOOTROW1,
        ROW2COLLECT,
        INTAKEON2,
        TAKEROW2,
        SHOOTROW2POSE,
        SHOOTROW2,

        SHOOT_END
    }

    PathState pathState;

    private final Pose startPose = new Pose(54,8,Math.toRadians(90));
    private final Pose shootPose1 = new Pose(60.56000000000002,18.24000000000001, Math.toRadians(119));
    private final Pose collectRow1 = new Pose(47.08,35.28, Math.toRadians(180));
    private final Pose takeRow1 = new Pose(13.599999999999996,35.28, Math.toRadians(180));
    private final Pose shootpose2 = new Pose(60.56000000000002,18.24000000000001, Math.toRadians(119));
    private final Pose collectRow2 = new Pose(46.800000000000026,59.88000000000001, Math.toRadians(180));
    private final Pose takeRow2 = new Pose(12.12,59.88000000000001, Math.toRadians(180));
    private final Pose shootpose3 = new Pose(60.56000000000002 ,18.24000000000001, Math.toRadians(121));
    private final Pose endPose = new Pose(29.60000000000001,65.39999999999999, Math.toRadians(0));
    private PathChain driveStartPosShootPos, shootPoseCollectPose, collectPoseTakePose,takePoseShootPose, shootPoseCollectPose2, collectPose2TakePose2, takePose2ShootPose2, shootPose2EndPos;

    public void buildPaths(){
        // put in coordinates start pos and end pos
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose1.getHeading())
                .build();
        shootPoseCollectPose = follower.pathBuilder()
                .addPath(new BezierLine(shootPose1, collectRow1))
                .setLinearHeadingInterpolation(shootPose1.getHeading(), collectRow1.getHeading())
                .build();
        collectPoseTakePose = follower.pathBuilder()
                .addPath(new BezierLine(collectRow1, takeRow1))
                .setLinearHeadingInterpolation(collectRow1.getHeading(), takeRow1.getHeading())
                .build();
        takePoseShootPose = follower.pathBuilder()
                .addPath(new BezierLine(takeRow1,shootpose2))
                .setLinearHeadingInterpolation(takeRow1.getHeading(), shootpose2.getHeading())
                .build();
        shootPoseCollectPose2 = follower.pathBuilder()
                .addPath(new BezierLine(shootpose2, collectRow2))
                .setLinearHeadingInterpolation(shootpose2.getHeading(), collectRow2.getHeading())
                .build();
        collectPose2TakePose2 = follower.pathBuilder()
                .addPath(new BezierLine(collectRow2, takeRow2))
                .setLinearHeadingInterpolation(collectRow2.getHeading(), takeRow2.getHeading())
                .build();
        takePose2ShootPose2 = follower.pathBuilder()
                .addPath(new BezierLine(takeRow2,shootpose3))
                .setLinearHeadingInterpolation(takeRow2.getHeading(), shootpose3.getHeading())
                .build();
        shootPose2EndPos = follower.pathBuilder()
                .addPath(new BezierLine(shootpose3, endPose))
                .setLinearHeadingInterpolation(shootpose3.getHeading(),endPose.getHeading())
                .build();
    }


    public void statePathUpdate() throws InterruptedException {
        switch (pathState){
            case DRIVE_START_SHOOT_POS:
                follower.followPath(driveStartPosShootPos,true);
                setPathState(PathState.SHOOT_PRELOAD); //reset timer and make new state
                break;
            case SHOOT_PRELOAD:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > .5){
                    // TODO add logic to flywheel shooter'
                    follower.followPath(driveStartPosShootPos,true);
                    setPathState(PathState.ROW1COLLECT);
                    shooter1.setVelocity(lowVelocity);
                    shooter2.setVelocity(lowVelocity);
                    sleep(500);
                    shoot();
                    sleep(500);
                    shoot();
                    sleep(500);
                    shoot();
                    sleep(100);
                    shooter1.setVelocity(0.0);
                    shooter2.setVelocity(0.0);
                    intake.setPower(0.0);

                }
                break;
            case ROW1COLLECT:
                follower.followPath(shootPoseCollectPose,true);
                setPathState(PathState.INTAKEON);
                break;
            case INTAKEON:
                intake.setPower(1);
                setPathState(PathState.TAKEROW1);
                break;
            case TAKEROW1:
                if (!follower.isBusy()) {
                    follower.followPath(collectPoseTakePose);
                    setPathState(PathState.SHOOTROW1POSE);
                }
                break;
            case SHOOTROW1POSE:
                if (!follower.isBusy()) {
                    follower.followPath(takePoseShootPose);
                    setPathState(PathState.SHOOTROW1);
                }
                break;
            case SHOOTROW1:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > .5){
                    // TODO add logic to flywheel shooter'
                    follower.followPath(shootPoseCollectPose2,true);
                    shooter1.setVelocity(lowVelocity);
                    shooter2.setVelocity(lowVelocity);
                    sleep(500);
                    shoot();
                    sleep(500);
                    shoot();
                    sleep(500);
                    shoot();
                    sleep(100);
                    shooter1.setVelocity(0.0);
                    shooter2.setVelocity(0.0);
                    intake.setPower(0.0);
                    setPathState(PathState.ROW2COLLECT);
                }
                break;
            case ROW2COLLECT:
                if (!follower.isBusy()) {
                    follower.followPath(shootPoseCollectPose2,true);
                    setPathState(PathState.INTAKEON2);

                }
                break;
            case INTAKEON2:
                intake.setPower(1);
                setPathState(PathState.TAKEROW2);
                break;
            case TAKEROW2:
                if (!follower.isBusy()) {
                    follower.followPath(collectPose2TakePose2);
                    setPathState(PathState.SHOOTROW2POSE);
                }
                break;
            case SHOOTROW2POSE:
                if (!follower.isBusy()) {
                    follower.followPath(takePose2ShootPose2);
                    setPathState(PathState.SHOOTROW2);
                }
                break;
            case SHOOTROW2:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > .5){
                    // TODO add logic to flywheel shooter'
                    follower.followPath(shootPose2EndPos,true);
                    shooter1.setVelocity(lowVelocity);
                    shooter2.setVelocity(lowVelocity);
                    sleep(500);
                    shoot();
                    sleep(500);
                    shoot();
                    sleep(500);
                    shoot();
                    sleep(100);
                    shooter1.setVelocity(0.0);
                    shooter2.setVelocity(0.0);
                    intake.setPower(0.0);
                    intake.setPower(0.0);
                    setPathState(PathState.SHOOT_END);
                }
                break;
            case SHOOT_END:
                // all done
                if (!follower.isBusy()){
                    telemetry.addLine("Done all Paths");
                }
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
        pathState = PathState.DRIVE_START_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        /*opModeTimer.resetTimer()*/;
        follower = Constants.createFollower(hardwareMap);
        // Todo add in other init mechanisms
        // Other motors
        shooter1 = hardwareMap.get(DcMotorEx.class, "Shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class,"Shooter2");
        intake  = hardwareMap.get(DcMotor.class, "Intake");
        kicker  = hardwareMap.get(Servo.class, "Kicker");

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(50,0,0,15);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);

        kicker.setPosition(0.31);
        buildPaths();
        follower.setPose(startPose);
    }

    public void start(){
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        telemetry.update();
        try {
            statePathUpdate();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }


    }

    public void shoot() throws InterruptedException {
        shooter1.setVelocity(lowVelocity);
        shooter2.setVelocity(lowVelocity);
        intake.setPower(1);
        sleep(550);
        kicker.setPosition(0.6);
        intake.setPower(0.32);
        sleep(500);
        kicker.setPosition(0.31);
    }

    public void shootLast() throws InterruptedException {

    }

}
