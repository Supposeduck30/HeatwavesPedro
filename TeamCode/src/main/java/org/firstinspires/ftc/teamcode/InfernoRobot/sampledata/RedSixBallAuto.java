package org.firstinspires.ftc.teamcode.InfernoRobot.sampledata;

import static java.lang.Thread.sleep;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
@Configurable
public class RedSixBallAuto extends OpMode {

    private TelemetryManager panelsTelemetry;
    //mechanisms
    private DcMotor shooter;
    private DcMotor intake;
    private Servo kicker;



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
        SHOOT_END
    }

    PathState pathState;

    private final Pose startPose = new Pose(119.32784184514004,127.15650741350906,Math.toRadians(37));
    private final Pose shootPose = new Pose(74.96540362438222,91.334431630972, Math.toRadians(42));
    private final Pose collectRow1 = new Pose(47.92092257001647,83.50576606260297, Math.toRadians(180));
    private final Pose takeRow1 = new Pose(18.978583196046127,83.98023064250413, Math.toRadians(180));
    private final Pose shootpose1 = new Pose(54.32619439868205,90.85996705107084, Math.toRadians(138));
    private final Pose endPose = new Pose(54.088962108731465,116.7182866556837, Math.toRadians(270));
    private PathChain driveStartPosShootPos, shootPoseCollectPose, collectPoseTakePose,takePoseShootPose, shootPoseEndPos;

    public void buildPaths(){
        // put in coordinates start pos and end pos
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose,shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        shootPoseCollectPose = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, collectRow1))
                .setLinearHeadingInterpolation(shootPose.getHeading(), collectRow1.getHeading())
                .build();
        collectPoseTakePose = follower.pathBuilder()
                .addPath(new BezierLine(collectRow1, takeRow1))
                .setLinearHeadingInterpolation(collectRow1.getHeading(), takeRow1.getHeading())
                .build();
        takePoseShootPose = follower.pathBuilder()
                .addPath(new BezierLine(takeRow1,shootpose1))
                .setLinearHeadingInterpolation(takeRow1.getHeading(), shootpose1.getHeading())
                .build();
        shootPoseEndPos = follower.pathBuilder()
                .addPath(new BezierLine(shootpose1, endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(),endPose.getHeading())
                .build();
    }


    public void statePathUpdate() throws InterruptedException {
        switch (pathState){
            case DRIVE_START_SHOOT_POS:
                follower.followPath(driveStartPosShootPos,true);
                setPathState(PathState.SHOOT_PRELOAD); //reset timer and make new state
                break;
            case SHOOT_PRELOAD:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2){
                    // TODO add logic to flywheel shooter'
                    follower.followPath(driveStartPosShootPos,true);
                    setPathState(PathState.ROW1COLLECT);
                    shooter.setPower(0.85);
                    sleep(500);
                    shoot();
                    sleep(500);
                    shoot();
                    sleep(500);
                    shoot();
                    sleep(100);
                    shooter.setPower(0.0);
                    intake.setPower(0.0);

                }
                break;
            case ROW1COLLECT:
                follower.followPath(shootPoseCollectPose,true);
                setPathState(PathState.INTAKEON);
                break;
            case INTAKEON:
                intake.setPower(0.8);
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
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2){
                    // TODO add logic to flywheel shooter'
                    follower.followPath(shootPoseEndPos,true);
                    setPathState(PathState.SHOOT_END);
                    shooter.setPower(0.85);
                    sleep(500);
                    shoot();
                    sleep(500);
                    shoot();
                    sleep(500);
                    shoot();
                    sleep(100);
                    shooter.setPower(0.0);
                    intake.setPower(0.0);
                }
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
        shooter = hardwareMap.get(DcMotor.class, "Shooter");
        intake  = hardwareMap.get(DcMotor.class, "Intake");
        kicker  = hardwareMap.get(Servo.class, "Kicker");

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
        shooter.setPower(0.85 );
        intake.setPower(0.6);
        sleep(500);
        kicker.setPosition(0.6);
        sleep(500);
        kicker.setPosition(0.31);
    }


}
