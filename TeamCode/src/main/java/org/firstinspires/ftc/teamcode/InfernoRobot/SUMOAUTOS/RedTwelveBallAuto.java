package org.firstinspires.ftc.teamcode.InfernoRobot.SUMOAUTOS;

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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
@Disabled
@Configurable
public class RedTwelveBallAuto extends OpMode {

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
        ROW2COLLECT,
        INTAKEON2,
        TAKEROW2,
        SHOOTROW2POSE,
        SHOOTROW2,
        ROW3COLLECT,
        INTAKEON3,
        TAKEROW3,
        SHOOTROW3POSE,
        SHOOTROW3,

        SHOOT_END
    }

    PathState pathState;

    private final Pose startPose = new Pose(123.83525535420098,127.15650741350906,Math.toRadians(37));
    private final Pose shootPose1 = new Pose(89.91103789126852,91.09719934102142, Math.toRadians(42));
    private final Pose collectRow1 = new Pose(98.92586490939044,86.35255354200987, Math.toRadians(0));
    private final Pose takeRow1 = new Pose(128.34266886326193,83.26853377265239, Math.toRadians(0));
    private final Pose shootpose2 = new Pose(91.80889621087314,92.52059308072488, Math.toRadians(42));
    private final Pose collectRow2 = new Pose(100.5864909390445,61.680395387149915, Math.toRadians(0));
    private final Pose takeRow2 = new Pose(131.9011532125206,59.78253706754529, Math.toRadians(0));
    private final Pose shootpose3 = new Pose(87.53871499176277,86.82701812191105, Math.toRadians(42));
    private final Pose collectRow3 = new Pose(101.77265238879737,35.58484349258651, Math.toRadians(0));
    private final Pose takeRow3 = new Pose(133.32454695222407,35.58484349258651, Math.toRadians(0));
    private final Pose shootpose4 = new Pose(87.77594728171334, 91.80889621087314, Math.toRadians(42));

    private final Pose endPose = new Pose(85.64085667215816,121.22570016474465, Math.toRadians(270));
    private PathChain driveStartPosShootPos, shootPoseCollectPose, collectPoseTakePose,takePoseShootPose,
            shootPoseCollectPose2, collectPose2TakePose2, takePose2ShootPose2, shootPose2CollectPose3,
            CollectPose3TakePos3, takePose3ShootPose4, shootPose3EndPos;

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
        shootPose2CollectPose3 = follower.pathBuilder()
                .addPath(new BezierLine(shootpose3, collectRow3))
                .setLinearHeadingInterpolation(shootpose3.getHeading(),collectRow3.getHeading())
                .build();

        CollectPose3TakePos3 = follower.pathBuilder()
                .addPath(new BezierLine(collectRow3,takeRow3))
                .setLinearHeadingInterpolation(collectRow3.getHeading(),takeRow3.getHeading())
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
                    shooter.setPower(0.9);
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
                    follower.followPath(shootPoseCollectPose2,true);
                    shooter.setPower(0.9);
                    sleep(500);
                    shoot();
                    sleep(500);
                    shoot();
                    sleep(500);
                    shoot();
                    sleep(100);
                    shooter.setPower(0.0);
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
                intake.setPower(0.8);
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
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2){
                    // TODO add logic to flywheel shooter'
                    follower.followPath(shootPose2CollectPose3,true);
                    shooter.setPower(0.9);
                    sleep(500);
                    shoot();
                    sleep(500);
                    shoot();
                    sleep(500);
                    shoot();
                    sleep(100);
                    shooter.setPower(0.0);
                    intake.setPower(0.0);
                    setPathState(PathState.SHOOT_END);
                }
                break;
            case ROW3COLLECT:
                if (!follower.isBusy()) {
                    follower.followPath(shootPose2CollectPose3, true);
                    setPathState(PathState.INTAKEON3);
                }
                break;
            case INTAKEON3:
                intake.setPower(0.8);
                setPathState(PathState.TAKEROW3);
                break;
            case TAKEROW3:
                if (!follower.isBusy()) {
                    follower.followPath(CollectPose3TakePos3, true);
                    setPathState(PathState.SHOOTROW2POSE);
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
        shooter.setPower(0.9);
        intake.setPower(0.6);
        sleep(500);
        kicker.setPosition(0.6);
        sleep(500);
        kicker.setPosition(0.31);
    }


}
