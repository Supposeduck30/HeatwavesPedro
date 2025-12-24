package org.firstinspires.ftc.teamcode.InfernoRobot.SUMOAUTOS;

import static java.lang.Thread.sleep;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
@Disabled
@Configurable
public class BlueThreeBallAuto extends OpMode {

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
        SHOOT_END
    }

    PathState pathState;

    private final Pose startPose = new Pose(21.825370675453048,122.88632619439869,Math.toRadians(145));
    private final Pose shootPose = new Pose(59.54530477759472,85.4036243822076, Math.toRadians(138));
    private final Pose endPose = new Pose(60.25700164744645,112.92257001647447, Math.toRadians(90));
    private PathChain driveStartPosShootPos, driveShootPosEndPos;

    public void buildPaths(){
        // put in coordinates start pos and end pos
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose,shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        driveShootPosEndPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
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
                    follower.followPath(driveShootPosEndPos,true);
                    setPathState(PathState.SHOOT_END);
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
