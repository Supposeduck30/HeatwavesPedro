package org.firstinspires.ftc.teamcode;

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


@Autonomous(name = "RedSideAutoPedro", group = "Pedro_Autonomous")
@Configurable
public class RedSideAutoPedro extends OpMode {

    private TelemetryManager panelsTelemetry;
    //mechanisms
    private DcMotor shooter1 = null;
    private DcMotor shooter2 = null;
    private Servo intake = null;



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

    private final Pose startPose = new Pose(119.32784184514004,127.15650741350906,Math.toRadians(37));
    private final Pose shootPose = new Pose(74.25370675453047,76.38879736408568, Math.toRadians(42));
    private final Pose endPose = new Pose(86.58978583196047,114.82042833607908, Math.toRadians(90));
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
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5){
                    // TODO add logic to flywheel shooter'
                    follower.followPath(driveShootPosEndPos,true);
                    setPathState(PathState.SHOOT_END);
                    shoot();
                    sleep(500);
                    shoot();
                    sleep(500);
                    shoot();
                    shooter1.setPower(0.0);
                    shooter2.setPower(0.0);

                }
                break;
            case SHOOT_END:
                // all done
                if (!follower.isBusy()){
                    telemetry.addLine("Done all Paths :)");
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
        shooter1 = hardwareMap.get(DcMotor.class, "SM1");
        shooter2 = hardwareMap.get(DcMotor.class, "SM2");
        intake = hardwareMap.get(Servo.class, "Intake");

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
        shooter1.setPower(-0.55);
        shooter2.setPower(0.55);
        sleep(1200);
        intake.setPosition(0.2);
        sleep(500);
        shooter1.setPower(0.1);
        shooter2.setPower(-0.1);
        sleep(400);
        intake.setPosition(1.0);
    }


}
