 package org.firstinspires.ftc.teamcode.SumoRobot.SUMOAUTOS;

import static java.lang.Thread.sleep;

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
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


 @Autonomous
 @Configurable
 public class Red12BallClose extends OpMode {
     private TelemetryManager panelsTelemetry;

     // Mechanisms
     private DcMotorEx shooter1;
     private DcMotorEx shooter2;
     private DcMotor intake;
     private Servo kicker;

     public double shootVelocity = 1460;
     public double idleVelocity = 1000;
     public double firstSpinUpTime = 0.55;

     // Software
     public Follower follower;
     private Timer pathTimer, opModeTimer, shootTimer;

     // Shooting state
     private int shotsRemaining = 0;
     private boolean isKicking = false;
     private boolean isFirstShot = true;
     private boolean shotsFired = false;
     private int ballsShot = 0;

     // States
     public enum PathState {
         DRIVE_PATH1,
         SHOOT_PATH_WAIT,
         SHOOT,
         SPIN_UP,
         SHOOT_3,
         SHOOT_PATH1,
         DRIVE_PATH2,
         DRIVE_PATH3,
         INTAKE_ON_PATH3,
         DRIVE_PATH4,
         DRIVE_PATH5,
         SHOOT_PATH5,
         DRIVE_PATH6,
         DRIVE_PATH7,
         INTAKE_ON_PATH7,
         DRIVE_PATH8,
         SHOOT_PATH8,
         DRIVE_PATH9,
         DRIVE_PATH10,
         DRIVE_PATH11,
         SHOOT_PATH11,
         DRIVE_PATH12,
         DONE,
         SPIN_UP2,
         SHOOT2,
         SPIN_UP3,
         SHOOT3,
         SPIN_UP4,
         SHOOT4,
     }

     PathState pathState;

     // Poses from your original paths
     private final Pose startPose = new Pose(119.4, 126.7, Math.toRadians(45));

     private final Pose pose1End = new Pose(88.2, 89.9, Math.toRadians(42));

     private final Pose pose2End = new Pose(95.7, 61.6, Math.toRadians(0));

     private final Pose pose3End = new Pose(128.8, 61.6, Math.toRadians(0));

     private final Pose bezierIntakeToGate = new Pose(109.8, 67.1, Math.toRadians(0));

     private final Pose pose4End = new Pose(128, 71.5, Math.toRadians(0));

     private final Pose bezierGateToShoot = new Pose(82.6, 61.3, Math.toRadians(0));

     private final Pose pose5End = new Pose(88.2, 89.9, Math.toRadians(42));

     private final Pose pose6End = new Pose(96.0, 83.5, Math.toRadians(0));

     private final Pose pose7End = new Pose(129.1, 83.5, Math.toRadians(0));

     private final Pose pose8End = new Pose(88.2, 89.9, Math.toRadians(42));

     private final Pose pose9End = new Pose(96.6, 36.5, Math.toRadians(0));

     private final Pose pose10End = new Pose(130, 36.5, Math.toRadians(0));

     private final Pose pose11End = new Pose(88.2, 89.9, Math.toRadians(42));

     private final Pose pose12End = new Pose(90.7, 110.2, Math.toRadians(-90));


     private PathChain path1, path2, path3, path4, path5, path6, path7, path8, path9, path10, path11, path12;

     public void buildPaths() {
         path1 = follower.pathBuilder()
                 .addPath(new BezierLine(startPose, pose1End))
                 .setConstantHeadingInterpolation(pose1End.getHeading())
                 .build();

         path2 = follower.pathBuilder()
                 .addPath(new BezierLine(pose1End, pose2End))
                 .setLinearHeadingInterpolation(pose1End.getHeading(), pose2End.getHeading())
                 .build();

         path3 = follower.pathBuilder()
                 .addPath(new BezierLine(pose2End, pose3End))
                 .setLinearHeadingInterpolation(pose2End.getHeading(), pose3End.getHeading())
                 .build();

         path4 = follower.pathBuilder()
                 .addPath(new BezierCurve(pose3End, bezierIntakeToGate, pose4End))
                 .setLinearHeadingInterpolation(pose3End.getHeading(), pose4End.getHeading())
                 .build();

         path5 = follower.pathBuilder()
                 .addPath(new BezierCurve(pose4End, bezierGateToShoot, pose5End))
                 .setConstantHeadingInterpolation( pose5End.getHeading())
                 .build();

         path6 = follower.pathBuilder()
                 .addPath(new BezierLine(pose5End, pose6End))
                 .setLinearHeadingInterpolation(pose5End.getHeading(), pose6End.getHeading())
                 .build();

         path7 = follower.pathBuilder()
                 .addPath(new BezierLine(pose6End, pose7End))
                 .setLinearHeadingInterpolation(pose6End.getHeading(), pose7End.getHeading())
                 .build();

         path8 = follower.pathBuilder()
                 .addPath(new BezierLine(pose7End, pose8End))
                 .setConstantHeadingInterpolation(pose8End.getHeading())
                 .build();

         path9 = follower.pathBuilder()
                 .addPath(new BezierLine(pose8End, pose9End))
                 .setLinearHeadingInterpolation(pose8End.getHeading(), pose9End.getHeading())
                 .build();

         path10 = follower.pathBuilder()
                 .addPath(new BezierLine(pose9End, pose10End))
                 .setLinearHeadingInterpolation(pose9End.getHeading(), pose10End.getHeading())
                 .build();

         path11 = follower.pathBuilder()
                 .addPath(new BezierLine(pose10End, pose11End))
                 .setConstantHeadingInterpolation(pose11End.getHeading())
                 .build();

         path12 = follower.pathBuilder()
                 .addPath(new BezierLine(pose11End, pose12End))
                 .setLinearHeadingInterpolation(pose11End.getHeading(), pose12End.getHeading())
                 .build();
     }

     public void statePathUpdate() throws InterruptedException {
         switch (pathState) {
             case DRIVE_PATH1:
                 follower.followPath(path1, true);
                 setPathState(PathState.SHOOT_PATH_WAIT);
                 break;
             case SHOOT_PATH_WAIT:
                 if (!follower.isBusy() && Math.toDegrees(follower.getHeadingError()) < 3) {
                     setPathState(PathState.SHOOT_PATH1);
                 }
                 break;
             case SHOOT_PATH1:
                 // Wait for path to finish and delay
                 if (pathTimer.getElapsedTimeSeconds() > 0) {
                     shooter1.setVelocity(shootVelocity);
                     shooter2.setVelocity(shootVelocity);
                     setPathState(PathState.SPIN_UP);

                 }
                 break;
             case SPIN_UP:
                 if (pathTimer.getElapsedTimeSeconds() > 1) {
                     setPathState(PathState.SHOOT);
                 }
                 break;
             case SHOOT:
                 if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                     shoot();
                     ballsShot += 1;


                     if (ballsShot >= 3) {
                         ballsShot = 0;
                         follower.followPath(path2, true);
                         setPathState(PathState.DRIVE_PATH2);
                     }
                     else {
                         setPathState(PathState.SHOOT);
                     }
                 }
                 break;

             case DRIVE_PATH2:
                 if (!follower.isBusy()) {
                     follower.followPath(path3, true);
                     setPathState(PathState.DRIVE_PATH3);
                 }
                 break;

             case DRIVE_PATH3:
                 if (!follower.isBusy()) {
                     intake.setPower(1);
                     setPathState(PathState.INTAKE_ON_PATH3);
                 }
                 break;

             case INTAKE_ON_PATH3:
                 follower.followPath(path4, true);
                 setPathState(PathState.DRIVE_PATH4);
                 break;

             case DRIVE_PATH4:
                 if (!follower.isBusy()) {
                     follower.followPath(path5, true);
                     setPathState(PathState.DRIVE_PATH5);
                 }
                 break;

             case DRIVE_PATH5:
                 if (!follower.isBusy()) {
                     setPathState(PathState.SHOOT_PATH5);
                 }
                 break;

             case SHOOT_PATH5:
                 if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5 && !shotsFired) {
                     setPathState(PathState.SPIN_UP2);
                 }
                 break;
             case SPIN_UP2:
                 if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                     shooter1.setVelocity(shootVelocity);
                     shooter2.setVelocity(shootVelocity);
                     setPathState(PathState.SHOOT2);
                 }
                 break;
             case SHOOT2:
                 if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                     shoot();
                     ballsShot += 1;


                     if (ballsShot >= 3) {
                         ballsShot = 0;
                         follower.followPath(path7, true);
                         setPathState(PathState.DRIVE_PATH6);
                     }
                     else {
                         setPathState(PathState.SHOOT2);
                     }
                 }
                 break;

             case DRIVE_PATH6:
                 if (!follower.isBusy()) {
                     follower.followPath(path7, true);
                     setPathState(PathState.DRIVE_PATH7);
                 }
                 break;

             case DRIVE_PATH7:
                 if (!follower.isBusy()) {
                     intake.setPower(1);
                     setPathState(PathState.INTAKE_ON_PATH7);
                 }
                 break;

             case INTAKE_ON_PATH7:
                 follower.followPath(path8, true);
                 setPathState(PathState.DRIVE_PATH8);
                 break;

             case DRIVE_PATH8:
                 if (!follower.isBusy()) {
                     setPathState(PathState.SHOOT_PATH8);
                 }
                 break;

             case SHOOT_PATH8:
                 if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5 && !shotsFired) {
                     setPathState(PathState.SPIN_UP3);
                 }
             case SPIN_UP3:
                 if (pathTimer.getElapsedTimeSeconds() > 0.5) {

                     shooter1.setVelocity(shootVelocity);
                     shooter2.setVelocity(shootVelocity);
                     setPathState(PathState.SHOOT3);
                 }
                 break;
             case SHOOT3:
                 if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                     shoot();
                     ballsShot += 1;


                     if (ballsShot >= 3) {
                         ballsShot = 0;
                         follower.followPath(path9, true);
                         setPathState(PathState.DRIVE_PATH9);
                     }
                     else {
                         setPathState(PathState.SHOOT3);
                     }
                 }
                 break;

             case DRIVE_PATH9:
                 if (!follower.isBusy()) {
                     follower.followPath(path10, true);
                     setPathState(PathState.DRIVE_PATH10);
                 }
                 break;

             case DRIVE_PATH10:
                 if (!follower.isBusy()) {
                     follower.followPath(path11, true);
                     setPathState(PathState.DRIVE_PATH11);
                 }
                 break;

             case DRIVE_PATH11:
                 if (!follower.isBusy()) {
                     setPathState(PathState.SHOOT_PATH11);
                 }
                 break;

             case SHOOT_PATH11:
                 if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5) {

                     shooter1.setVelocity(shootVelocity);
                     shooter2.setVelocity(shootVelocity);
                     setPathState(PathState.SPIN_UP4);
                 }
                 break;
             case SPIN_UP4:
                 if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                     setPathState(PathState.SHOOT4);
                 }
                 break;
             case SHOOT4:
                 if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                     shoot();
                     ballsShot += 1;


                     if (ballsShot >= 3) {
                         ballsShot = 0;
                         follower.followPath(path12, true);
                         setPathState(PathState.DRIVE_PATH12);
                     }
                     else {
                         setPathState(PathState.SHOOT4);
                     }
                 }
                 break;

             case DRIVE_PATH12:
                 if (!follower.isBusy()) {
                     setPathState(PathState.DONE);
                 }
                 break;

             case DONE:
                 telemetry.addLine("Done all Paths");
                 break;

             default:
                 telemetry.addLine("DONE :)");
                 break;
         }
     }

     public void setPathState(PathState newState) {
         pathState = newState;
         pathTimer.resetTimer();
         shotsFired = false;
     }

     @Override
     public void init() {
         panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

          pathState = PathState.DRIVE_PATH1;
         pathTimer = new Timer();
         opModeTimer = new Timer();
         shootTimer = new Timer();

         follower = Constants.createFollower(hardwareMap);

         // Initialize mechanisms
         shooter1 = hardwareMap.get(DcMotorEx.class, "Shooter1");
         shooter2 = hardwareMap.get(DcMotorEx.class, "Shooter2");
         intake = hardwareMap.get(DcMotor.class, "Intake");
         kicker = hardwareMap.get(Servo.class, "Kicker");

         shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

         PIDFCoefficients pidfCoefficients = new PIDFCoefficients(100, 0, 0, 15);
         shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
         shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

         kicker.setPosition(0.31);

         buildPaths();
         follower.setPose(startPose);

         panelsTelemetry.debug("Status", "Initialized");
         panelsTelemetry.update(telemetry);
     }

     public void start() {
         opModeTimer.resetTimer();
         setPathState(pathState);

         // Start shooters at idle velocity
         shooter1.setVelocity(idleVelocity);
         shooter2.setVelocity(idleVelocity);
     }

     @Override
     public void loop() {
         follower.update();

         try {
             statePathUpdate();
         } catch (InterruptedException e) {
             throw new RuntimeException(e);
         }

         // Log values

         panelsTelemetry.debug("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
         panelsTelemetry.debug("Path State", pathState);
         panelsTelemetry.debug("Follower Busy", follower.isBusy());
         panelsTelemetry.debug("Shots Fired", shotsFired);
         panelsTelemetry.debug("X", follower.getPose().getX());
         panelsTelemetry.debug("Y", follower.getPose().getY());
         panelsTelemetry.debug("Shooter1 Velocity", shooter1.getVelocity());
         panelsTelemetry.debug("Shooter2 Velocity", shooter2.getVelocity());
         panelsTelemetry.update(telemetry);
     }

     public void shoot() throws InterruptedException {
         intake.setPower(1);
         kicker.setPosition(0.6);
         sleep(160);
         kicker.setPosition(0.31);
     }
 }