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
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
@Configurable
public class HeatwavesQuantaumClean extends OpMode {

    /* ===================== HARDWARE ===================== */
    private DcMotorEx shooter1, shooter2;
    private DcMotor intake;
    private Servo kicker;

    /* ===================== CONSTANTS ===================== */
    public static double SHOOT_VELOCITY = 2250;

    public static double KICKER_IN  = 0.31;
    public static double KICKER_OUT = 0.60;

    public static double INTAKE_FAST = 1.0;
    public static double INTAKE_SLOW = 0.3;

    /* ===================== PEDRO ===================== */
    private Follower follower;
    private PathChain autoPath;

    /* ===================== TIMERS ===================== */
    private final Timer shooterTimer = new Timer();

    /* ===================== SHOOTER STATE ===================== */
    private enum ShooterState {
        IDLE,
        SPINUP,
        FIRE,
        RESET
    }

    private ShooterState shooterState = ShooterState.IDLE;

    /* ===================== POSES ===================== */
    private final Pose startPose   = new Pose(88, 8, Math.toRadians(90));
    private final Pose shootPose1  = new Pose(84.08, 13.68, Math.toRadians(57));
    private final Pose collectRow1 = new Pose(99.88, 34.88, Math.toRadians(0));
    private final Pose takeRow1    = new Pose(136.4, 34.88, Math.toRadians(0));
    private final Pose shootPose2  = new Pose(84.08, 13.68, Math.toRadians(59));
    private final Pose collectRow2 = new Pose(136.28, 35.08, Math.toRadians(270));
    private final Pose takeRow2    = new Pose(136.04, 12.48, Math.toRadians(270));
    private final Pose shootPose3  = new Pose(84.48, 13.44, Math.toRadians(59));
    private final Pose endPose     = new Pose(119.72, 16.28, Math.toRadians(270));

    /* ===================== INIT ===================== */
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        shooter1 = hardwareMap.get(DcMotorEx.class, "Shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "Shooter2");
        intake   = hardwareMap.get(DcMotor.class, "Intake");
        kicker   = hardwareMap.get(Servo.class, "Kicker");

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidf = new PIDFCoefficients(50, 0, 0, 15);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        kicker.setPosition(KICKER_IN);

        buildPath();
        follower.setPose(startPose);
    }

    /* ===================== BUILD PATH ===================== */
    private void buildPath() {
        autoPath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose1))
                .addPath(new BezierLine(shootPose1, collectRow1))
                .addPath(new BezierLine(collectRow1, takeRow1))
                .addPath(new BezierLine(takeRow1, shootPose2))
                .addPath(new BezierLine(shootPose2, collectRow2))
                .addPath(new BezierLine(collectRow2, takeRow2))
                .addPath(new BezierLine(takeRow2, shootPose3))
                .addPath(new BezierLine(shootPose3, endPose))
                .build();
    }

    /* ===================== START ===================== */
    @Override
    public void start() {
        follower.followPath(autoPath, true);
    }

    /* ===================== LOOP ===================== */
    @Override
    public void loop() {
        follower.update();

        Pose pose = follower.getPose();

        // Intake runs ALL THE TIME by default
        intake.setPower(INTAKE_FAST);

        updateShooter();

        tryShootAt(pose, shootPose1);
        tryShootAt(pose, shootPose2);
        tryShootAt(pose, shootPose3);

        telemetry.addData("Shooter State", shooterState);
        telemetry.update();
    }

    /* ===================== SHOOT TRIGGERS ===================== */
    private void tryShootAt(Pose current, Pose target) {
        if (distanceTo(current, target) < 1.5 &&
                shooterState == ShooterState.IDLE) {
            shooterState = ShooterState.SPINUP;
            shooterTimer.resetTimer();
        }
    }

    private double distanceTo(Pose a, Pose b) {
        return Math.hypot(
                a.getX() - b.getX(),
                a.getY() - b.getY()
        );
    }

    /* ===================== SHOOTER LOGIC ===================== */
    private void updateShooter() {
        switch (shooterState) {

            case SPINUP:
                shooter1.setVelocity(SHOOT_VELOCITY);
                shooter2.setVelocity(SHOOT_VELOCITY);

                if (shooterTimer.getElapsedTimeSeconds() > 0.6) {
                    intake.setPower(INTAKE_SLOW);   // 👈 slow intake
                    kicker.setPosition(KICKER_OUT);
                    shooterState = ShooterState.FIRE;
                    shooterTimer.resetTimer();
                }
                break;

            case FIRE:
                if (shooterTimer.getElapsedTimeSeconds() > 0.35) {
                    kicker.setPosition(KICKER_IN);
                    intake.setPower(INTAKE_FAST);  // 👈 resume full intake
                    shooterState = ShooterState.RESET;
                    shooterTimer.resetTimer();
                }
                break;

            case RESET:
                if (shooterTimer.getElapsedTimeSeconds() > 0.25) {
                    shooter1.setVelocity(0);
                    shooter2.setVelocity(0);
                    shooterState = ShooterState.IDLE;
                }
                break;

            case IDLE:
                break;
        }
    }
}
