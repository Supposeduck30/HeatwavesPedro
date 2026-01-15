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
public class PEDROVERSION2REDFAR extends OpMode {

    /* ===================== HARDWARE ===================== */
    private DcMotorEx shooter1, shooter2;
    private DcMotor intake;
    private Servo kicker;

    /* ===================== CONSTANTS ===================== */
    public static double SHOOT_VELOCITY = 2300;
    public static double INTAKE_FAST = 1.0;
    public static double INTAKE_SLOW = 0.6;

    // Kicker positions
    public static final double KICKER_OUT = 0.52;
    public static final double KICKER_IN  = 0.35;

    private static final double WAYPOINT_TOLERANCE = 1.5;
    private static final int SHOTS_PER_FIRE = 3;

    /* ===================== PEDRO ===================== */
    private Follower follower;
    private PathChain autoPath;

    /* ===================== TIMERS ===================== */
    private final Timer shooterTimer = new Timer();

    /* ===================== SHOOTER FSM ===================== */
    private enum ShooterState {
        IDLE,
        KICK_OUT,
        KICK_IN
    }

    private ShooterState shooterState = ShooterState.IDLE;
    private int shotsFired = 0;

    /* ===================== WAYPOINT ACTIONS ===================== */
    private enum Action {
        NONE,
        INTAKE_ON,
        FIRE
    }

    private final Pose[] waypoints = {
            new Pose(91.000, 8.500, 90),
            new Pose(83.730, 15.835, 63),
            new Pose(96.316, 60.257, 0),
            new Pose(129.018, 60.257, 0),
            new Pose(129.018, 70.002, 270),
            new Pose(83.730, 15.835, 63),
            new Pose(96.316, 34.754, 0),
            new Pose(131.569, 34.754, 0),
            new Pose(83.730, 15.835, 63),
            new Pose(137.100, 31.000, 270),
            new Pose(137.100, 10.000, 270),
            new Pose(83.730, 15.835, 63),
            new Pose(106.000, 33.000, 90),
    };


    private final Action[] actions = {
            Action.NONE,
            Action.FIRE,
            Action.INTAKE_ON,
            Action.INTAKE_ON,
            Action.NONE,
            Action.NONE,
            Action.FIRE,
            Action.INTAKE_ON,
            Action.INTAKE_ON,
            Action.FIRE,
            Action.INTAKE_ON,
            Action.INTAKE_ON,
            Action.FIRE,
            Action.NONE
    };

    private int waypointIndex = 0;
    private boolean actionExecuted = false;

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

        PIDFCoefficients pidf = new PIDFCoefficients(100, 0, 0, 20);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        kicker.setPosition(KICKER_IN);

        buildPath();
        follower.setPose(waypoints[0]);
    }

    /* ===================== PATH ===================== */
    private void buildPath() {
        autoPath = follower.pathBuilder()
                .addPath(new BezierLine(waypoints[0], waypoints[1]))
                .addPath(new BezierLine(waypoints[1], waypoints[2]))
                .addPath(new BezierLine(waypoints[2], waypoints[3]))
                .addPath(new BezierLine(waypoints[3], waypoints[4]))
                .addPath(new BezierLine(waypoints[4], waypoints[5]))
                .addPath(new BezierLine(waypoints[5], waypoints[6]))
                .addPath(new BezierLine(waypoints[6], waypoints[7]))
                .addPath(new BezierLine(waypoints[7], waypoints[8]))
                .addPath(new BezierLine(waypoints[8], waypoints[9]))
                .addPath(new BezierLine(waypoints[9], waypoints[10]))
                .addPath(new BezierLine(waypoints[10], waypoints[11]))
                .addPath(new BezierLine(waypoints[11], waypoints[12]))
                .build();
    }

    /* ===================== START ===================== */
    @Override
    public void start() {
        follower.followPath(autoPath, true);

        // Shooter ALWAYS running
        shooter1.setVelocity(SHOOT_VELOCITY);
        shooter2.setVelocity(SHOOT_VELOCITY);

        intake.setPower(INTAKE_FAST);
    }

    /* ===================== LOOP ===================== */
    @Override
    public void loop() {

        follower.update();
        Pose currentPose = follower.getPose();

        updateWaypointActions(currentPose);
        updateShooterFSM();

        telemetry.addData("Waypoint Index", waypointIndex);
        telemetry.addData("Shooter State", shooterState);
        telemetry.addData("Shots Fired", shotsFired);
        telemetry.update();
    }

    /* ===================== WAYPOINT LOGIC ===================== */
    private void updateWaypointActions(Pose current) {

        if (waypointIndex >= waypoints.length) return;

        Pose target = waypoints[waypointIndex];

        if (distanceTo(current, target) < WAYPOINT_TOLERANCE && !actionExecuted) {
            executeAction(actions[waypointIndex]);
            actionExecuted = true;
            waypointIndex++;
            actionExecuted = false; // ← ADD THIS
        }

    }

    private void executeAction(Action action) {

        switch (action) {

            case FIRE:
                if (shooterState == ShooterState.IDLE) {
                    shotsFired = 0;
                    shooterState = ShooterState.KICK_OUT;
                    shooterTimer.resetTimer();
                }
                break;

            case INTAKE_ON:
                intake.setPower(INTAKE_FAST);
                break;

            case NONE:
            default:
                break;
        }
    }

    /* ===================== SHOOTER FSM (3 SHOTS) ===================== */
    private void updateShooterFSM() {

        switch (shooterState) {

            case KICK_OUT:
                intake.setPower(INTAKE_SLOW);
                kicker.setPosition(KICKER_OUT);

                if (shooterTimer.getElapsedTimeSeconds() > 0.25) {
                    shooterState = ShooterState.KICK_IN;
                    shooterTimer.resetTimer();
                }
                break;

            case KICK_IN:
                kicker.setPosition(KICKER_IN);

                if (shooterTimer.getElapsedTimeSeconds() > 0.25) {
                    shotsFired++;

                    if (shotsFired >= SHOTS_PER_FIRE) {
                        intake.setPower(INTAKE_FAST);
                        shooterState = ShooterState.IDLE;
                    } else {
                        shooterState = ShooterState.KICK_OUT;
                    }

                    shooterTimer.resetTimer();
                }
                break;

            case IDLE:
                break;
        }
    }

    /* ===================== UTIL ===================== */
    private double distanceTo(Pose a, Pose b) {
        return Math.hypot(
                a.getX() - b.getX(),
                a.getY() - b.getY()
        );
    }
}
