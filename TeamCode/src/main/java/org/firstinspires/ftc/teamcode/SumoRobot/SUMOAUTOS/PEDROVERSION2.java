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
public class PEDROVERSION2 extends OpMode {

    /* ===================== HARDWARE ===================== */
    private DcMotorEx shooter1, shooter2;
    private DcMotor intake;
    private Servo kicker;

    /* ===================== CONSTANTS ===================== */
    public static double SHOOT_VELOCITY = 2300;
    public static double INTAKE_FAST = 1.0;
    public static double INTAKE_SLOW = 0.3;

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
            new Pose(53.0, 8.500, 90),
            new Pose(60.270, 15.835, 117),
            new Pose(47.684, 60.257, 180),
            new Pose(14.982, 60.130, 180),
            new Pose(14.982, 70.002, 270)
    };

    private final Action[] actions = {
            Action.NONE,
            Action.FIRE,
            Action.INTAKE_ON,
            Action.INTAKE_ON,
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

            // Advance immediately to prevent retrigger
            waypointIndex++;
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
