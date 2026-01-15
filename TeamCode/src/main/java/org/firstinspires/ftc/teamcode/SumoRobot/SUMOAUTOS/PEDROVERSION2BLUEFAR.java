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
public class PEDROVERSION2BLUEFAR extends OpMode {

    /* ===================== HARDWARE ===================== */
    private DcMotorEx shooter1, shooter2;
    private DcMotor intake;
    private Servo kicker;

    /* ===================== CONSTANTS ===================== */
    public static double HIGH_VELOCITY = 2300;
    public static double INTAKE_POWER = 1.0;

    // Kicker positions - MATCHING TELEOP
    public static final double KICKER_OUT = 0.52;
    public static final double KICKER_IN  = 0.35;

    // Kicker timing - MATCHING TELEOP
    private static final long KICK_TIME = 130; // milliseconds

    private static final double WAYPOINT_TOLERANCE = 1.5;
    private static final int SHOTS_PER_FIRE = 3;

    /* ===================== PEDRO ===================== */
    private Follower follower;
    private PathChain autoPath;

    /* ===================== TIMERS ===================== */
    private long kickStartTime = 0;

    /* ===================== SHOOTER FSM ===================== */
    private enum ShooterState {
        IDLE,
        KICKING
    }

    private ShooterState shooterState = ShooterState.IDLE;
    private int shotsFired = 0;
    private int targetShots = 0;

    /* ===================== WAYPOINT ACTIONS ===================== */
    private enum Action {
        NONE,
        INTAKE_ON,
        FIRE
    }

    private final Pose[] waypoints = {
            new Pose(53.0, 8.500, Math.toRadians(90)),
            new Pose(60.270, 15.835, Math.toRadians(117)),
            new Pose(47.684, 60.257, Math.toRadians(180)),
            new Pose(14.982, 60.257, Math.toRadians(180)),
            new Pose(14.982, 70.002, Math.toRadians(270)),
            new Pose(60.270, 15.835, Math.toRadians(117)),
            new Pose(47.684, 34.754, Math.toRadians(180)),
            new Pose(12.431, 34.754, Math.toRadians(180)),
            new Pose(60.270, 15.835, Math.toRadians(117)),
            new Pose(6.9, 31, Math.toRadians(270)),
            new Pose(6.9, 10, Math.toRadians(270)),
            new Pose(60.270, 15.835, Math.toRadians(117)),
            new Pose(38, 33, Math.toRadians(90)),
    };

    private final Action[] actions = {
            Action.NONE,
            Action.FIRE,
            Action.INTAKE_ON,
            Action.INTAKE_ON,
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
        follower.setMaxPower(1);

        shooter1 = hardwareMap.get(DcMotorEx.class, "Shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "Shooter2");
        intake   = hardwareMap.get(DcMotor.class, "Intake");
        kicker   = hardwareMap.get(Servo.class, "Kicker");

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // MATCHING TELEOP PIDF
        PIDFCoefficients pidf = new PIDFCoefficients(120, 0, 0, 15);
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
                .setLinearHeadingInterpolation(waypoints[0].getHeading(), waypoints[1].getHeading())
                .addPath(new BezierLine(waypoints[1], waypoints[2]))
                .setLinearHeadingInterpolation(waypoints[1].getHeading(), waypoints[2].getHeading())
                .addPath(new BezierLine(waypoints[2], waypoints[3]))
                .setLinearHeadingInterpolation(waypoints[2].getHeading(), waypoints[3].getHeading())
                .addPath(new BezierLine(waypoints[3], waypoints[4]))
                .setLinearHeadingInterpolation(waypoints[3].getHeading(), waypoints[4].getHeading())
                .addPath(new BezierLine(waypoints[4], waypoints[5]))
                .setLinearHeadingInterpolation(waypoints[4].getHeading(), waypoints[5].getHeading())
                .addPath(new BezierLine(waypoints[5], waypoints[6]))
                .setLinearHeadingInterpolation(waypoints[5].getHeading(), waypoints[6].getHeading())
                .addPath(new BezierLine(waypoints[6], waypoints[7]))
                .setLinearHeadingInterpolation(waypoints[6].getHeading(), waypoints[7].getHeading())
                .addPath(new BezierLine(waypoints[7], waypoints[8]))
                .setLinearHeadingInterpolation(waypoints[7].getHeading(), waypoints[8].getHeading())
                .addPath(new BezierLine(waypoints[8], waypoints[9]))
                .setLinearHeadingInterpolation(waypoints[8].getHeading(), waypoints[9].getHeading())
                .addPath(new BezierLine(waypoints[9], waypoints[10]))
                .setLinearHeadingInterpolation(waypoints[9].getHeading(), waypoints[10].getHeading())
                .addPath(new BezierLine(waypoints[10], waypoints[11]))
                .setLinearHeadingInterpolation(waypoints[10].getHeading(), waypoints[11].getHeading())
                .addPath(new BezierLine(waypoints[11], waypoints[12]))
                .setLinearHeadingInterpolation(waypoints[11].getHeading(), waypoints[12].getHeading())
                .build();
    }

    /* ===================== START ===================== */
    @Override
    public void start() {
        follower.followPath(autoPath, true);

        // Shooter ALWAYS running at HIGH_VELOCITY
        shooter1.setVelocity(HIGH_VELOCITY);
        shooter2.setVelocity(HIGH_VELOCITY);

        intake.setPower(INTAKE_POWER);
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
        telemetry.addData("Shots Fired", shotsFired + "/" + targetShots);
        telemetry.addData("Pose", currentPose);
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
            actionExecuted = false;
        }
    }

    private void executeAction(Action action) {
        switch (action) {
            case FIRE:
                if (shooterState == ShooterState.IDLE) {
                    shotsFired = 0;
                    targetShots = SHOTS_PER_FIRE;
                    shooterState = ShooterState.KICKING;
                    kickStartTime = System.currentTimeMillis();
                    kicker.setPosition(KICKER_OUT);
                }
                break;

            case INTAKE_ON:
                intake.setPower(INTAKE_POWER);
                break;

            case NONE:
            default:
                break;
        }
    }

    /* ===================== SHOOTER FSM - MATCHING TELEOP ===================== */
    private void updateShooterFSM() {
        if (shooterState == ShooterState.KICKING) {
            // Check if kick time has elapsed
            if (System.currentTimeMillis() - kickStartTime >= KICK_TIME) {
                kicker.setPosition(KICKER_IN);
                shotsFired++;

                // Check if we've fired all shots
                if (shotsFired >= targetShots) {
                    shooterState = ShooterState.IDLE;
                    intake.setPower(INTAKE_POWER);
                } else {
                    // Prepare for next shot
                    kickStartTime = System.currentTimeMillis();
                    kicker.setPosition(KICKER_OUT);
                }
            }
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