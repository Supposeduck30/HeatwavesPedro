package org.firstinspires.ftc.teamcode.InfernoRobot.sampledata;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Blue Side Autonomous", group = "Autonomous")
@Configurable
public class BlueNineBallAuto extends OpMode {

    /* ===================== PEDRO ===================== */
    private TelemetryManager panelsTelemetry;
    private Follower follower;
    private Paths paths;

    /* ===================== HARDWARE ===================== */
    private DcMotor shooter;
    private DcMotor intake;
    private Servo kicker;

    /* ===================== STATE ===================== */
    private enum PathState {
        END_POSE,
        ROW1,
        DRIVE_SHOOT_ROW1,
        SHOOT_ROW1,
        ROW2,
        COLLECT_ROW2,
        DRIVE_SHOOT_ROW2,
        SHOOT_ROW2,
        LEAVE,
        DONE
    }

    private PathState pathState;
    private boolean hasShotRow1 = false;
    private boolean hasShotRow2 = false;
    private double pathStartTime; // timer using getRuntime()

    /* ===================== INIT ===================== */
    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(22.774299835255352, 124.54695222405273, Math.toRadians(90)));

        paths = new Paths(follower);

        shooter = hardwareMap.get(DcMotor.class, "Shooter");
        intake = hardwareMap.get(DcMotor.class, "Intake");
        kicker = hardwareMap.get(Servo.class, "Kicker");

        setPathState(PathState.END_POSE);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        pathStartTime = getRuntime();
    }

    /* ===================== LOOP ===================== */
    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        panelsTelemetry.debug("State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    /* ===================== STATE MACHINE ===================== */
    private void autonomousPathUpdate() {
        switch (pathState) {

            case END_POSE:
                follower.followPath(paths.EndPose, true);
                setPathState(PathState.ROW1);
                break;

            case ROW1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Row1, true);
                    setPathState(PathState.DRIVE_SHOOT_ROW1);
                }
                break;

            case DRIVE_SHOOT_ROW1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.SHOOTROW1, true);
                    setPathState(PathState.SHOOT_ROW1);
                }
                break;

            case SHOOT_ROW1:
                if (!follower.isBusy() && !hasShotRow1 && (getRuntime() - pathStartTime > 0.3)) {
                    shootThree();
                    hasShotRow1 = true;
                    follower.followPath(paths.ROW2, true);
                    setPathState(PathState.ROW2);
                }
                break;

            case ROW2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.ROW2, true);
                    setPathState(PathState.COLLECT_ROW2);
                }
                break;

            case COLLECT_ROW2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.COLLECTROW2, true);
                    setPathState(PathState.DRIVE_SHOOT_ROW2);
                }
                break;

            case DRIVE_SHOOT_ROW2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.SHOOTROW2, true);
                    setPathState(PathState.SHOOT_ROW2);
                }
                break;

            case SHOOT_ROW2:
                if (!follower.isBusy() && !hasShotRow2 && (getRuntime() - pathStartTime > 0.3)) {
                    shootThree();
                    hasShotRow2 = true;
                    setPathState(PathState.LEAVE);
                }
                break;

            case LEAVE:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Leave, true);
                    setPathState(PathState.DONE);
                }
                break;

            case DONE:
                telemetry.addLine("Autonomous Complete");
                break;
        }
    }

    private void setPathState(PathState newState) {
        pathState = newState;
        pathStartTime = getRuntime(); // reset timer for each state
    }

    /* ===================== SHOOT LOGIC ===================== */
    private void shootThree() {
        shooter.setPower(0.8);
        intake.setPower(0.8);
        sleep(1200); // spin up shooter

        kick();
        kick();
        kick();

        shooter.setPower(0);
        intake.setPower(0);
    }

    private void kick() {
        kicker.setPosition(0.6);
        sleep(300);
        kicker.setPosition(0.25);
        sleep(300);
    }

    private void sleep(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException ignored) {}
    }

    /* ===================== PATHS ===================== */
    public static class Paths {

        public PathChain EndPose, Row1, SHOOTROW1, ROW2, COLLECTROW2, SHOOTROW2, Leave;

        public Paths(Follower follower) {

            // ----------------- EndPose -----------------
            // Start: near Blue Alliance starting tile (approx)
            // End: move toward first row to prepare shooting
            EndPose = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(22.774, 123.598), // Starting position (near alliance wall, high Y)
                            new Pose(56.461, 86.827))) // Move to middle of field for first ball collection
                    .setLinearHeadingInterpolation(
                            Math.toRadians(145),
                            Math.toRadians(135))
                    .build();

            // ----------------- Row1 -----------------
            // Drive along first row of balls
            Row1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(56.461, 86.827), // Start at end of EndPose
                            new Pose(41.753, 83.743))) // Position aligned with first row of balls
                    .setLinearHeadingInterpolation(
                            Math.toRadians(135),
                            Math.toRadians(180))
                    .build();

            // ----------------- ShootRow1 -----------------
            // Move to shooting position for first 3 balls
            SHOOTROW1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(41.753, 83.743), // Collect position of row1
                            new Pose(56.461, 86.827))) // Shooting position
                    .setLinearHeadingInterpolation(
                            Math.toRadians(180),
                            Math.toRadians(135))
                    .build();

            // ----------------- Row2 -----------------
            // Drive toward second row of balls
            ROW2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(56.461, 86.827), // End of ShootRow1
                            new Pose(56.461, 59.545))) // Move down field Y toward row2
                    .setLinearHeadingInterpolation(
                            Math.toRadians(135),
                            Math.toRadians(180))
                    .build();

            // ----------------- CollectRow2 -----------------
            // Drive along second row to collect balls
            COLLECTROW2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(56.461, 59.545), // Start at row2 entry
                            new Pose(41.278, 59.545))) // Collect along row2 horizontally
                    .setLinearHeadingInterpolation(
                            Math.toRadians(180),
                            Math.toRadians(180))
                    .build();

            // ----------------- ShootRow2 -----------------
            // Move to second shooting position
            SHOOTROW2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(41.278, 59.545), // End of CollectRow2
                            new Pose(56.461, 87.064))) // Shooting position for second 3 balls
                    .setLinearHeadingInterpolation(
                            Math.toRadians(180),
                            Math.toRadians(135))
                    .build();

            // ----------------- Leave -----------------
            // Final move to leave field or park
            Leave = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(56.461, 87.064), // Shooting end
                            new Pose(72, 120))) // Park near alliance wall or designated area
                    .setLinearHeadingInterpolation(
                            Math.toRadians(135),
                            Math.toRadians(270))
                    .build();
        }
    }

}