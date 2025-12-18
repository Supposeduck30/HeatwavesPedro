package org.firstinspires.ftc.teamcode.OldRobot;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
@Autonomous(name = "AutoPath2", group = "Autonomous")
@Configurable
public class AutoPath2 extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState = 0;
    private Paths paths;
    private DcMotor shooter1 = null;
    private DcMotor shooter2 = null;
    private Servo intake = null;

    // Timer for state machine
    private long stateStartTime = 0;

    // --------------------------
    // Configurable timings
    // --------------------------
    public static double shooterPower = 0.6;

    public static int shooterSpinUpTimeMs = 1000; // time to spin up shooter

    public static int servoMoveTimeMs = 2000; // time for servo to push balls

    public static double servoShootPosition = 1.0; // servo position to shoot

    public static double servoRetractPosition = 0.2
            ; // servo position to retract

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        shooter1 = hardwareMap.get(DcMotor.class, "SM1");
        shooter2 = hardwareMap.get(DcMotor.class, "SM2");
        intake = hardwareMap.get(Servo.class, "Intake");

        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void start() {
        pathState = 0;
    }

    @Override
    public void loop() {
        follower.update();
        pathState = autonomousPathUpdate();

        // Telemetry
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    // -------------------------
    //   PATH CLASS
    // -------------------------
    public static class Paths {
        public PathChain line1;

        public Paths(Follower follower) {
            line1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.000, 8.000), new Pose(71.881, 83.743))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(130))
                    .build();
        }
    }

    // -----------------------------------------
    //          STATE MACHINE LOGIC
    // -----------------------------------------
    public int autonomousPathUpdate() {

        switch (pathState) {

            case 0:
                // Start path
                follower.followPath(paths.line1);
                return 1;

            case 1:
                // Wait for path to finish
                if (!follower.isBusy()) {
                    return 2;
                }
                return 1;

            case 2:
                // Start shooter
                shooter1.setPower(-shooterPower);
                shooter2.setPower(shooterPower);
                stateStartTime = System.currentTimeMillis();
                return 3;

            case 3:
                // Wait for shooter to spin up
                if (System.currentTimeMillis() - stateStartTime >= shooterSpinUpTimeMs) {
                    return 4;
                }
                return 3;

            case 4:
                // Move servo to shoot
                intake.setPosition(servoShootPosition);
                stateStartTime = System.currentTimeMillis();
                return 5;

            case 5:
                // Wait for servo to finish
                if (System.currentTimeMillis() - stateStartTime >= servoMoveTimeMs) {
                    return 6;
                }
                return 5;

            case 6:
                // Stop shooter and retract servo
                shooter1.setPower(0);
                shooter2.setPower(0);
                intake.setPosition(servoRetractPosition);
                return 7;

            case 7:
                // Done — robot waits
                return 7;
        }

        return pathState;
    }
}
