package org.firstinspires.ftc.teamcode.SumoRobot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class RegressionTest extends OpMode {

    private DcMotorEx shooter1, shooter2;
    private DcMotor intake;
    private Servo kicker;
    private Follower follower;
    private TurretController turretController;

    // The manual velocity we will adjust with the D-Pad
    private double manualVelocity = 1200.0;

    // Button state tracking (so holding the button doesn't add 1000 instantly)
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadRight = false;
    private boolean lastDpadLeft = false;

    // Kicker constants
    public static final double KICKER_BLOCK = 0.15;
    public static final double KICKER_OPEN  = 0.25;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(54.5, 8, Math.toRadians(90)));

        shooter1 = hardwareMap.get(DcMotorEx.class, "Shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "Shooter2");
        intake = hardwareMap.get(DcMotor.class, "Intake");
        kicker = hardwareMap.get(Servo.class, "Kicker");

        turretController = new TurretController(hardwareMap, "Turret");
        turretController.resetEncoder();

        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);

        PIDFCoefficients pidf = new PIDFCoefficients(120, 0, 0, 20);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        kicker.setPosition(KICKER_BLOCK);

        telemetry.addLine("Tuning Mode Ready.");
        telemetry.addLine("Use D-PAD UP/DOWN for +/- 50 RPM");
        telemetry.addLine("Use D-PAD RIGHT/LEFT for +/- 10 RPM");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();
        Pose currentPose = follower.getPose();

        // 1. BASIC DRIVING
        double driveForward = -gamepad1.left_stick_y;
        double driveStrafe  = -gamepad1.left_stick_x;
        double driveTurn    = -gamepad1.right_stick_x;
        follower.setTeleOpDrive(driveForward, driveStrafe, driveTurn, true);

        // 2. ADJUST VELOCITY WITH D-PAD (Gamepad 1 or 2)
        boolean dpadUp = gamepad1.dpad_up || gamepad2.dpad_up;
        boolean dpadDown = gamepad1.dpad_down || gamepad2.dpad_down;
        boolean dpadRight = gamepad1.dpad_right || gamepad2.dpad_right;
        boolean dpadLeft = gamepad1.dpad_left || gamepad2.dpad_left;

        if (dpadUp && !lastDpadUp) manualVelocity += 50;
        if (dpadDown && !lastDpadDown) manualVelocity -= 50;
        if (dpadRight && !lastDpadRight) manualVelocity += 10;
        if (dpadLeft && !lastDpadLeft) manualVelocity -= 10;

        // Clamp to prevent breaking the motors
        manualVelocity = Range.clip(manualVelocity, 0, 2600);

        lastDpadUp = dpadUp;
        lastDpadDown = dpadDown;
        lastDpadRight = dpadRight;
        lastDpadLeft = dpadLeft;

        // 3. AUTO-AIM TURRET
        turretController.aimAtGoalWithPrediction(currentPose, new Pose(0,0,0));

        // 4. SHOOT MACRO (Right Bumper)
        if (gamepad2.right_bumper || gamepad1.right_bumper) {
            intake.setPower(1.0);
            kicker.setPosition(KICKER_OPEN);
        } else

            if (gamepad2.left_bumper) {
                intake.setPower(1);
        kicker.setPosition(KICKER_BLOCK);
        }
        else {
            intake.setPower(0.0);
            kicker.setPosition(KICKER_BLOCK);
        }

        shooter1.setVelocity(manualVelocity);
        shooter2.setVelocity(manualVelocity);

        // 5. TELEMETRY (WRITE THESE NUMBERS DOWN!)
        double distance = turretController.getDistanceToGoal(currentPose);

        telemetry.addLine("=== SHOOTER TUNER ===");
        telemetry.addLine("Adjust speed using D-Pad.");
        telemetry.addLine("Hold Right Bumper to fire.");
        telemetry.addLine("-----------------------------");
        telemetry.addData("1. CURRENT DISTANCE (X)", "%.1f", distance);
        telemetry.addData("2. PERFECT VELOCITY (Y)", "%.0f", manualVelocity);
        telemetry.addLine("-----------------------------");
        telemetry.addData("S1 Actual", "%.0f", shooter1.getVelocity());
        telemetry.addData("S2 Actual", "%.0f", shooter2.getVelocity());
        telemetry.update();
    }
}