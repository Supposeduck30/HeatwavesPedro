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

    // Changed to Red Turret Controller
    private TurretControllerRED turretController;

    // The manual velocity we will adjust with the D-Pad
    private double manualVelocity = 1200.0;

    // Button state tracking for Shooter Velocity (Gamepad 1 D-pad)
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadRight = false;
    private boolean lastDpadLeft = false;

    // Button state tracking for Turret Trim (Gamepad 1 X and B)
    private boolean lastX = false;
    private boolean lastB = false;

    // Kicker constants
    public static final double KICKER_BLOCK = 0.15;
    public static final double KICKER_OPEN  = 0.25;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(89.8, 8.7, Math.toRadians(90)));

        shooter1 = hardwareMap.get(DcMotorEx.class, "Shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "Shooter2");
        intake = hardwareMap.get(DcMotor.class, "Intake");
        kicker = hardwareMap.get(Servo.class, "Kicker");

        // Instantiating the RED controller (Starts in HOMING state by default)
        turretController = new TurretControllerRED(hardwareMap, "Turret");

        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);

        PIDFCoefficients pidf = new PIDFCoefficients(117, 0, 0, 22);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        kicker.setPosition(KICKER_BLOCK);

        telemetry.addLine("Tuning Mode Ready (Single Controller Tuning)");
        telemetry.addLine("G1 D-PAD: Adjust Shooter RPM (+/- 50 or 10)");
        telemetry.addLine("G1 X / B: Adjust Live Turret Trim (+/- 0.2°)");
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

        // 2. ADJUST VELOCITY WITH D-PAD (Gamepad 1)
        boolean dpadUp = gamepad1.dpad_up;
        boolean dpadDown = gamepad1.dpad_down;
        boolean dpadRight = gamepad1.dpad_right;
        boolean dpadLeft = gamepad1.dpad_left;

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

        // 3. LIVE TURRET TRIM WITH X AND B (Gamepad 1)
        boolean xNow = gamepad1.x;
        boolean bNow = gamepad1.b;

        if (xNow && !lastX) {
            turretController.ANGLE_OFFSETRED += 0.2;
        } else if (bNow && !lastB) {
            turretController.ANGLE_OFFSETRED -= 0.2;
        }

        lastX = xNow;
        lastB = bNow;

        // 4. AUTO-AIM TURRET
        turretController.aimAtGoalWithPredictionRED(currentPose, new Pose(0,0,0));

        // Optional Manual Override to Reset state if things go wrong
        if (gamepad2.options || gamepad1.options) {
            turretController.resetEncoderRED();
        }

        // 5. SHOOT MACRO (Right Bumper)
        if (gamepad2.right_bumper || gamepad1.right_bumper) {
            intake.setPower(1.0);
            kicker.setPosition(KICKER_OPEN);
        } else if (gamepad2.left_bumper || gamepad1.left_bumper) {
            intake.setPower(1);
            kicker.setPosition(KICKER_BLOCK);
        } else {
            intake.setPower(0.0);
            kicker.setPosition(KICKER_BLOCK);
        }

        shooter1.setVelocity(manualVelocity);
        shooter2.setVelocity(manualVelocity);

        // 6. TELEMETRY
        double distance = turretController.getDistanceToGoalRED(currentPose);

        telemetry.addLine("=== G1 SHOOTER & TURRET TUNER ===");
        telemetry.addLine("D-Pad -> RPM | X & B -> Turret Trim");
        telemetry.addLine("Hold Right Bumper to fire.");
        telemetry.addLine("-----------------------------");
        telemetry.addData("Turret Trim Offset", "%.2f°", turretController.ANGLE_OFFSETRED);
        telemetry.addData("1. CURRENT DISTANCE (X)", "%.1f in", distance);
        telemetry.addData("2. PERFECT VELOCITY (Y)", "%.0f ticks/s", manualVelocity);
        telemetry.addLine("-----------------------------");
        telemetry.addData("S1 Actual", "%.0f", shooter1.getVelocity());
        telemetry.addData("S2 Actual", "%.0f", shooter2.getVelocity());
        telemetry.update();
    }
}