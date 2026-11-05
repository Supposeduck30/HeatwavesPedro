package org.firstinspires.ftc.teamcode.biobuzz.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import static org.firstinspires.ftc.teamcode.biobuzz.teleop.BioBuzzConfig.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Teleop used at the BioBuzz scrimmage (carried over from the other repo, logic unchanged).
 * Simple and sensor-free: constant shooter velocity, no turret, no ball counting.
 */
@TeleOp(name = "Scrimmage Teleop", group = "BioBuzz")
public class ScrimmageTeleop extends OpMode {

    private Follower follower;

    private Servo blocker;
    private DcMotorEx shooter;
    private DcMotor intake, transfer;

    // Shooter runs at SHOOTER_FIXED_VELOCITY with the SHOOTER_P / SHOOTER_F from BioBuzzConfig.

    // KICKER
    public static final double KICKER_BLOCK = 0.63;
    public static final double KICKER_OPEN = 0.85;

    private Servo flower;
    private boolean flowerActive = false;
    private boolean lastCross = false;

    @Override
    public void init() {
        // Pedro handles the drivetrain
        follower = Constants.create(hardwareMap);

        blocker = hardwareMap.get(Servo.class, "blocker");
        flower = hardwareMap.get(Servo.class, FLOWER);
        transfer = hardwareMap.get(DcMotor.class, "transfer");
        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(SHOOTER_DIR);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F));

        // Start closed
        blocker.setPosition(KICKER_BLOCK);
        flower.setPosition(FLOWER_REST);

        transfer.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        // Drive (gamepad1): robot-centric, holds position when sticks are released
        ManualDrive.driveOrHold(
                follower,
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x
        );
        follower.update();

        shooter.setVelocity(SHOOTER_FIXED_VELOCITY);

        // Hold triangle (either controller) to open blocker, release to close
        boolean blockerOpen = gamepad2.triangle || gamepad1.triangle;
        blocker.setPosition(blockerOpen ? KICKER_OPEN : KICKER_BLOCK);

        // Gamepad1 cross toggles Flower1: press once = active, press again = rest
        if (gamepad1.cross && !lastCross) {
            flowerActive = !flowerActive;
        }
        lastCross = gamepad1.cross;
        flower.setPosition(flowerActive ? FLOWER_ACTIVE : FLOWER_REST);

        // Right bumper = intake + transfer, left bumper = intake only, cross = reverse both
        if (gamepad2.right_bumper) {
            transfer.setPower(1.0);
            intake.setPower(1.0);
        } else if (gamepad2.left_bumper) {
            transfer.setPower(0);
            intake.setPower(1.0);
        } else if (gamepad2.cross) {
            transfer.setPower(-1.0);
            intake.setPower(-1.0);
        } else {
            transfer.setPower(0);
            intake.setPower(0);
        }

        telemetry.addData("Gamepad2 assigned", gamepad2.getGamepadId() != Gamepad.ID_UNASSOCIATED);
        telemetry.addData("Shooter target / actual (ticks/s)", "%.0f / %.0f", SHOOTER_FIXED_VELOCITY, shooter.getVelocity());
        telemetry.addData("Blocker", blockerOpen ? "OPEN" : "BLOCKED");
        telemetry.addData("Flower1", flowerActive ? "ACTIVE" : "REST");
        telemetry.addData("Robot X", follower.pose().x());
        telemetry.addData("Robot Y", follower.pose().y());
        telemetry.addData("Robot Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.update();
    }

    @Override
    public void stop() {
        shooter.setVelocity(0);
    }
}
