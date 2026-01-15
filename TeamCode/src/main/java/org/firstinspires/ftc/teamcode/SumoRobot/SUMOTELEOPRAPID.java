package org.firstinspires.ftc.teamcode.SumoRobot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class SUMOTELEOPRAPID extends OpMode {

    private DcMotor fr, fl, br, bl;
    private DcMotorEx shooter1, shooter2;
    private DcMotor intake;
    private Servo kicker;

    private boolean kickerUp = false;           // Current kicker position state
    private long lastKickToggle = 0;            // Last toggle timestamp
    private static final long KICK_TIME = 145;  // Toggle interval (ms)

    public static final double LOW_VELOCITY  = 1800;
    public static final double HIGH_VELOCITY = 2300;

    public static final double KICKER_OUT  = 0.52;
    public static final double KICKER_IN   = 0.35;

    @Override
    public void init() {
        fr = hardwareMap.get(DcMotor.class, "FR");
        fl = hardwareMap.get(DcMotor.class, "FL");
        br = hardwareMap.get(DcMotor.class, "BR");
        bl = hardwareMap.get(DcMotor.class, "BL");

        shooter1 = hardwareMap.get(DcMotorEx.class, "Shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "Shooter2");

        intake = hardwareMap.get(DcMotor.class, "Intake");
        kicker = hardwareMap.get(Servo.class, "Kicker");

        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);

        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidf = new PIDFCoefficients(110, 0, 0, 14);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        kicker.setPosition(KICKER_IN);
    }

    @Override
    public void loop() {
        // Continuous rapid kicker while holding triangle
        if (gamepad2.triangle) {
            long now = System.currentTimeMillis();
            if (now - lastKickToggle >= KICK_TIME) {
                kickerUp = !kickerUp;
                kicker.setPosition(kickerUp ? KICKER_OUT : KICKER_IN);
                lastKickToggle = now;
            }
        } else {
            kicker.setPosition(KICKER_IN);
            kickerUp = false;
        }

        // Shooter presets
        boolean lowGoal  = gamepad2.left_bumper;
        boolean highGoal = gamepad2.right_bumper;

        double targetVelocity = 0;

        if (highGoal) {
            targetVelocity = HIGH_VELOCITY;
        } else if (lowGoal) {
            targetVelocity = LOW_VELOCITY;
        }

        // Apply shooter power
        if (targetVelocity > 0) {
            shooter1.setVelocity(targetVelocity);
            shooter2.setVelocity(targetVelocity);
            intake.setPower(1.0);
        } else {
            shooter1.setVelocity(0);
            shooter2.setVelocity(0);

        }

        if (targetVelocity > 0) {
            intake.setPower(1);
        } else {
            intake.setPower(0);
        }

        // Telemetry
        double currentVelocity = (shooter1.getVelocity() + shooter2.getVelocity()) / 2.0;
        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Current Velocity", currentVelocity);
        telemetry.update();

        drive();
    }

    private void drive() {
        double vertical   = gamepad1.left_stick_y;
        double horizontal = -gamepad1.left_stick_x;
        double pivot      = gamepad1.right_stick_x * 0.75;

        fr.setPower(pivot + vertical - horizontal);
        br.setPower(pivot + vertical + horizontal);
        bl.setPower(pivot - vertical + horizontal);
        fl.setPower(pivot + -vertical - horizontal);
    }
}
