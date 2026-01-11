package org.firstinspires.ftc.teamcode.SumoRobot;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled

@TeleOp
public class SUMOTELEOP2 extends OpMode {

    private DcMotor motorRightFront;
    private DcMotor motorLeftFront;
    private DcMotor motorRightBack;
    private DcMotor motorLeftBack;
    private DcMotorEx shooter1;
    private DcMotorEx shooter2;
    private DcMotor intake;
    private Servo kicker;

    public static double LOW_VELOCITY  = 1850;
    public static double HIGH_VELOCITY = 2250;

    public static final double KICKER_OUT = 0.6;
    public static final double KICKER_IN  = 0.31;

    boolean kickerActive = false;

    private Follower follower;

    @Override
    public void init() {

        // Drive motors
        motorRightFront = hardwareMap.get(DcMotor.class, "FR");
        motorLeftFront  = hardwareMap.get(DcMotor.class, "FL");

        motorRightBack  = hardwareMap.get(DcMotor.class, "BR");
        motorLeftBack   = hardwareMap.get(DcMotor.class, "BL");

        // Other motors
        shooter1 = hardwareMap.get(DcMotorEx.class, "Shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class,"Shooter2");

        intake  = hardwareMap.get(DcMotor.class, "Intake");
        kicker  = hardwareMap.get(Servo.class, "Kicker");

        // Motor directions
        motorRightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLeftBack.setDirection(DcMotorSimple.Direction.FORWARD);

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(25,0,0,15);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);

        // Brake mode
        motorRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    @Override
    public void loop() {
        // Drive controls
       moveDriveTrain();
        // Shooter and Intake

        if (gamepad2.triangle) {
            shooter1.setVelocity(LOW_VELOCITY);
            shooter2.setVelocity(LOW_VELOCITY);
        } else {
           shooter1.setVelocity(0.0);
           shooter2.setVelocity(0.0);
        }

        // === KICKER CONTROL ===
        if (gamepad2.dpad_up) {
            kicker.setPosition(KICKER_OUT);
            kickerActive = true;
        } else {
            kicker.setPosition(KICKER_IN);
            kickerActive = false;
        }

        boolean lowGoal  = gamepad2.left_bumper;
        boolean highGoal = gamepad2.right_bumper;

        if (highGoal) {
            shooter1.setVelocity(HIGH_VELOCITY);
            shooter2.setVelocity(HIGH_VELOCITY);

            if (!kickerActive) {
                intake.setPower(1.0);
            } else {
                intake.setPower(0.0);
            }

        } else if (lowGoal) {
            shooter1.setVelocity(LOW_VELOCITY);
            shooter2.setVelocity(LOW_VELOCITY);

            if (!kickerActive) {
                intake.setPower(1.0);
            } else {
                intake.setPower(0.0);
            }

        } else {
            shooter1.setVelocity(0);
            shooter2.setVelocity(0);
            intake.setPower(0);
        }

    }

    public void moveDriveTrain() {
        double vertical;
        double horizontal;
        double pivot;
        vertical =  gamepad1.left_stick_y;
        horizontal = gamepad1.left_stick_x;
        pivot = gamepad1.right_stick_x;

        motorRightBack.setDirection(DcMotor.Direction.REVERSE);
        motorLeftFront.setDirection(DcMotorSimple.Direction.FORWARD);


        motorRightFront.setPower(-(0.75 * -pivot) + (vertical + horizontal));
        motorRightBack.setPower(-(0.75 * pivot) + (-vertical + horizontal));
        motorLeftBack.setPower(-(0.75 * -pivot) + (-vertical -  horizontal));
        motorLeftFront.setPower(-(0.75 * pivot) + (vertical - horizontal));
    }
}
