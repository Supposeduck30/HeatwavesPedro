package org.firstinspires.ftc.teamcode.SumoRobot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Sumo Teleop")
public class SUMOTELEOP extends OpMode {

    private DcMotor motorRightFront;
    private DcMotor motorLeftFront;
    private DcMotor motorRightBack;
    private DcMotor motorLeftBack;
    private DcMotorEx shooter1;
    private DcMotor intake;
    private Servo kicker;

    public double highVelocity = 2300;
    public double lowVelocity = 1500;

    double curTargetVelocity = highVelocity;

    @Override
    public void init() {

        // Drive motors
        motorRightFront = hardwareMap.get(DcMotor.class, "FR");
        motorLeftFront  = hardwareMap.get(DcMotor.class, "FL");

        motorRightBack  = hardwareMap.get(DcMotor.class, "BR");
        motorLeftBack   = hardwareMap.get(DcMotor.class, "BL");

        // Other motors
        shooter1 = hardwareMap.get(DcMotorEx.class, "Shooter1");

        intake  = hardwareMap.get(DcMotor.class, "Intake");
        kicker  = hardwareMap.get(Servo.class, "Kicker");

        // Motor directions
        motorRightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLeftBack.setDirection(DcMotorSimple.Direction.FORWARD);

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(100,0,0,15.5);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);

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

        // Kicker
        if (gamepad2.right_bumper) {
            kicker.setPosition(0.6);
        } else {
            kicker.setPosition(0.31);
        }

        // Intake
        if (gamepad2.left_bumper) {
            intake.setPower(0.9);
            shooter1.setVelocity(curTargetVelocity);
        } else {
            intake.setPower(0.0);
            shooter1.setVelocity(0.0);
        }



        // Shooter

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
