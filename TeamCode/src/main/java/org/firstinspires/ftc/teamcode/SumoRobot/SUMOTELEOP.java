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
    private DcMotorEx shooter2;
    private DcMotor intake;
    private Servo kicker;

    public double highVelocity = 4000;
    public double lowVelocity = 2100;


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


        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(-10,0,0,-60);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);

        shooter1.setDirection(DcMotorEx.Direction.REVERSE);
        shooter2.setDirection(DcMotorEx.Direction.REVERSE);

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

        intake.setPower(1.0);
        // Shooter and Intake
        if (gamepad2.right_bumper) {
           // intake.setPower(0.9);
            //shooter1.setVelocity(3000);
            //shooter2.setVelocity(3000);
        } else {
           // intake.setPower(0.0);
           // shooter1.setVelocity(0.0);
            // shooter2.setVelocity(0.0);
        }

        if (gamepad2.left_bumper) {
            intake.setPower(0.9);
            shooter1.setVelocity(lowVelocity);
            shooter2.setVelocity(lowVelocity);
        } else {
            intake.setPower(0.0);
            shooter1.setVelocity(0.0);
            shooter2.setVelocity(0.0);
        }

        // Intake

        if(gamepad2.dpad_up){
            kicker.setPosition(0.6);
        } else {
            kicker.setPosition(0.31);
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
