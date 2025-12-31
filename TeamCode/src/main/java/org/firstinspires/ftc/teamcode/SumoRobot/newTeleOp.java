package org.firstinspires.ftc.teamcode.SumoRobot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class newTeleOp extends OpMode {

    private DcMotor motorRightFront;
    private DcMotor motorLeftFront;
    private DcMotor motorRightBack;
    private DcMotor motorLeftBack;
    private DcMotor shooter;
    private DcMotor intake;
    private Servo kicker;

    @Override
    public void init() {

        // Drive motors
        motorRightFront = hardwareMap.get(DcMotor.class, "FR");
        motorLeftFront  = hardwareMap.get(DcMotor.class, "FL");
        motorRightBack  = hardwareMap.get(DcMotor.class, "BR");
        motorLeftBack   = hardwareMap.get(DcMotor.class, "BL");

        // Other motors
        shooter = hardwareMap.get(DcMotor.class, "Shooter");
        intake  = hardwareMap.get(DcMotor.class, "Intake");
        kicker  = hardwareMap.get(Servo.class, "Kicker");

        // Motor directions
        motorRightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLeftBack.setDirection(DcMotorSimple.Direction.FORWARD


        );

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
            shooter.setPower(0.9);
        } else {
            intake.setPower(0.0);
            shooter.setPower(0.0);
        }



        // Shooter

    }

    public void moveDriveTrain() {
        double vertical;
        double horizontal;
        double pivot;
        vertical =  -gamepad1.left_stick_y;
        horizontal = gamepad1.left_stick_x;
        pivot = -gamepad1.right_stick_x;

        motorRightBack.setDirection(DcMotor.Direction.REVERSE);
        motorLeftFront.setDirection(DcMotorSimple.Direction.FORWARD);


        motorRightFront.setPower(-(0.6 * pivot) + (-vertical + horizontal));
        motorRightBack.setPower(-(0.6 * -pivot) + (vertical + horizontal));
        motorLeftBack.setPower(-(0.6 * pivot) + (vertical -  horizontal));
        motorLeftFront.setPower(-(0.6 * -pivot) + (-vertical - horizontal));
    }

    public void shoot() {

    }
}
