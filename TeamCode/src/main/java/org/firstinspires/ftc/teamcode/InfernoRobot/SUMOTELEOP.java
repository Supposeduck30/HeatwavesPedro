package org.firstinspires.ftc.teamcode.InfernoRobot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Sumo Teleop")
public class SUMOTELEOP extends OpMode {

    private  DcMotor motorRightFront = null;
    private  DcMotor motorLeftFront = null;
    private  DcMotor motorRightBack = null;
    private  DcMotor motorLeftBack = null;
    private  DcMotor shooter = null;
    private  DcMotor intake = null;
    private  Servo kicker = null;

    @Override
    public void init() {
        motorRightFront = hardwareMap.get(DcMotor.class, "FR");
        motorLeftBack = hardwareMap.get(DcMotor.class, "BL");
        motorRightBack = hardwareMap.get(DcMotor.class, "BR");
        motorLeftFront = hardwareMap.get(DcMotor.class, "FL");
        shooter = hardwareMap.get(DcMotor.class,"Shooter");
        intake = hardwareMap.get(DcMotor.class, "Intake");
        kicker = hardwareMap.get(Servo.class, "Kicker");
    }

    @Override
    public void loop() {

        moveDriveTrain();
        if(gamepad2.a){
            kicker.setPosition(0.6);
        } else {

            kicker.setPosition(0.35);
        }

        if(gamepad2.b){

            intake.setPower(0.4);
        }else {
            intake.setPower(0.0);

        }
        if (gamepad2.x){
            shooter.setPower(1.0);
        }else{
            shooter.setPower(0.0);
        }
    }

    public void moveDriveTrain() {
        double vertical;
        double horizontal;
        double pivot;
        vertical =  -gamepad1.left_stick_y;
        horizontal = gamepad1.left_stick_x;
        pivot = -gamepad1.right_stick_x;

        motorLeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRightFront.setDirection(DcMotorSimple.Direction.FORWARD);

        motorLeftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRightBack.setDirection(DcMotorSimple.Direction.FORWARD);


        motorRightFront.setPower((-pivot) + (vertical - horizontal));
        motorRightBack.setPower((-pivot) + (vertical + horizontal));
        motorLeftBack.setPower((-pivot) + (-vertical + horizontal));
        motorLeftFront.setPower((-pivot) + (-vertical - horizontal));
    }

}
