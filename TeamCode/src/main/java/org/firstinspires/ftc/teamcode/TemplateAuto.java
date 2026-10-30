package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
@Disabled
public class TemplateAuto extends LinearOpMode {
    //Motors
    protected DcMotor motorRightFront = null;
    protected DcMotor BR = null;
    protected DcMotor motorLeftFront = null;
    protected DcMotor motorLeftBack = null;
    protected DcMotor shooter1;
    protected DcMotor shooter2;


    protected Servo intake = null;

    @Override
    public void runOpMode() throws InterruptedException {
        motorRightFront = hardwareMap.get(DcMotor.class, "rf");
        BR = hardwareMap.get(DcMotor.class, "BR");
        motorLeftBack = hardwareMap.get(DcMotor.class, "lr");
        motorLeftFront = hardwareMap.get(DcMotor.class, "lf");
        intake = hardwareMap.get(Servo.class, "Intake");

        shooter1 = hardwareMap.get(DcMotor.class,"SM1");
        shooter2 = hardwareMap.get(DcMotor.class, "SM2");
        motorLeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE );
        motorLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE );
        motorRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE );
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE );

    }

    public void strafeLeft(double speed){
        motorLeftFront.setPower(-speed);
        motorRightFront.setPower(-speed);
        BR.setPower(speed);
        motorLeftBack.setPower(speed);
    }

    public void strafeRight(double speed){
        BR.setPower(-speed);
        motorRightFront.setPower(speed);
        motorLeftBack.setPower(-speed);
        motorLeftFront.setPower(speed);
    }

    public void moveFront(double speed){
        BR.setPower(-speed);
        motorRightFront.setPower(-speed);
        motorLeftFront.setPower(speed);
        motorLeftBack.setPower(speed);
    }

    public void moveBack(double speed){
        motorLeftBack.setPower(-speed);
        motorLeftFront.setPower(-speed);
        motorRightFront.setPower(speed);
        BR.setPower(speed);

    }

    public void rotateRight(double speed){
        BR.setPower(speed);
        motorRightFront.setPower(speed);
        motorLeftFront.setPower(speed);
        motorLeftBack.setPower(speed);
    }

    public void rotateLeft(double speed){
        BR.setPower(-speed);
        motorRightFront.setPower(-speed);
        motorLeftFront.setPower(-speed);
        motorLeftBack.setPower(-speed);
    }

    public void shootOneBall(double speed){
        shooter1.setPower(-speed);
        shooter2.setPower(speed);
    }

    public void transfer (double position ) {
        intake.setPosition(position);
    }

}