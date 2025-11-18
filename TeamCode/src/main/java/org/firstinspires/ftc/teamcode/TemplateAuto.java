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
    protected DcMotor motorRightBack = null;
    protected DcMotor motorLeftFront = null;
    protected DcMotor motorLeftBack = null;
    protected DcMotor shooter1;
    protected DcMotor shooter2;

    protected Servo intake = null;

    @Override
    public void runOpMode() throws InterruptedException {
        motorRightFront = hardwareMap.get(DcMotor.class, "rf");
        motorRightBack = hardwareMap.get(DcMotor.class, "rr");
        motorLeftBack = hardwareMap.get(DcMotor.class, "lr");
        motorLeftFront = hardwareMap.get(DcMotor.class, "lf");
        intake = hardwareMap.get(Servo.class, "Intake");


        motorLeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE );
        motorLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE );
        motorRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE );
        motorRightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE );

    }

    public void strafeLeft(double speed){
        motorLeftFront.setPower(-speed);
        motorRightFront.setPower(-speed);
        motorRightBack.setPower(speed);
        motorLeftBack.setPower(speed);
    }

    public void strafeRight(double speed){
        motorRightBack.setPower(-speed);
        motorRightFront.setPower(speed);
        motorLeftBack.setPower(-speed);
        motorLeftFront.setPower(speed);
    }

    public void moveFront(double speed){
        motorRightBack.setPower(-speed);
        motorRightFront.setPower(-speed);
        motorLeftFront.setPower(speed);
        motorLeftBack.setPower(speed);
    }

    public void moveBack(double speed){
        motorLeftBack.setPower(-speed);
        motorLeftFront.setPower(-speed);
        motorRightFront.setPower(speed);
        motorRightBack.setPower(speed);

    }

    public void rotateRight(double speed){
        motorRightBack.setPower(speed);
        motorRightFront.setPower(speed);
        motorLeftFront.setPower(speed);
        motorLeftBack.setPower(speed);
    }

    public void rotateLeft(double speed){
        motorRightBack.setPower(-speed);
        motorRightFront.setPower(-speed);
        motorLeftFront.setPower(-speed);
        motorLeftBack.setPower(-speed);
    }

}