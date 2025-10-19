package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class tests extends OpMode {

    private DcMotor shooter1 = null ;
    private DcMotor shooter2 = null ;

    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;

    private DcMotor leftBack = null;

    @Override
    public void init() {
        shooter1 = hardwareMap.get(DcMotor.class,"shooter1");
        shooter2 = hardwareMap.get(DcMotor.class, "shooter2");
        leftFront = hardwareMap.get(DcMotor.class,"lf");
        rightFront= hardwareMap.get(DcMotor.class, "rf");
        rightBack = hardwareMap.get(DcMotor.class, "rr");
        leftBack = hardwareMap.get(DcMotor.class, "rr");
    }

    @Override
    public void loop(){
        if(gamepad1.x) {
            shooter1.setPower(1);
            shooter2.setPower(-1);
        } else {
            shooter1.setPower(0);
            shooter2.setPower(0);
        }

        if (gamepad1.dpad_up) {
            leftFront.setPower(0.5);
            rightFront.setPower(0.5);
            rightBack.setPower(0.5);
            leftBack.setPower(0.5);
        }
        else {
            leftFront.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);
            leftBack.setPower(0);
        }
    }
}
