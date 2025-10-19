package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class tests extends OpMode {

    private DcMotor shooter1 = null ;
    private DcMotor shooter2 = null ;

    private CRServo intake = null;

    @Override
    public void init() {
        shooter1 = hardwareMap.get(DcMotor.class,"shooter1");
        shooter2 = hardwareMap.get(DcMotor.class, "shooter2");
        intake = hardwareMap.get(CRServo.class, "intake");
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

        if (gamepad1.a){
            intake.setPower(1);
        }
        else {
            intake.setPower(0);
        }
    }
}
