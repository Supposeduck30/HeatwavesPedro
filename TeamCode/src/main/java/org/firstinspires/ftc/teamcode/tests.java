package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class tests extends OpMode {

    private DcMotor shooter = null ;

    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotor.class,"shooter");

    }

    @Override
    public void loop(){
        if(gamepad1.x) {
            shooter.setPower(1);
        } else {
            shooter.setPower(0);
        }
    }
}
