package org.firstinspires.ftc.teamcode.InfernoRobot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class wheel extends OpMode {

    private DcMotor shooterWheel;
    @Override
    public void init() {
        shooterWheel = hardwareMap.get(DcMotor.class,"shooterWheel");
    }

    @Override
    public void loop() {
        if(gamepad1.a){
            shooterWheel.setPower(0.5);
        } else {
            shooterWheel.setPower(0.0);
        }
        shooterWheel.setPower(gamepad1.left_trigger);
    }
}
