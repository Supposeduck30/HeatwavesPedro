package org.firstinspires.ftc.teamcode.InfernoRobot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class autoWheel extends OpMode {
    private DcMotor shooterWheel;
    @Override
    public void init() {
        shooterWheel = hardwareMap.get(DcMotor.class,"shooterWheel");
    }

    @Override
    public void loop() {
        shooterWheel.setPower(1.0
        );
    }
}
