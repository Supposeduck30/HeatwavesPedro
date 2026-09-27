package org.firstinspires.ftc.teamcode.DecodeBot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class ShootingTester extends OpMode {
    private DcMotor shooter1;

    @Override
    public void init() {
        shooter1 = hardwareMap.get(DcMotorEx.class, "Shooter1");
    }

    @Override
    public void loop() {
        shooter1.setPower(0.6);
    }
}
