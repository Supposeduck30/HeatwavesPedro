package org.firstinspires.ftc.teamcode.SumoRobot.SUMOAUTOS;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.TemplateAuto;

@Autonomous
public class Intake extends TemplateAuto {

    private DcMotor intake = null;

    public void runOpMode() throws InterruptedException {

        intake = hardwareMap.get(DcMotor.class,"Intake");
        waitForStart();

        intake.setPower(0.8);
        sleep(10000);
    }
}
