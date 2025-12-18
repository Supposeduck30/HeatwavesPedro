package org.firstinspires.ftc.teamcode.OldRobot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.TemplateAuto;

@Disabled
@Autonomous
public class LongDistance extends TemplateAuto {
    private DcMotor shooter1 = null ;
    private DcMotor shooter2 = null ;

    public void runOpMode() throws InterruptedException {

        super.runOpMode();
        shooter1 = hardwareMap.get(DcMotor.class,"SM1");
        shooter2 = hardwareMap.get(DcMotor.class, "SM2");

        waitForStart();
        moveFront(-0.5);
        sleep(300);

        moveFront(0.0);
        sleep(2000);
        shoot(.55);
        sleep(4000);
        intake.setPosition(0.3);
        sleep(1000);
        intake.setPosition(0.8);

        moveFront(0.0);
        sleep(2000);
        shoot(.45);
        sleep(4000);
        intake.setPosition(0.3);
        sleep(1000);
        intake.setPosition(0.8);

        moveFront(0.0);
        sleep(2000);
        shoot(.6);
        sleep(4000);
        intake.setPosition(0.3);
        sleep(1000);
        intake.setPosition(0.8);



        moveFront(-0.4);
        sleep(600);
        /*intake.setPosition(.8);
        moveFront(0.0);
        shoot(.6);
        sleep(6000);
        intake.setPosition(.1);*/
    }
    public void shoot(double speed){

        shooter1.setPower(-speed);
        shooter2.setPower(speed);
    }
}
