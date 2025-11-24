package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class BlueSide2 extends TemplateAuto{
     ;

    public void runOpMode() throws InterruptedException {

        super.runOpMode();

        waitForStart();
        moveFront(0.3);
        sleep(700);

       shoot(0.4);
        sleep(1250);
        intake.setPosition(0.1);
        sleep(1500);

        moveFront(0.0);
        intake.setPosition(0.8);
        shoot(0.35);
        sleep(2100);
        intake.setPosition(0.1);
        sleep(3000);

        moveFront(0.0);
        intake.setPosition(0.8);
        shoot(0.4);
        sleep(2000);
        intake.setPosition(0.1);
        sleep(2000);


        moveFront(-0.8);
        sleep(400);

        rotateRight(0.3);
        sleep(400);

        moveFront(-0.6);
        sleep(400);









    }

    public void shoot(double speed){
        shooter1.setPower(-speed);
        shooter2.setPower(speed);
    }
}
