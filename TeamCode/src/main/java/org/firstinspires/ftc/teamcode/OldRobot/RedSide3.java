package org.firstinspires.ftc.teamcode.OldRobot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.TemplateAuto;

@Disabled
@Autonomous
public class RedSide3 extends TemplateAuto {
    private DcMotor shooter1 = null ;
    private DcMotor shooter2 = null ;

    public void runOpMode() throws InterruptedException {

        super.runOpMode();

        shooter1 = hardwareMap.get(DcMotor.class,"SM1");
        shooter2 = hardwareMap.get(DcMotor.class, "SM2");

        waitForStart();
        moveFront(0.3);
        sleep(700);

        shoot(0.43);
        sleep(2450);
        moveFront(0.0);
        intake.setPosition(0.2);
        sleep(750);
        intake.setPosition(1);

        moveFront(0.0);
        shoot(0.47);
        sleep(2300);
        intake.setPosition(0.2);
        sleep(1000);
        intake.setPosition(1.0);
        sleep(3000);

        moveFront(0.0);
        shoot(0.4);
        sleep(2000);
        intake.setPosition(0.2);
        sleep(1000);
        intake.setPosition(1.0);
        sleep(2000);


        moveFront(-0.8);
        sleep(400);

        rotateRight(-0.3);
        sleep(400);

        moveFront(-0.6);
        sleep(400);









    }

    public void shoot(double speed){
        shooter1.setPower(-speed);
        shooter2.setPower(speed);
    }
}
