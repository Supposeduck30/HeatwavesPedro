package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class BlueSide extends TemplateAuto{
    private DcMotor shooter1 = null ;
    private DcMotor shooter2 = null ;

    public void runOpMode() throws InterruptedException {

        super.runOpMode();

        shooter1 = hardwareMap.get(DcMotor.class,"SM1");
        shooter2 = hardwareMap.get(DcMotor.class, "SM2");

        waitForStart();
        moveFront(0.3);
        sleep(600);


        shoot(0.5);
        sleep(2000);
        intake.setPosition(0.1);
        sleep(1000);

        moveFront(0.0);
        intake.setPosition(0.8);
        shoot(0.5);
        sleep(2000);
        intake.setPosition(0.1);
        sleep(1000);

        moveFront(0.0);
        intake.setPosition(0.8);
        shoot(0.5);
        sleep(2000);
        intake.setPosition(0.1);
        sleep(2000);

        sleep(2000);
        strafeRight(0.7);
        sleep(1000);








    }

    public void shoot(double speed){
        shooter1.setPower(-speed);
        shooter2.setPower(speed);
    }
}
