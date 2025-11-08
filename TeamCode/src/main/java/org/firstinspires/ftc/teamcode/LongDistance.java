package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class LongDistance extends TemplateAuto{
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
        sleep(4000);
        shoot(.6);
        sleep(3000);
        intake.setPosition(0.1);
    }
    public void shoot(double speed){
        shooter1.setPower(-speed);
        shooter2.setPower(speed);
    }
}
