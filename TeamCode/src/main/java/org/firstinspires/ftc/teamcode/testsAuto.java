package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class testsAuto  extends TemplateAuto{

    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        shooter1.setPower(-0.7);
        shooter2.setPower(0.7);
        sleep(400);

        intake.setPosition(0.1);

    }
}
