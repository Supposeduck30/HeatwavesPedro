package org.firstinspires.ftc.teamcode.SumoRobot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class shoot extends OpMode {
    private DcMotor shooterWheel = null ;
    private DcMotor intake = null;
    private Servo kicker = null;



    @Override

    public void init() {
        shooterWheel = hardwareMap.get(DcMotor.class,"Shooter");
        intake = hardwareMap.get(DcMotor.class,"Intake");
        kicker = hardwareMap.get(Servo.class, "Kicker");
    }

    @Override
    public void loop() {
        if(gamepad2.a){
            kicker.setPosition(0.6);
        } else {

            kicker.setPosition(0.35);
        }
        if(gamepad2.b){

            intake.setPower(0.8);
        }else {
            intake.setPower(0.0);


        }
        if (gamepad2.x){
            shooterWheel.setPower(1.0);
        }else{
            shooterWheel.setPower(0.0);
        }
    }
}
