package org.firstinspires.ftc.teamcode.SumoRobot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ShooterTuner extends OpMode {

    private DcMotor motorRightFront;
    private DcMotor motorLeftFront;
    private DcMotor motorRightBack;
    private DcMotor motorLeftBack;
    private DcMotorEx shooter1;
    private DcMotor intake;
    private Servo kicker;

    public double highVelocity = 2060;
    public double lowVelocity = 2000;

    double curTargetVelocity = highVelocity;



    double F = 0;
    double P = 0;

    double[] stepSizes = {10.0, 1.0, 0.1, 0.001,0.0001};

    int stepIndex = 1;

    @Override
    public void init() {

        // Drive motors

        // Other motors
        shooter1 = hardwareMap.get(DcMotorEx.class, "Shooter1");

        intake  = hardwareMap.get(DcMotor.class, "Intake");
        kicker  = hardwareMap.get(Servo.class, "Kicker");


        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
        telemetry.addLine("Init complete");

    }

    @Override
    public void loop() {

        if (gamepad1.left_bumper) {
            intake.setPower(0.80);
        }

        if (gamepad1.yWasPressed()) {

            if (curTargetVelocity == highVelocity) {
                curTargetVelocity = lowVelocity;
            } else {
                curTargetVelocity = highVelocity;}
        }

        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1)% stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            F += stepSizes[stepIndex];
        }

        if (gamepad1.dpadRightWasPressed()) {
            F-=stepSizes[stepIndex];
        }

        if (gamepad1.dpadDownWasPressed()) {
            P += stepSizes[stepIndex];
        }

        if (gamepad1.dpadUpWasPressed()) {
            P-= stepSizes[stepIndex];
        }

        if (gamepad1.right_bumper) {
            kicker.setPosition(0.6);
        } else {
            kicker.setPosition(0.31);
        }

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);

        shooter1.setVelocity(curTargetVelocity);

        double curVelocity = shooter1.getVelocity();
        double error = curTargetVelocity - curVelocity;

        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity", "%.2f", curVelocity);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addLine("-------------------");
        telemetry.addData("Tuning P", "%.4f (D-Pad U/D" , P);
        telemetry.addData("Tuning F", "%.4f (D-Pad L/R" , F);
        telemetry.addData("Step Size", "%.4f (B Button)" , stepSizes[stepIndex]);

    }
}
