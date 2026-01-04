package org.firstinspires.ftc.teamcode.SumoRobot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@TeleOp
public class driveTrain extends OpMode {

    private DcMotor motorRightFront = null;
    private DcMotor motorLeftFront = null;
    private DcMotor motorRightBack = null;
    private DcMotor motorLeftBack = null;

    public void moveDriveTrain() {
        double vertical;
        double horizontal;
        double pivot;
        vertical =  -gamepad1.left_stick_y;
        horizontal = gamepad1.left_stick_x;
        pivot = -gamepad1.right_stick_x;

        motorRightBack.setDirection(DcMotor.Direction.REVERSE);
        motorLeftFront.setDirection(DcMotorSimple.Direction.FORWARD);


        motorRightFront.setPower(-(0.6 * -pivot) + (vertical - horizontal));
        motorRightBack.setPower(-(0.6 * -pivot) + (vertical + horizontal));
        motorLeftBack.setPower(-(0.6 * -pivot) + (-vertical + horizontal));
        motorLeftFront.setPower(-(0.6 * -pivot) + (-vertical - horizontal));
    }

    @Override
    public void init() {
        motorRightFront = hardwareMap.get(DcMotor.class, "FR");
        motorLeftBack = hardwareMap.get(DcMotor.class, "BL");
        motorRightBack = hardwareMap.get(DcMotor.class, "BR");
        motorLeftFront = hardwareMap.get(DcMotor.class, "FL");
    }

    @Override
    public void loop() {

        moveDriveTrain();

    }
}
