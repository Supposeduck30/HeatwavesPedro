package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class tests extends OpMode {
    private DcMotor motorRightFront = null;
    private DcMotor motorLeftFront = null;
    private DcMotor motorRightBack = null;
    private DcMotor motorLeftBack = null;
    private DcMotor shooter1 = null ;
    private DcMotor shooter2 = null ;

    private Servo intake = null;

    public void moveDriveTrain() {
        double vertical;
        double horizontal;
        double pivot;
        vertical =  gamepad1.left_stick_y;
        horizontal = -gamepad1.left_stick_x;
        pivot = gamepad1.right_stick_x;

        motorRightFront.setPower((0.6 * -pivot) + (vertical - horizontal));
        motorRightBack.setPower((0.6 * -pivot) + (vertical + horizontal));
        motorLeftBack.setPower((0.6 * -pivot) + (-vertical + horizontal));
        motorLeftFront.setPower((0.6 * +-pivot) + (-vertical - horizontal));
    }

    @Override
    public void init() {
        shooter1 = hardwareMap.get(DcMotor.class,"SM1");
        shooter2 = hardwareMap.get(DcMotor.class, "SM2");
        intake = hardwareMap.get(Servo.class, "Intake");
        motorRightFront = hardwareMap.get(DcMotor.class, "rf");
        motorLeftBack = hardwareMap.get(DcMotor.class, "lr");
        motorRightBack = hardwareMap.get(DcMotor.class, "rr");
        motorLeftFront = hardwareMap.get(DcMotor.class, "lf");

        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    ElapsedTime timer = new ElapsedTime();
    boolean startedDelay = false;

    @Override
    public void loop() {

        moveDriveTrain();

        if (gamepad1.x) {
            shooter1.setPower(-0.4);
            shooter2.setPower(0.4);
        } else {
            shooter1.setPower(0);
            shooter2.setPower(0);
        }


        if (gamepad1.y) {
            intake.setPosition(0.1);
        } else {
            intake.setPosition(.8);
        }

    } }
