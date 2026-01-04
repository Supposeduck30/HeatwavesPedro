package org.firstinspires.ftc.teamcode.SumoRobot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Disabled
@TeleOp
public class FieldCentric extends OpMode {

    // Drive motors
    private DcMotor motorRightFront;
    private DcMotor motorLeftFront;
    private DcMotor motorRightBack;
    private DcMotor motorLeftBack;

    // Other hardware
    private DcMotorEx shooter1;
    private DcMotor intake;
    private Servo kicker;

    // Internal IMU
    private IMU imu;

    public double highVelocity = 2300;
    public double lowVelocity = 1500;

    double curTargetVelocity = highVelocity;

    @Override
    public void init() {

        // Drive motors
        motorRightFront = hardwareMap.get(DcMotor.class, "FR");
        motorLeftFront  = hardwareMap.get(DcMotor.class, "FL");
        motorRightBack  = hardwareMap.get(DcMotor.class, "BR");
        motorLeftBack   = hardwareMap.get(DcMotor.class, "BL");

        // Other motors
        shooter1 = hardwareMap.get(DcMotorEx.class, "Shooter1");
        intake   = hardwareMap.get(DcMotor.class, "Intake");
        kicker   = hardwareMap.get(Servo.class, "Kicker");

        // Motor directions (UNCHANGED)
        motorRightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLeftBack.setDirection(DcMotorSimple.Direction.FORWARD);

        // Shooter setup (UNCHANGED)
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(100, 0, 0, 15.5);
        shooter1.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                pidfCoefficients
        );

        // Brake mode
        motorRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ===== INTERNAL IMU (SIDEWAYS HUB) =====
        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                );

        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    @Override
    public void loop() {

        // Drive
        moveDriveTrain();

        // Kicker (UNCHANGED)
        if (gamepad2.right_bumper) {
            kicker.setPosition(0.6);
        } else {
            kicker.setPosition(0.31);
        }

        // Intake + Shooter (UNCHANGED)
        if (gamepad2.left_bumper) {
            intake.setPower(0.9);
            shooter1.setVelocity(curTargetVelocity);
        } else {
            intake.setPower(0.0);
            shooter1.setVelocity(0.0);
        }
    }

    // ================= FIELD-CENTRIC DRIVE =================
    public void moveDriveTrain() {

        // Gamepad input
        double y  = -gamepad1.left_stick_y;  // forward
        double x  =  gamepad1.left_stick_x;  // strafe
        double rx =  gamepad1.right_stick_x; // rotate

        // Heading from IMU
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        double heading = angles.getYaw(AngleUnit.RADIANS);

        // Field-centric rotation
        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

        // Mecanum math
        double frontLeft  = rotY + rotX + rx;
        double backLeft   = rotY - rotX + rx;
        double frontRight = rotY - rotX - rx;
        double backRight  = rotY + rotX - rx;

        // Normalize
        double max = Math.max(
                Math.max(Math.abs(frontLeft), Math.abs(backLeft)),
                Math.max(Math.abs(frontRight), Math.abs(backRight))
        );

        if (max > 1.0) {
            frontLeft  /= max;
            backLeft   /= max;
            frontRight /= max;
            backRight  /= max;
        }

        // Apply power
        motorLeftFront.setPower(frontLeft);
        motorLeftBack.setPower(backLeft);
        motorRightFront.setPower(frontRight);
        motorRightBack.setPower(backRight);
    }
}
