package org.firstinspires.ftc.teamcode.SumoRobot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@TeleOp(name = "Blue SUMO TeleOp")
public class blueSUMOTELEOP extends OpMode {

    // Drive motors
    private DcMotor motorRightFront;
    private DcMotor motorLeftFront;
    private DcMotor motorRightBack;
    private DcMotor motorLeftBack;

    // Other hardware
    private DcMotor shooter;
    private DcMotor intake;
    private Servo kicker;

    // Pedro Pathing
    private Follower follower;
    private boolean autoDriving = false;

    // Target position
    static final double TARGETX = 59.54530477759472;
    static final double TARGETY = 85.4036243822076;
    static final double HEADING = 138;

    @Override
    public void init() {

        motorRightFront = hardwareMap.get(DcMotor.class, "FR");
        motorLeftFront  = hardwareMap.get(DcMotor.class, "FL");
        motorRightBack  = hardwareMap.get(DcMotor.class, "BR");
        motorLeftBack   = hardwareMap.get(DcMotor.class, "BL");

        shooter = hardwareMap.get(DcMotor.class, "Shooter");
        intake  = hardwareMap.get(DcMotor.class, "Intake");
        kicker  = hardwareMap.get(Servo.class, "Kicker");

        motorRightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLeftBack.setDirection(DcMotorSimple.Direction.FORWARD);

        motorRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
    }

    @Override
    public void loop() {

        follower.update();

        // Start path on X press
        if (gamepad1.x && !autoDriving) {
            Path path = new Path(
                    new BezierLine(
                            follower.getPose(),
                            new Pose(TARGETX, TARGETY, Math.toRadians(HEADING))
                    )
            );
            follower.followPath(path);
            autoDriving = true;
        }

        // Stop auto once finished
        if (autoDriving && !follower.isBusy()) {
            autoDriving = false;
        }

        // Manual drive only when not auto pathing
        if (!autoDriving) {
            moveDriveTrain();
        }

        // Kicker
        if (gamepad2.right_bumper) {
            kicker.setPosition(0.6);
        } else {
            kicker.setPosition(0.31);
        }

        // Intake + Shooter
        if (gamepad2.left_bumper) {
            intake.setPower(0.9);
            shooter.setPower(0.9);
        } else {
            intake.setPower(0);
            shooter.setPower(0);
        }
    }

    public void moveDriveTrain() {
        double vertical = -gamepad1.left_stick_y;
        double horizontal = gamepad1.left_stick_x;
        double pivot = -gamepad1.right_stick_x;

        motorRightFront.setPower(-(0.6 * pivot) + (-vertical + horizontal));
        motorRightBack.setPower(-(0.6 * -pivot) + (vertical + horizontal));
        motorLeftBack.setPower(-(0.6 * pivot) + (vertical - horizontal));
        motorLeftFront.setPower(-(0.6 * -pivot) + (-vertical - horizontal));
    }
}
