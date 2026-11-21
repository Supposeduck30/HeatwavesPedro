package org.firstinspires.ftc.teamcode.biobuzz.shooter;

import static org.firstinspires.ftc.teamcode.biobuzz.teleop.BioBuzzConfig.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * PIDF flywheel: the motor runs in RUN_USING_ENCODER and the hub's velocity PIDF holds the target we pass
 * to setVelocity(), using SHOOTER_P / I / D / F from BioBuzzConfig as-is (no battery-voltage scaling).
 *
 * All PIDF constants are in the "PIDF" block of BioBuzzConfig. Tune F and P with "Shooter Graph Test".
 */
public class PIDFShooter implements ShooterControl {

    private final double p, i, d, f;
    private final double intakeTransferPower;

    private DcMotorEx motor;

    /** Uses SHOOTER_P / I / D / F and PIDF_INTAKE_TRANSFER_POWER from BioBuzzConfig. */
    public PIDFShooter() {
        this(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F, PIDF_INTAKE_TRANSFER_POWER);
    }

    /** Uses these values instead of the config (for experiments). */
    public PIDFShooter(double p, double i, double d, double f, double intakeTransferPower) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
        this.intakeTransferPower = intakeTransferPower;
    }

    @Override
    public String name() {
        return "PIDF";
    }

    @Override
    public void init(DcMotorEx motor, HardwareMap hardwareMap) {
        this.motor = motor;
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, i, d, f));
    }

    @Override
    public void update(double targetVelocity) {
        motor.setVelocity(Math.max(0, targetVelocity));
    }

    @Override
    public void stop() {
        motor.setVelocity(0);
    }

    @Override
    public double velocity() {
        return motor.getVelocity();
    }

    @Override
    public double velocityOffset() {
        return PIDF_VELOCITY_OFFSET;
    }

    @Override
    public double intakeTransferPower() {
        return intakeTransferPower;
    }

    @Override
    public double autoFeedPower() {
        return PIDF_AUTO_FEED_POWER;
    }

    @Override
    public double autoReadyHoldSec() {
        return PIDF_AUTO_READY_HOLD_SEC;
    }

    @Override
    public double autoSpinUpTimeoutSec() {
        return PIDF_AUTO_SPIN_UP_TIMEOUT_SEC;
    }

    @Override
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Shooter control", "PIDF  P=%s I=%s D=%s F=%s", p, i, d, f);
        telemetry.addData("Shooter power (from hub)", "%.2f", motor.getPower());
    }

    @Override
    public String tuningSummary() {
        return "P=" + p + " F=" + f;
    }
}
