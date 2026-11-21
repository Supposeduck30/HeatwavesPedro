package org.firstinspires.ftc.teamcode.biobuzz.shooter;

import static org.firstinspires.ftc.teamcode.biobuzz.teleop.BioBuzzConfig.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Bang-bang flywheel: open loop (RUN_WITHOUT_ENCODER, BRAKE at zero power). Full power while the flywheel is
 * below (target - hysteresis), zero above (target + hysteresis), and the previous state in between. The
 * encoder is only read for velocity.
 *
 * All bang-bang constants are in the "BANG BANG" block of BioBuzzConfig. This class never touches the
 * SHOOTER_P / I / D / F constants or setVelocity.
 */
public class BangBangShooter implements ShooterControl {

    private DcMotorEx motor;
    private boolean on = false;

    @Override
    public String name() {
        return "Bang Bang";
    }

    @Override
    public void init(DcMotorEx motor, HardwareMap hardwareMap) {
        this.motor = motor;
        // Open loop: we set the power ourselves; the encoder is only read for velocity.
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // BRAKE at zero power (what bang-bang was originally tuned with), same as BangBangTuner.
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void update(double targetVelocity) {
        if (targetVelocity <= 0) {
            stop();
            return;
        }
        double velocity = motor.getVelocity();
        if (velocity < targetVelocity - BANG_BANG_HYSTERESIS) {
            on = true;
        } else if (velocity > targetVelocity + BANG_BANG_HYSTERESIS) {
            on = false;
        }
        motor.setPower(on ? BANG_BANG_ON_POWER : BANG_BANG_OFF_POWER);
    }

    @Override
    public void stop() {
        on = false;
        motor.setPower(0);
    }

    @Override
    public double velocity() {
        return motor.getVelocity();
    }

    @Override
    public double velocityOffset() {
        return BANG_BANG_VELOCITY_OFFSET;
    }

    @Override
    public double intakeTransferPower() {
        return BANG_BANG_INTAKE_TRANSFER_POWER;
    }

    @Override
    public double autoFeedPower() {
        return BANG_BANG_AUTO_FEED_POWER;
    }

    @Override
    public double autoReadyHoldSec() {
        return BANG_BANG_AUTO_READY_HOLD_SEC;
    }

    @Override
    public double autoSpinUpTimeoutSec() {
        return BANG_BANG_AUTO_SPIN_UP_TIMEOUT_SEC;
    }

    @Override
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Shooter control", "BANG BANG  on=%.2f off=%.2f hysteresis=%.0f",
                BANG_BANG_ON_POWER, BANG_BANG_OFF_POWER, BANG_BANG_HYSTERESIS);
        telemetry.addData("Shooter power", "%.1f", on ? BANG_BANG_ON_POWER : BANG_BANG_OFF_POWER);
    }

    @Override
    public String tuningSummary() {
        return "on=" + BANG_BANG_ON_POWER + " off=" + BANG_BANG_OFF_POWER + " hyst=" + BANG_BANG_HYSTERESIS;
    }
}
