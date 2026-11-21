package org.firstinspires.ftc.teamcode.biobuzz.shooter;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * How the flywheel is driven. The only implementation is {@link PIDFShooter} (the hub's velocity PIDF);
 * every teleop and auto uses it. Its constants, feed powers and auto timings are in BioBuzzConfig.
 */
public interface ShooterControl {

    /** "PIDF", for telemetry. */
    String name();

    /** Put the motor in this mode's run mode. Direction is set by the caller. */
    void init(DcMotorEx motor, HardwareMap hardwareMap);

    /** Call every loop with the target speed in ticks/s. A target of 0 or less turns the flywheel off. */
    void update(double targetVelocity);

    /** Flywheel off. */
    void stop();

    /** Current flywheel speed, ticks/s. */
    double velocity();

    /** Added to SHOOTER_FIXED_VELOCITY for this mode (each mode has its own constant in BioBuzzConfig). */
    double velocityOffset();

    /** Intake AND transfer power when both run together in the teleop (gamepad2 right bumper). */
    double intakeTransferPower();

    /** Intake AND transfer power while shooting / feeding in the autos. */
    double autoFeedPower();

    /** Autos: the flywheel must stay READY this long before the first ball. */
    double autoReadyHoldSec();

    /** Autos: shoot anyway after this long. */
    double autoSpinUpTimeoutSec();

    /** Adds this mode's shooter lines (control values, power) to the telemetry. */
    void addTelemetry(Telemetry telemetry);

    /** One short string with this mode's tuning values, for the "SEND TO CLAUDE" line. */
    String tuningSummary();
}
