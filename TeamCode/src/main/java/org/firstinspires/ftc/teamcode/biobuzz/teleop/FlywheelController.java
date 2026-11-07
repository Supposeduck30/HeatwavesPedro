package org.firstinspires.ftc.teamcode.biobuzz.teleop;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * Voltage-compensated feedforward + bang-bang flywheel control.
 *
 * Plain bang-bang (1.0 / 0.0) behaves differently on every battery: a fresh battery accelerates the
 * wheel harder during each "on" burst, so it overshoots further before the next loop sees it. This
 * controller instead runs the wheel at the power that HOLDS the target, corrected for battery voltage,
 * and only nudges above or below that:
 *
 *   hold  = (KS + KV * target) * (NOMINAL_VOLTAGE / batteryVoltage)
 *   far below target (error > RECOVER_ERROR)   -> 1.0           (fast recovery after a shot)
 *   below target - BAND                         -> hold + BOOST  (spin up)
 *   inside the band                             -> hold          (hold speed)
 *   above target + BAND                         -> hold - CUT    (let it ease down, not slam to 0)
 *
 * The motor is left in RUN_WITHOUT_ENCODER (we set the power ourselves) and set to FLOAT, so "less
 * power" means coasting, never shorting the motor into a brake.
 */
public class FlywheelController {

    // ================= TUNE =================
    /**
     * Power per tick/s needed to hold speed at NOMINAL_VOLTAGE. Starting value comes from the F you
     * measured with the hub PIDF (F = 13.02, where 32767 = full power): 13.02 / 32767.
     * Tune so that "inside the band" holds the target with no help.
     */
    public static final double KV = 13.02 / 32767.0;
    /** Static friction power (power just to keep the wheel turning). Start at 0. */
    public static final double KS = 0.0;
    /** Battery voltage when KV was tuned. Set this to what the battery read when you tuned F. */
    public static final double NOMINAL_VOLTAGE = 12.5;
    /** Extra power (at nominal voltage) while below the band. */
    public static final double BOOST = 0.25;
    /** Power taken off the hold power (at nominal voltage) while above the band. */
    public static final double CUT = 0.15;
    /** Half-width of the hold band, ticks/s. */
    public static final double BAND = 20;
    /** This far below the target (e.g. right after a shot), use full power to recover fast. */
    public static final double RECOVER_ERROR = 150;

    /** Voltage low-pass filter (0..1, lower = smoother). The reading sags during every power burst. */
    public static final double VOLTAGE_ALPHA = 0.1;
    /** Only read the voltage sensor every N updates (each read costs a hub transaction). */
    public static final int VOLTAGE_READ_EVERY = 5;
    /** Ignore readings below this (sensor glitch / brownout) so the compensation can't explode. */
    public static final double MIN_VALID_VOLTAGE = 9.0;

    public enum Mode { RECOVER, SPIN_UP, HOLD, EASE_DOWN, STOPPED }

    private final DcMotorEx motor;
    private final VoltageSensor voltageSensor;

    private double filteredVoltage = NOMINAL_VOLTAGE;
    private int updatesSinceVoltageRead = VOLTAGE_READ_EVERY; // read on the first update
    private double velocity, power, hold, comp = 1.0;
    private Mode mode = Mode.STOPPED;

    /** The motor must already be configured (direction, RUN_WITHOUT_ENCODER) by the caller. */
    public FlywheelController(DcMotorEx motor, HardwareMap hardwareMap) {
        this.motor = motor;
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.voltageSensor = hardwareMap.voltageSensor.iterator().hasNext()
                ? hardwareMap.voltageSensor.iterator().next()
                : null;
    }

    /** Run one control step toward targetVelocity (ticks/s). Returns the power applied. */
    public double update(double targetVelocity) {
        readVoltage();
        velocity = motor.getVelocity();

        if (targetVelocity <= 0) {
            mode = Mode.STOPPED;
            hold = 0;
            power = 0;
            motor.setPower(0);
            return 0;
        }

        comp = NOMINAL_VOLTAGE / filteredVoltage;
        hold = (KS + KV * targetVelocity) * comp;
        double error = targetVelocity - velocity;

        if (error > RECOVER_ERROR) {
            mode = Mode.RECOVER;
            power = 1.0;
        } else if (error > BAND) {
            mode = Mode.SPIN_UP;
            power = hold + BOOST * comp;
        } else if (error < -BAND) {
            mode = Mode.EASE_DOWN;
            power = hold - CUT * comp;
        } else {
            mode = Mode.HOLD;
            power = hold;
        }

        power = Range.clip(power, 0.0, 1.0);
        motor.setPower(power);
        return power;
    }

    public void stop() {
        mode = Mode.STOPPED;
        power = 0;
        motor.setPower(0);
    }

    private void readVoltage() {
        if (voltageSensor == null) return;
        if (++updatesSinceVoltageRead < VOLTAGE_READ_EVERY) return;
        updatesSinceVoltageRead = 0;
        double v = voltageSensor.getVoltage();
        if (v >= MIN_VALID_VOLTAGE) {
            filteredVoltage += VOLTAGE_ALPHA * (v - filteredVoltage);
        }
    }

    public double getVelocity()        { return velocity; }
    public double getPower()           { return power; }
    public double getHoldPower()       { return hold; }
    public double getVoltage()         { return filteredVoltage; }
    public double getCompensation()    { return comp; }
    public Mode getMode()              { return mode; }
    public boolean hasVoltageSensor()  { return voltageSensor != null; }
}
