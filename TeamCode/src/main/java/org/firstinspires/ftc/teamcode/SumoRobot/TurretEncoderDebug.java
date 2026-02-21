package org.firstinspires.ftc.teamcode.SumoRobot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "DEBUG: Turret Encoder Test", group = "Debug")
public class TurretEncoderDebug extends OpMode {

    private DcMotorEx turretMotor;

    // Tracking variables
    private int lastTicks = 0;
    private long lastTime = 0;

    // Fault counters
    private int spikeFaults = 0;
    private int deadFaults = 0;
    private int zeroFaults = 0;

    // Physical limits of a 5203 435RPM motor
    // Max theoretical speed is ~2800 ticks per second.
    private final double MAX_POSSIBLE_VELOCITY = 3500.0;

    @Override
    public void init() {
        // MATCH THIS TO YOUR CONFIG ("Turret" or "turret")
        turretMotor = hardwareMap.get(DcMotorEx.class, "Turret");

        // Raw mode: No PID, just voltage
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry.addLine("Encoder Debugger Ready.");
        telemetry.addLine("USE GAMEPAD 1 LEFT STICK TO SPIN TURRET.");
        telemetry.update();
    }

    @Override
    public void start() {
        lastTicks = turretMotor.getCurrentPosition();
        lastTime = System.currentTimeMillis();
    }

    @Override
    public void loop() {
        // 1. RAW CONTROL (Limit power to 50% for safety while testing)
        double power = -gamepad1.left_stick_x * 0.5;
        turretMotor.setPower(power);

        // 2. READ DATA
        int currentTicks = turretMotor.getCurrentPosition();
        long currentTime = System.currentTimeMillis();

        double dt = (currentTime - lastTime) / 1000.0;
        if (dt <= 0) dt = 0.001; // Prevent divide by zero

        double deltaTicks = Math.abs(currentTicks - lastTicks);
        double ticksPerSecond = deltaTicks / dt;

        // 3. FAULT DETECTION

        // Fault A: Electrical Spike (Impossible velocity)
        // Usually caused by Static Electricity (ESD) jumping to the encoder wire
        if (ticksPerSecond > MAX_POSSIBLE_VELOCITY) {
            spikeFaults++;
        }

        // Fault B: Dead Wire (Motor has power, but encoder doesn't move)
        // Usually a loose JST connector or broken wire trace
        if (Math.abs(power) > 0.3 && deltaTicks == 0) {
            deadFaults++;
        }

        // Fault C: Instant Zero (Encoder resets itself randomly)
        // Usually a severe power drop to the 3.3v rail on the encoder level shifter
        if (currentTicks == 0 && Math.abs(lastTicks) > 50) {
            zeroFaults++;
        }

        // 4. TELEMETRY
        telemetry.addLine("=== RAW HARDWARE DATA ===");
        telemetry.addData("Motor Power", "%.2f", power);
        telemetry.addData("Current Ticks", currentTicks);
        telemetry.addData("Velocity (Ticks/Sec)", "%.1f", ticksPerSecond);

        telemetry.addLine("\n=== FAULT DETECTORS ===");
        telemetry.addData("SPIKES (Velocity too high)", spikeFaults).addData("->", spikeFaults > 0 ? "FAIL (Static/Short)" : "OK");
        telemetry.addData("DEAD (No data while moving)", deadFaults).addData("->", deadFaults > 10 ? "FAIL (Loose Wire)" : "OK");
        telemetry.addData("ZEROS (Random resets)", zeroFaults).addData("->", zeroFaults > 0 ? "FAIL (Power Drop)" : "OK");

        telemetry.addLine("\nHow to test:");
        telemetry.addLine("1. Spin the motor slowly with the joystick.");
        telemetry.addLine("2. With your other hand, WIGGLE the encoder wire.");
        telemetry.addLine("3. Watch for faults to trigger.");

        telemetry.update();

        // Save for next loop
        lastTicks = currentTicks;
        lastTime = currentTime;
    }
}