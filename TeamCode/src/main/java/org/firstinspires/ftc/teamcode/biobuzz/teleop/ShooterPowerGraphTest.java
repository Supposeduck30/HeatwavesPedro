package org.firstinspires.ftc.teamcode.biobuzz.teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * Shooter power graph test: runs the shooter at a CONSTANT open-loop power (no PIDF) and plots its speed in
 * Panels, both in ticks/s and in RPM, so you can see what the motor really does at that power.
 *
 * VIEW IT: laptop on the robot's Wi-Fi, open  http://192.168.43.1:8001 , add a GRAPH widget and tick
 * "Velocity (ticks/s)" and/or "Velocity (RPM)". "Power" and "Battery (V)" can go on a second graph.
 *
 * The shooter starts spinning at POWER as soon as you press PLAY.
 * gamepad1: cross (A) = stop / start again.
 *
 * RPM uses the ticks per revolution of the motor type picked in the robot configuration (shown in telemetry).
 * If that motor type is wrong, set TICKS_PER_REV_OVERRIDE to the right number (e.g. 28 for a bare goBILDA /
 * REV HD hex motor encoder). RPM is at the MOTOR; divide by any gear ratio to get the wheel.
 */
@TeleOp(name = "Shooter Power Graph Test", group = "BioBuzz")
public class ShooterPowerGraphTest extends OpMode {

    private static final String SHOOTER = "shooter";
    private static final DcMotorSimple.Direction SHOOTER_DIR = DcMotorSimple.Direction.REVERSE;

    /** Constant power the shooter runs at. */
    private static final double POWER = 0.5;
    /** 0 = use the configured motor type's ticks/rev. Anything above 0 replaces it. */
    private static final double TICKS_PER_REV_OVERRIDE = 0;

    private DcMotorEx shooter;
    private VoltageSensor battery;
    private TelemetryManager panels;
    private double ticksPerRev;
    private boolean spinning = true;
    private boolean lCross;

    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, SHOOTER);
        shooter.setDirection(SHOOTER_DIR);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Plain power, no velocity PID. The encoder is still read for the graph.
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        if (hardwareMap.voltageSensor.iterator().hasNext()) battery = hardwareMap.voltageSensor.iterator().next();

        ticksPerRev = TICKS_PER_REV_OVERRIDE > 0 ? TICKS_PER_REV_OVERRIDE : shooter.getMotorType().getTicksPerRev();

        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        panels.addData("Status", "Ready. Press PLAY: shooter runs at power " + POWER + ".");
        panels.addData("Motor type (config)", shooter.getMotorType().getName());
        panels.addData("Ticks per rev used", ticksPerRev);
        panels.update(telemetry);
    }

    @Override
    public void loop() {
        boolean cross = gamepad1.cross;
        if (cross && !lCross) spinning = !spinning;
        lCross = cross;

        double power = spinning ? POWER : 0;
        shooter.setPower(power);

        double tps = shooter.getVelocity();
        double rpm = ticksPerRev > 0 ? tps * 60.0 / ticksPerRev : 0;

        // Graph keys (numbers).
        panels.addData("Velocity (ticks/s)", tps);
        panels.addData("Velocity (RPM)", rpm);
        panels.addData("Power", power);
        if (battery != null) panels.addData("Battery (V)", battery.getVoltage());

        // Info.
        panels.addData("Shooter", spinning ? "RUNNING at " + POWER + " (cross to stop)" : "STOPPED (cross to start)");
        panels.addData("Motor type / ticks per rev", shooter.getMotorType().getName() + " / " + ticksPerRev);
        panels.update(telemetry);
    }

    @Override
    public void stop() {
        shooter.setPower(0);
    }
}
