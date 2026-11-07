package org.firstinspires.ftc.teamcode.biobuzz.teleop;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Every number you will touch while tuning the BioBuzz robot lives here.
 * Values marked TUNE were carried over from the Decode robot and are only a starting point
 * (the new robot is smaller, so expect all of them to move).
 */
public final class BioBuzzConfig {
    private BioBuzzConfig() {}

    // ================= HARDWARE NAMES =================
    public static final String SHOOTER      = "shooter";
    public static final String INTAKE       = "intake";
    public static final String TRANSFER     = "transfer";
    public static final String BLOCKER   = "blocker";
    public static final String TURRET    = "Turret";

    public static final DcMotorSimple.Direction SHOOTER_DIR   = DcMotorSimple.Direction.REVERSE; // scrimmage teleop ran it at setPower(-0.5), so + velocity needs REVERSE
    public static final DcMotorSimple.Direction INTAKE_DIR    = DcMotorSimple.Direction.FORWARD; // TUNE
    public static final DcMotorSimple.Direction TRANSFER_DIR  = DcMotorSimple.Direction.REVERSE; // from the scrimmage teleop
    /** Rotate the turret by hand counter-clockwise (seen from above): the angle telemetry must INCREASE. If not, flip this. */
    public static final DcMotorSimple.Direction TURRET_DIR    = DcMotorSimple.Direction.REVERSE; // TUNE

    // ================= FIELD / ROBOT GEOMETRY =================
    // Alliance-specific values (start pose, goals, park pose, shooter regression, trim) are in BioBuzzAlliance.

    // ================= HOLD-TO-POINT (gamepad1, hold the button) =================
    // The shoot poses (left / right bumper) differ per alliance, see BioBuzzAlliance.
    // TODO: the gate pose is a Decode field position. Replace with BioBuzz-season positions.
    public static final Pose EMPTY_GATE_POSE  = new Pose(132.5, 68.5, Math.toRadians(180)); // square
    // Park pose (circle) differs per alliance, see BioBuzzAlliance.

    /** Turret pivot relative to the robot center: forward (+) / left (+) in inches. TUNE for the smaller robot. */
    public static final double TURRET_OFFSET_FORWARD = -6.0;
    public static final double TURRET_OFFSET_LEFT    = 0.0;

    // ================= TURRET =================
    /** Encoder ticks per degree of TURRET rotation (motor CPR * gear ratio / 360). TUNE */
    public static final double TURRET_TICKS_PER_DEGREE = 366.0 / 90.0;
    /** Turret angle 0 = pointing straight out the front of the robot, positive = counter-clockwise. */
    public static final double TURRET_START_ANGLE_DEG = 0.0;

    public static final double TURRET_KP        = 0.065; // TUNE
    public static final double TURRET_KD        = 0.004; // TUNE
    public static final double TURRET_MAX_POWER = 1.0;
    /** Minimum power to overcome friction while still outside the deadband. TUNE */
    public static final double TURRET_MIN_POWER = 0.08;
    public static final double TURRET_DEADBAND_DEG = 0.10;

    /** How fast gamepad2 dpad left/right moves the trim. */
    public static final double TURRET_TRIM_RATE_DEG_PER_SEC = 10.0;

    // ================= PREDICTIVE AIMING =================
    public static final double PROJECTILE_SPEED_IN_PER_SEC = 193.75; // TUNE
    public static final double SYSTEM_DELAY_SECONDS        = 0.0;    // TUNE
    public static final double MIN_SPEED_FOR_PREDICTION    = 2.0;    // in/s

    // ================= SHOOTER =================
    // velocity (ticks/s) = A*d^2 + B*d + C per alliance, see BioBuzzAlliance. Re-run your regression. TUNE
    /** true = the shooter always runs at SHOOTER_FIXED_VELOCITY; false = use the distance regression below. */
    public static final boolean USE_FIXED_SHOOTER_VELOCITY = true;
    /** Shooter target (ticks/s) while USE_FIXED_SHOOTER_VELOCITY is on. (F was tuned at 1450.) */
    public static final double SHOOTER_FIXED_VELOCITY = 1480;
    /** Multiplies the regression output. 1.0 = the regression as-is; 1.5 = 50% faster. TUNE */
    public static final double SHOOTER_VELOCITY_SCALE = 3.0;
    /** Safety cap on the final target (ticks/s). Above the motor's real max it just means full power. */
    public static final double MAX_SHOOTER_VELOCITY = 2300;
    /** Telemetry says READY when the flywheel is within this many ticks/s of the target. */
    public static final double SHOOTER_READY_TOLERANCE = 50;

    // Bang-bang shooter control (TeleOps): full power below the target, zero above it.
    /** Power applied while the flywheel is below the target. */
    public static final double BANG_BANG_ON_POWER = 0.7;
    /** Power applied once the flywheel has reached the target. */
    public static final double BANG_BANG_OFF_POWER = 0.4;
    /** Ticks/s band around the target: turns on below (target - H), off above (target + H), holds in between. */
    public static final double BANG_BANG_HYSTERESIS = 4; // og value is 5

    public static final double SHOOTER_P = 120, SHOOTER_I = 0, SHOOTER_D = 0, SHOOTER_F = 13; //F was 14.9, F was 12.6; after +250g the tuner read +200 error (too slow, ~1300 at target 1500), so 14.5, then avg error +14 at P=0, so 14.6, then avg error +20 at P=120, so ~14.8. P=120 F=14.9 tested in teleop: settle 1.98s, overshoot 40, max dip 100, recovery 0.24s. Then ShooterVelocityTuner at 1400 showed error 40-80 oscillating, so P lowered 120 -> 100 (estimate). Re-check with FlywheelPIDFTuner.

    // ================= BLOCKER =================
    public static final double BLOCKER_BLOCK = 0.63;
    public static final double BLOCKER_OPEN  = 0.85;

    // ================= FLOWER =================l
    public static final String FLOWER = "Flower1";
    /** Default position. */
    public static final double FLOWER_REST   = 0.2;
    /** Position while toggled on with gamepad1 cross. */
    public static final double FLOWER_ACTIVE = 1.0;

    // ================= INTAKE =================
    public static final double INTAKE_IN_POWER      = 1.0;
    public static final double INTAKE_REVERSE_POWER = -0.9;
    /** Intake AND transfer power while shooting (gamepad2 left bumper) at the primary / secondary goal. */
    public static final double PRIMARY_FEED_POWER   = 1.0;
    public static final double SECONDARY_FEED_POWER = 1.0; // Decode intended this slower; lower it if you want

    // ================= TRANSFER =================
    public static final double TRANSFER_IN_POWER      = 1.0;  // TUNE
    public static final double TRANSFER_REVERSE_POWER = -0.9; // TUNE
}
