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
    public static final String TURRET    = "turret";

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
    public static final double TURRET_OFFSET_FORWARD = 0.0; // turret is at the robot center (was -6.0)
    public static final double TURRET_OFFSET_LEFT    = 0.0;

    // ================= TURRET =================
    /** Turret motor: goBILDA 1150 rpm Yellow Jacket, 145.1 encoder ticks per motor-shaft revolution. */
    public static final double TURRET_MOTOR_TICKS_PER_REV = 145.1;
    /** Infinite turret gearing: 30 tooth motor gear drives the 116 tooth turret gear (turret turns 30/116 rev per motor rev). */
    public static final double TURRET_GEAR_TEETH_MOTOR  = 30;
    public static final double TURRET_GEAR_TEETH_TURRET = 116;
    /** Encoder ticks per degree of TURRET rotation = 145.1 * 116/30 / 360 = ~1.5585 (561 ticks per full turret turn). */
    public static final double TURRET_TICKS_PER_DEGREE =
            TURRET_MOTOR_TICKS_PER_REV * (TURRET_GEAR_TEETH_TURRET / TURRET_GEAR_TEETH_MOTOR) / 360.0;
    /** Turret angle convention: 0 = robot RIGHT, 90 = robot FRONT, 180 = left, 270 = back, positive = counter-clockwise (infinite, no wrap). */
    public static final double TURRET_FRONT_ANGLE_DEG = 90.0;
    /** The turret is started pointing straight out the front, so it begins at the front angle. */
    public static final double TURRET_START_ANGLE_DEG = TURRET_FRONT_ANGLE_DEG;

    // ================= TURRET HOMING (magnet limit switch) =================
    // The belt/gears skip, so the encoder angle drifts. The magnet fixes it: the switch reads "detected" over a band
    // of ~4 deg, so we only ever use ONE edge of it: the LEFT (CCW-side, higher-angle) edge, which is the edge the
    // turret hits when it is spinning CLOCKWISE. Every clockwise entry into the magnet re-homes the angle, and the
    // start / button homing always finishes by spinning clockwise into it. Counter-clockwise entries are ignored.
    /** Magnetic limit switch, configured as a "REV Touch Sensor" in the robot config. */
    public static final String TURRET_LIMIT = "turretLimit";
    /**
     * Turret angle at the LEFT edge of the magnet band (where it triggers while spinning CLOCKWISE).
     * "Turret Limit Finder" 2026-11-20: CW leading edge 33.12 (band ~29.8..33.1, center 31.5).
     * "Turret Magnet Angle Test" 2026-11-21: CW ENTER at 42.52 (this frame, front = 90; = -47.48 when front = 0). NaN = homing off. TUNE
     */
    public static final double TURRET_LIMIT_ANGLE_DEG = 42.52;
    /** Re-home on every clockwise pass over the magnet while aiming (false = only on start / button homing). */
    public static final boolean TURRET_REHOME_ON_PASS = true;
    /**
     * Ignore a pass if the turret moved more than this in one loop: the edge is placed halfway between two loops, so a
     * fast pass is less exact. Max error is about half of this.
     */
    public static final double TURRET_REHOME_MAX_STEP_DEG = 4.0;
    /** Clockwise seek power for start / button homing. Slow and constant (the finder sweep was 0.15). TUNE */
    public static final double TURRET_HOME_POWER = 0.15;
    /** Counter-clockwise back-off power, used when homing starts while already on the magnet. */
    public static final double TURRET_HOME_BACKOFF_POWER = 0.2;
    /** How far past the magnet to back off (CCW) before seeking clockwise. */
    public static final double TURRET_HOME_BACKOFF_DEG = 20.0;
    /** Give up if seeking clockwise goes this far without finding the magnet (more than one full turn). */
    public static final double TURRET_HOME_MAX_SEEK_DEG = 400.0;
    /** Give up if backing off goes this far and the magnet still reads as detected. */
    public static final double TURRET_HOME_MAX_BACKOFF_DEG = 90.0;

    // KP / KD are the Decode values (that turret was ~2.6x more ticks per degree and a slower motor). The 1150 rpm
    // motor on 30:116 is much faster (~1800 deg/s free), so if it overshoots / buzzes lower KP, or raise KD. TUNE
    /**
     * +1 or -1. Positive power must make the turret ANGLE (encoder) go up. 2026-11-20 test: angle ran away to 180 deg
     * off (positive feedback), so this is -1. (Flipping TURRET_DIR does NOT fix that: it flips power and encoder together.)
     */
    public static final double TURRET_POWER_SIGN = -1.0;
    // Softened 2026-11-20 (was KP 0.065, KD 0.004, max 1.0, min 0.08, deadband 0.10, which buzzed: one encoder tick is ~0.64 deg).
    public static final double TURRET_KP        = 0.04; // TUNE
    public static final double TURRET_KD        = 0.0005; // TUNE (was 0.0015; encoder-tick velocity is noisy and D was cancelling the small-error push)
    public static final double TURRET_MAX_POWER = 0.8;
    /** Minimum power to overcome friction while still outside the deadband. TUNE (was 0.10; turret stalled ~3 deg short, so friction needs more) */
    public static final double TURRET_MIN_POWER = 0.16;
    public static final double TURRET_DEADBAND_DEG = 1.0; // ~1.5 encoder ticks (was 0.7)

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
    /** Shooter target (ticks/s) while USE_FIXED_SHOOTER_VELOCITY is on. */
    public static final double SHOOTER_FIXED_VELOCITY = 1500;
    /** Multiplies the regression output. 1.0 = the regression as-is; 1.5 = 50% faster. TUNE */
    public static final double SHOOTER_VELOCITY_SCALE = 3.0;
    /** Safety cap on the final target (ticks/s). Above the motor's real max it just means full power. */
    public static final double MAX_SHOOTER_VELOCITY = 2300;
    /** Telemetry says READY when the flywheel is within this many ticks/s of the target. */
    public static final double SHOOTER_READY_TOLERANCE = 50;

    // ================= PIDF =================
    // The hub's velocity PIDF holds the shooter at the target (PIDFShooter). F is used as-is, no voltage scaling.
    // Tune with "Shooter Graph Test".
    public static final double SHOOTER_P = 400, SHOOTER_I = 0, SHOOTER_D = 0, SHOOTER_F = 13.85; // 2026-11-21 (P / F / target found on Shooter Graph Test)

    /** Added to SHOOTER_FIXED_VELOCITY in every teleop / auto (0 = target stays SHOOTER_FIXED_VELOCITY). */
    public static final double PIDF_VELOCITY_OFFSET = 0;
    /** Teleop: intake AND transfer power when both run together (gamepad2 right bumper). */
    public static final double PIDF_INTAKE_TRANSFER_POWER = 1.0;
    /** Autos: intake AND transfer power while spinning up / feeding. */
    public static final double PIDF_AUTO_FEED_POWER = 1.0;
    /** Autos: the flywheel must stay READY this long before the first ball (longer so the wheel is fully at speed). */
    public static final double PIDF_AUTO_READY_HOLD_SEC = 0.5;
    /** Autos: shoot anyway after this long so a slow flywheel cannot eat the whole auto. */
    public static final double PIDF_AUTO_SPIN_UP_TIMEOUT_SEC = 4.0;

    // ================= BLOCKER =================
    public static final double BLOCKER_BLOCK = 0.49;
    public static final double BLOCKER_OPEN  = 0.68;

    // ================= FLOWER =================l
    public static final String FLOWER = "Flower1";
    /** Default position. */
    public static final double FLOWER_REST   = 0.32;
    public static final double FLOWER_ACTIVE = 0.59;
    /** Second flower servo. Name must match the robot config; positions are placeholders until tuned. */
    public static final String FLOWER2 = "Flower2";
    public static final double FLOWER2_REST   = 0.32;
    public static final double FLOWER2_ACTIVE = 0.59;

    // ================= INTAKE =================
    public static final double INTAKE_IN_POWER      = 1.0;
    public static final double INTAKE_REVERSE_POWER = -0.9;
    // (Intake + transfer power while shooting: PIDF_INTAKE_TRANSFER_POWER / PIDF_AUTO_FEED_POWER above.)

    // ================= TRANSFER =================
    public static final double TRANSFER_IN_POWER      = 1.0;  // TUNE
    public static final double TRANSFER_REVERSE_POWER = -0.9; // TUNE
}
