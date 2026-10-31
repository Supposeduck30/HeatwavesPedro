package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;

/**
 * Pedro Pathing 3.x AutoTune procedures. Deploy the code, connect to the Robot Controller and open
 * http://192.168.43.1:10158 to pick one. Every {@link Tuner} method must be public, static,
 * take no arguments and return a {@link Procedure}.
 *
 * Tune in this order: Mecanum -> Pinpoint -> Foresight -> Tests, pasting the generated config
 * into {@link Constants} after each step.
 *
 * AutoTune is currently excluded from the build (see TeamCode/build.gradle), because it needs
 * the com.pedropathing:tuning dependency, which brings in Sloth.
 */
public class Tuning {

    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }

    @Tuner
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }

    @Tuner
    public static Procedure foresightTuner() {
        return new ForesightTuner(
                (hardwareMap) -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
                (hardwareMap) -> new Mecanum(hardwareMap, Constants.drivetrainConfig)
        );
    }

    @Tuner
    public static Procedure tests() {
        return new Tests(
                (hardwareMap) -> new Mecanum(hardwareMap, Constants.drivetrainConfig),
                (hardwareMap) -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
                () -> new Foresight(Constants.foresightConfig)
        );
    }
}
