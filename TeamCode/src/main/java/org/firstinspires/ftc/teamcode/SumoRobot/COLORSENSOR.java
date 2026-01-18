package org.firstinspires.ftc.teamcode.SumoRobot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class COLORSENSOR extends OpMode {

    private ColorSensor CS;
    private Servo RGB;

    private static final int GREEN_THRESHOLD = 150;
    private static final int PURPLE_RED_THRESHOLD = 100;
    private static final int PURPLE_BLUE_THRESHOLD = 100;

    // goBILDA RGB LED positions
    private static final double OFF = 0.0;
    private static final double GREEN = 0.37;
    private static final double VIOLET = 0.87;

    private long lastFlashTime = 0;
    private boolean flashOn = false;
    private static final long FLASH_INTERVAL = 500;

    @Override
    public void init() {
        CS = hardwareMap.get(ColorSensor.class, "ColorSensor");
        RGB = hardwareMap.get(Servo.class, "RGB");
        CS.enableLed(true);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        int red = CS.red();
        int green = CS.green();
        int blue = CS.blue();

        // Display RGB values
        telemetry.addData("Red", red);
        telemetry.addData("Green", green);
        telemetry.addData("Blue", blue);

        // Toggle flash
        long currentTime = System.currentTimeMillis();
        if (currentTime - lastFlashTime > FLASH_INTERVAL) {
            flashOn = !flashOn;
            lastFlashTime = currentTime;
        }

        // Detect green and flash green
        if (green > GREEN_THRESHOLD && green > red && green > blue) {
            RGB.setPosition(flashOn ? GREEN : OFF);
            telemetry.addData("Color Detected", "GREEN");
        }
        // Detect purple/violet and flash purple
        else if (red > PURPLE_RED_THRESHOLD && blue > PURPLE_BLUE_THRESHOLD &&
                red > green && blue > green) {
            RGB.setPosition(flashOn ? VIOLET : OFF);
            telemetry.addData("Color Detected", "PURPLE");
        }
        else {
            RGB.setPosition(OFF);
            telemetry.addData("Color Detected", "None");
        }

        telemetry.update();
    }
}