package org.firstinspires.ftc.teamcode.SumoRobot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "RGB Indicator Distance Sensor", group = "Sensor")
public class RGBIndicatorDistance extends OpMode {

    // Hardware
    private Servo rgbIndicator;
    private Rev2mDistanceSensor distanceSensor;

    // RGB Indicator positions (these control the color)
    private static final double RED = 0.277;;
    private static final double BLUE = 0.5;

    // Detection threshold in centimeters
    private static final double DETECTION_DISTANCE = 11.0; // Adjust this value as needed

    @Override
    public void init() {
        // Initialize hardware
        rgbIndicator = hardwareMap.get(Servo.class, "RGB");
        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distanceSensor");

        // Start with indicator off
        rgbIndicator.setPosition(RED);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Detection Distance", DETECTION_DISTANCE + " cm");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Get distance reading
        double distance = distanceSensor.getDistance(DistanceUnit.CM);

        // Check if something is detected within range
        if (distance <= DETECTION_DISTANCE) {
            // Object detected - turn green
            rgbIndicator.setPosition(BLUE);
        } else {
            // Nothing detected - turn off (or you could use RED)
            rgbIndicator.setPosition(RED);
        }

        // Display telemetry
        telemetry.addData("Distance (cm)", "%.2f", distance);
        telemetry.addData("Object Detected", distance < DETECTION_DISTANCE);
        telemetry.addData("Indicator Color", distance < DETECTION_DISTANCE ? "GREEN" : "OFF");
        telemetry.update();
    }
}