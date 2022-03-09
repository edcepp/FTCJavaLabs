// Dr. Edward C. Epp
// Oregon IEEE RAS
// March 8, 2022
//
// Display RGB and HSV model colors read from a color sensor.
//
// based on https://docs.revrobotics.com/color-sensor/application-examples
// with help from https://stackoverflow.com/questions/28893613/convert-rgb-to-hsv-in-android

package org.firstinspires.ftc.teamcode;

         import android.graphics.Color;
         import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
         import com.qualcomm.robotcore.eventloop.opmode.Disabled;
         import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

         import android.graphics.Color;
         import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous(name = "Epp Test Color Sensor v01", group = "Concept")
// @Disabled

public class TestColorSensorEpp extends LinearOpMode
{
    @Override
    public void runOpMode()
    {
        // Configure to read values from the physical sensor
        ColorSensor rgbValues = hardwareMap.get(ColorSensor.class, "sensor_color");

        // initialize the robot and wait for the user to press start
        telemetry.addData(">", " To start press play");
        telemetry.update();
        waitForStart();

        final float[] hsvValues = new float[3];

        // While the Op Mode is running, display the detected color values.
        while (opModeIsActive())
        {
            // Convert from the RGB to the HSV color model
            int r = rgbValues.red();
            int g = rgbValues.green();
            int b = rgbValues.blue();
            Color.RGBToHSV(r, g, b, hsvValues);

            // Display the RGB and HSV color model components on the Driver Hub
            telemetry.addLine()
                    .addData("Red", rgbValues.red())
                    .addData("Green", rgbValues.green())
                    .addData("Blue", rgbValues.blue());
            telemetry.addLine()
                    .addData("Hue", "%.3f", hsvValues[0])
                    .addData("Saturation", "%.3f", hsvValues[1])
                    .addData("Value", "%.3f", hsvValues[2]);
             telemetry.update();
        }
    }
}
