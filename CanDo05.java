/ CanDo Beginning of a CanDo type of program
// Autonomous program that test the driveUntilColor and justDriveForATime methods.
// Include saturation and value to experiment with black and white surfaces
//
// Phase V05 - Stops moving forward when HSV Value drops too low. Not a good test.
//
// Ed C. Epp
// April 28, 2022

package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name="CanDo challenge v05", group="Challenge")
//@Disabled

// --------------------------- DriveInASquare class -----------------------------
// ------------------------------------------------------------------------------
public  class CanDo05 extends LinearOpMode
{
    int MAX_HITS              =   2;
    double TARGET_STOP_COLOR  = 150;   // Greenish card
    double HUE_TOLERANCE      =   5;   // degrees
    double TARGET_POWER       = 0.50;  // percent max
    int TARGET_TIME           = 3000;  // milliseconds
    double MIN_HSV_VALUE     =   6.0;

    DcMotorEx leftMotor;
    DcMotorEx rightMotor;
    ColorSensor rgbValues;

    @Override
    public void runOpMode() throws InterruptedException
    {
        // Configure the motors
        leftMotor = hardwareMap.get(DcMotorEx.class,"myLeftMotor");
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor = hardwareMap.get(DcMotorEx.class,"myRightMotor");

        // Configure to read values from the physical sensor
        rgbValues = hardwareMap.get(ColorSensor.class, "sensor_color");

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // wait for start button.
        waitForStart();

        // wait while opmode is active and loop only once for testing
        int hits = 0;
        while (opModeIsActive() && hits < MAX_HITS)
        {
            driveUntilColor (TARGET_STOP_COLOR, TARGET_POWER);
            justDriveForATime (-TARGET_POWER, -TARGET_POWER, TARGET_TIME);
            //displayColor();
            hits++;
        }

        // wait 5 sec to you can observe the final encoder position.
        Thread.sleep(5000);
    }

    // ----------- driveUntilColor ----------------------------------------
    // Drive for a specific distance and velocity
    //    targetHue:    Hue of color that will stop forward move
    //    velocityMm:   target velocity in mm per second
    void driveUntilColor (double targetHue, double percentPower)
    {
        // Convert from the RGB to the HSV color model
        int rChannel = rgbValues.red();
        int gChannel = rgbValues.green();
        int bChannel = rgbValues.blue();

        float[] hsvValues = new float[3];
        Color.RGBToHSV(rChannel, gChannel, bChannel, hsvValues);
        double measuredHue        = hsvValues[0];
        double measuredSaturation = hsvValues[1];
        double measuredValue      = hsvValues[2];

        // Power up the motors
        leftMotor.setPower(percentPower);
        rightMotor.setPower(percentPower);

         // Busy loop until the target color is detected
        int count = 1;

        while (measuredValue > MIN_HSV_VALUE)
        {
            telemetry.addData (
                    String.format("%d: ", count++),
                    String.format("minimum HSV Value: %.1f    ", MIN_HSV_VALUE) +
                            String.format("measured HSV Value: %.1f", measuredValue));
            telemetry.update();

            // Convert RGB to HSV
            rChannel = rgbValues.red();
            gChannel = rgbValues.green();
            bChannel = rgbValues.blue();
            Color.RGBToHSV(rChannel, gChannel, bChannel, hsvValues);

            // Extract the HSV HUE
            measuredValue = hsvValues[2];
        }

        // set motor power to zero to turn off motors. The motors stop on their own but
        // power is still applied so we turn off the power.
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    // ----------- justDriveForATime ----------------------------------------
    // Drive for a time with motors powered as follows
    //    leftMotorPower:    percent of max
    //    rightMotorPoser:   percent of max
    //    targetTime:        milliseconds that the motors are powered
    void justDriveForATime (double leftMotorPower, double rightMotorPower, int targetTime)
            throws InterruptedException
    {
        // Power up the motors
        leftMotor.setPower(leftMotorPower);
        rightMotor.setPower(rightMotorPower);

        // after targetTime milliseconds turn off the motors
        Thread.sleep(targetTime);
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    // ----------- displayColor ----------------------------------------
    // Take over control and display the
    //              RGB color value received by the color sensor and
    //              the corresponding HSV value
    void displayColor ()
    {
        while (true)
        {
            // Convert from the RGB to the HSV color model
            int rChannel = rgbValues.red();
            int gChannel = rgbValues.green();
            int bChannel = rgbValues.blue();

            float[] hsvValues = new float[3];
            Color.RGBToHSV(rChannel, gChannel, bChannel, hsvValues);

            telemetry.addData (
                    String.format("red %d: ", rChannel),
                    String.format("green: %d    ", gChannel) +
                            String.format("blue: %d", bChannel));
            telemetry.addData (
                    String.format("hue %.1f: ", hsvValues[0]),
                    String.format("saturatio: %.2f    ", hsvValues[1]) +
                            String.format("value: %.2f", hsvValues[2]));
            telemetry.update();
        }
    }
}
