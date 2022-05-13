// TensorFlowCanDo Version 11
// Autonomous program that combine TensorFlow object detection with CanDo. The goal is
// to use Machine Learning to
//     find find a gold cube,      
//     move straight to it,
//     knock it of the platform and
//     stop
// The code needs to be completed.
//
// This code is an integration of these two programs
//     https://github.com/edcepp/FTCJavaLabs/blob/main/CanDo06.java
//     https://github.com/edcepp/FTCEppTensorCode/blob/master/FTCEppTensorCode/TestTensorFlowObjectDetectionWebcamEpp.java
//
// Ed C. Epp
// May 13, 2022

package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.tensorflow.lite.task.vision.detector.Detection;

import java.util.List;
import java.util.stream.Stream;

@Autonomous(name="TensorFlow CanDo challenge v11", group="Challenge")
//@Disabled

// --------------------------- DriveInASquare class -----------------------------
// ------------------------------------------------------------------------------
public  class TensorFlowCanDo11 extends LinearOpMode
{
    int MAX_HITS              =   2;
    double TARGET_STOP_COLOR  = 150;   // Greenish card
    double HUE_TOLERANCE      =   5;   // degrees
    double TARGET_POWER       = 0.25;  // percent max
    int TARGET_TIME           = 1500;  // milliseconds
    int TURN_TIME             =  500;  // milliseconds
    double MIN_HSV_VALUE     =   6.0;

    DcMotorEx leftMotor;
    DcMotorEx rightMotor;
    ColorSensor rgbValues;

    /* Note: This sample uses the all-objects Tensor Flow model (FreightFrenzy_BCDM.tflite), which contains
     * the following 4 detectable objects
     *  0: Ball,
     *  1: Cube,
     *  2: Duck,
     *  3: Marker (duck location tape marker)
     *
     *  Two additional model assets are available which only contain a subset of the objects:
     *  FreightFrenzy_BC.tflite  0: Ball,  1: Cube
     *  FreightFrenzy_DM.tflite  0: Duck,  1: Marker
     */
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };

     // Once you've obtained a license key, copy the string from the Vuforia web site
     // and paste it in to your code on the next line, between the double quotes.
     // Please do not use mine.
     private static final String VUFORIA_KEY = "AY2Daiz/////AAABmYb00Vop7EAWqs/eRSieR19M5zxWECKfF05bE/xrCZXcvuMIT5zW88kcMPbUb2Bh/yA1O30q1tiOUQBj1TAXbCj4eRSLWWaYrxAYm+0Y1093z7T4uMbD0S+R/JXJAg/Siy8ALkMXiJWDA16H7GmOz1xqSb8v7R77hxcFP82xpmMk3kp4145aqeSzRI2UhyETgYqwAyQB8rtgbfRa0w+iG+A8F47Lwroq2g4PHgVZ5qHv6YDpz2Krw8StYEDoF1PtANTyNPWpGs9aABZakCBlXoZlzixwCoqZHpmS3RrkMyGRER+74aIDk2u+RJOf6DDDa5SHKdpCr24QVrV2W0AwoP6Fvpdm9rfTZ7nYYs7lk7ol";

     // {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     // localization engine.
    private VuforiaLocalizer vuforia;

     // {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     // Detection engine.
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() throws InterruptedException
    {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();
        telemetry.addData("Vuforia and TensorFlow", "Intialized");

        // Activate TensorFlow Object Detection before we wait for the start command.
        // Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
        if (tfod != null) {
            tfod.activate();
            telemetry.addData("TensorFlow", "Activated");

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            // feature disabled
            // tfod.setZoom(2.5, 16.0/9.0);
        }

        // Configure the motors
        leftMotor = hardwareMap.get(DcMotorEx.class,"myLeftMotor");
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor = hardwareMap.get(DcMotorEx.class,"myRightMotor");
        telemetry.addData("Motors", "Initialized");

        // Configure to read values from the physical sensor
        rgbValues = hardwareMap.get(ColorSensor.class, "sensor_color");
        telemetry.addData("Color Sensor", "Initialized");

        telemetry.addData ("Start", "Waiting press");
        telemetry.update();

        // wait for start button.
        waitForStart();

        // TBD
        // Start the robot spinning

        // Explore the list of recognized objects
        // TBD Stop when the object of interest is in the center of the screen
        while (opModeIsActive())
        {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null)
            {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions)
                {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                    i++;
                }
                telemetry.update();
            }
        }

        // Push the object off the platform
        driveUntilColor (TARGET_STOP_COLOR, TARGET_POWER);
    }

    // ----------- driveUntilColor ----------------------------------------
    // Drive for a specific distance and velocity
    //    targetHue:    Hue of color that will stop forward move
    //    velocityMm:   target velocity in mm per second
    void driveUntilColor (double targetHue, double percentPower) {
        // Convert from the RGB to the HSV color model
        int rChannel = rgbValues.red();
        int gChannel = rgbValues.green();
        int bChannel = rgbValues.blue();

        float[] hsvValues = new float[3];
        Color.RGBToHSV(rChannel, gChannel, bChannel, hsvValues);
        double measuredHue = hsvValues[0];
        double measuredSaturation = hsvValues[1];
        double measuredValue = hsvValues[2];

        // Power up the motors
        leftMotor.setPower(percentPower);
        rightMotor.setPower(percentPower);

        // Busy loop until the target color is detected
        int count = 1;

        while (measuredValue > MIN_HSV_VALUE) {
            telemetry.addData(
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
    }

    // ----------- driveUntilColor ----------------------------------------
    // Drive for a specific distance and velocity
    //    targetHue:    Hue of color that will stop forward move
    //    velocityMm:   target velocity in mm per second
    void driveUntilColorPref (double targetHue, double percentPower)
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
    /***************************** initVuforia ***************************
     * Initialize the Vuforia localization engine.
     ********************************************************************/
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /***************************** initTfod ******************************
     * Initialize the TensorFlow Object Detection engine.
     ********************************************************************/
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}
