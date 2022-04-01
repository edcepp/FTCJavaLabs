// Autonomous program that drives bot in the shape of a square.
// Phase V05 - finishes first pass of turnForDegrees method.
//
// Ed C. Epp
// April 1, 2022

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name="Drive in a square v05", group="Concept")
//@Disabled

// --------------------------- DriveInASquare class -----------------------------
// ------------------------------------------------------------------------------
public  class DriveInASquare05 extends LinearOpMode
{
    //double TARGET_DISTANCE    = 300;         // mm (about 12 inches)
    double TARGET_DISTANCE    = 450;         // mm (about 18 inches)
    double MAX_MOTOR_VELOCITY = 600;         // mm / second
    double TARGET_VELOCITY    = MAX_MOTOR_VELOCITY / 4;
    double TARGET_TURN        =  90;         // degrees
    int    SQUARE_SIDES       =   4;

    DcMotorEx leftMotor;
    DcMotorEx rightMotor;

    @Override
    public void runOpMode() throws InterruptedException
    {
        // Configure the motors
        leftMotor = hardwareMap.get(DcMotorEx.class,"myLeftMotor");
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor = hardwareMap.get(DcMotorEx.class,"myRightMotor");
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // wait for start button.
        waitForStart();

        // wait while opmode is active and
        // left or right motor is busy running to position.
        int count = 0;
        while (opModeIsActive() && count < SQUARE_SIDES)
        {
            driveForMmAt (TARGET_DISTANCE, TARGET_VELOCITY);
            turnForDegrees (TARGET_TURN);
            count++;
        }


        // wait 5 sec to you can observe the final encoder position.
        Thread.sleep(5000);
    }

    // ----------- driveForMmAt ----------------------------------------
    // Drive for a specific distance and velocity
    //    distanceMm:   target distance in mm
    //    velocityMm:   target velocity in mm per second
    void driveForMmAt (double distanceMm, double velocityMmPerSec)
    {
        // REV-41-1300 Core Hex Motor
        // 4 Ticks per revolution at the motor
        // Gear ration: 72:1 motor revolutions per output revolution
        // 72 motor rev per output rev * 4 ticks per rev  => 288 ticks per output rev

        // REV-41-1354 90 mm traction wheel
        // 90 mm Wheel diameter
        // 90 mm * pi => 283 mm / rev

        double mMPerTick = 283.0 / 288.0;
        double ticksToMove = distanceMm / mMPerTick;
        // telemetry.addData("   mMPerTick  ticksToMove  ", mMPerTick + "  " + ticksToMove);

        double ticksPerSecond = velocityMmPerSec / mMPerTick;

        // reset the motors
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftMotor.setTargetPosition((int)ticksToMove);
        rightMotor.setTargetPosition((int)ticksToMove);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);    // forgot this
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setVelocity(ticksPerSecond);
        rightMotor.setVelocity(ticksPerSecond);

        // telemetry.addData("ticksPerSecond  ", ticksPerSecond);
        // telemetry.update();
        while (leftMotor.isBusy() || leftMotor.isBusy())
        {
            telemetry.addData("position  velocity ", leftMotor.getCurrentPosition() +
                                                    "   " + leftMotor.getVelocity());
            telemetry.update();
            idle();
        }

        // set motor power to zero to turn off motors. The motors stop on their own but
        // power is still applied so we turn off the power.
        leftMotor.setVelocity(0);
        rightMotor.setVelocity(0);
    }

    // ---------- turnForDegrees -----------------------------------
    // turnDegrees:  target turn in degrees
    // To be completed by the student
    void turnForDegrees (double turnDegrees)
    {
        // See driveForMmAt for additional drive train characteristics
        double turnDiameter = 208;    // distance between wheels in mm
        double wheelDistanceMm = (3.14156 * turnDiameter * turnDegrees) / 360;

        double mMPerTick = 283.0 / 288.0;
        double ticksToMove = wheelDistanceMm / mMPerTick;

        telemetry.addData("   wheelDistance  ticksToMove  ",
                wheelDistanceMm + " " + ticksToMove);
        telemetry.update();
        
        // reset the motors
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setTargetPosition((int)ticksToMove);
        rightMotor.setTargetPosition(-(int)ticksToMove);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setVelocity(TARGET_VELOCITY);
        rightMotor.setVelocity(-TARGET_VELOCITY);
        while (leftMotor.isBusy() || leftMotor.isBusy())
        {
            telemetry.addData("position  velocity ", leftMotor.getCurrentPosition() +
                    "   " + leftMotor.getVelocity());
            telemetry.update();
            idle();
        }

        // set motor power to zero to turn off motors. The motors stop on their own but
        // power is still applied so we turn off the power.
        leftMotor.setVelocity(0);
        rightMotor.setVelocity(0);
    }
}
