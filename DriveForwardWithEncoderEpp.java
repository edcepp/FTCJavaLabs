// autonomous program that drives bot forward a set distance, stops then
// backs up to the starting point using encoders to measure the distance.
// This example assumes there is one encoder, attached to the left motor.
// http://stemrobotics.cs.pdx.edu/node/4746
//
// Simplified version
// Ed C. Epp
// March 1, 2022

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Epp Drive Forward Encoder v02", group="Concept")
//@Disabled
public  class DriveForwardWithEncoderEpp extends LinearOpMode
{
    int    MOTOR_COUNT = -1000;
    double MOTOR_POWER =  0.50;

    DcMotor leftMotor;
    DcMotor rightMotor;

    @Override
    public void runOpMode() throws InterruptedException {

        // Configure left motor control mover forward until encoder count is reached
        leftMotor = hardwareMap.dcMotor.get("myLeftMotor");
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setTargetPosition(MOTOR_COUNT);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // configure right motor to follow the left motor's lead
        rightMotor = hardwareMap.dcMotor.get("myRightMotor");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // wait for start button.
        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        // set both motors to 25% power. Movement will start.
        leftMotor.setPower(-0.25);
        rightMotor.setPower(-0.25);

        // wait while opmode is active and left motor is busy running to position.
        while (opModeIsActive() && leftMotor.isBusy()) {
            telemetry.addData("encoder-fwd", leftMotor.getCurrentPosition() + "  busy=" + leftMotor.isBusy());
            telemetry.update();
            idle();
        }

        // set motor power to zero to turn off motors. The motors stop on their own but
        // power is still applied so we turn off the power.
        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);

        // wait 5 sec to you can observe the final encoder position.
        Thread.sleep(5000);
    }
}
