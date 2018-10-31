/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 * <p>
 * The code REQUIRES that you DO have encoders on the wheels,
 * otherwise you would use: PushbotAutoDriveByTime;
 * <p>
 * This code ALSO requires that the drive Motors have been configured such that a positive
 * power command moves them forwards, and causes the encoders to count UP.
 * <p>
 * The desired path in this example is:
 * - Drive forward for 48 inches
 * - Spin right for 12 Inches
 * - Drive Backwards for 24 inches
 * - Stop and close the claw.
 * <p>
 * The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 * that performs the actual movement.
 * This methods assumes that each movement is relative to the last stopping place.
 * There are other ways to perform encoder based moves, but this method is probably the simplest.
 * This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */


public class ChassisAuton1 {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    HardwarePlatter robot;

    static final double COUNTS_PER_MOTOR_REV = 280;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    public ChassisAuton1(HardwarePlatter hwPlatter) {
        robot = hwPlatter;


        robot.leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        robot.rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        robot.leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        robot.rightBackDrive.setDirection(DcMotor.Direction.REVERSE);


        // Send telemetry message to signify robot waiting;
        //telemetry.addData("Status", "Resetting Encoders");    //
        //telemetry.update();

        robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        //telemetry.addData("Path0",  "Starting at %7d :%7d",
        //                  robot.leftFrontDrive.getCurrentPosition(),
        //                  robot.rightFrontDrive.getCurrentPosition());
        //robot.leftBackDrive.getCurrentPosition(),
        //robot.rightBackDrive.getCurrentPosition());

        //telemetry.update();
    }

    public void driveAuton(float distanceInches) {

        double speed = 1;
        double leftInches = distanceInches;
        double rightInches = distanceInches;
        double timeoutS;
        int newLeftTarget;
        int newRightTarget;

        // Determine new target position, and pass to motor controller
        newLeftTarget = robot.leftFrontDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
        newRightTarget = robot.rightFrontDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
        newLeftTarget = robot.leftBackDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
        newRightTarget = robot.rightBackDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

        robot.leftFrontDrive.setTargetPosition(newLeftTarget);
        robot.rightFrontDrive.setTargetPosition(newRightTarget);
        robot.leftBackDrive.setTargetPosition(newLeftTarget);
        robot.rightBackDrive.setTargetPosition(newRightTarget);

        // Turn On RUN_TO_POSITION
        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        robot.leftFrontDrive.setPower(Math.abs(speed));
        robot.rightFrontDrive.setPower(Math.abs(speed));
        robot.leftBackDrive.setPower(Math.abs(speed));
        robot.rightBackDrive.setPower(Math.abs(speed));

        while (robot.leftFrontDrive.isBusy() && robot.rightFrontDrive.isBusy()) {
            //telemetry.addLine("Encoders_run");
            //telemetry.update();
        }

    }

    public boolean isBusy() {
        if (!robot.leftFrontDrive.isBusy() || !robot.rightFrontDrive.isBusy()) {
            // Stop all motion;
            robot.leftFrontDrive.setPower(0);
            robot.rightFrontDrive.setPower(0);
            robot.leftFrontDrive.setPower(0);
            robot.rightFrontDrive.setPower(0);
            // Turn off RUN_TO_POSITION
            //robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            return (false);
        } else {
            return (true);
        }
    }
}
