package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ChassisAuton {
    //Rev Hex HD Motor 2240 counts per rotation
    static final double COUNTS_PER_MOTOR_REV = 2240;    // 20 - 537.6, 40 - 1120, 60 - 1680 (cpr) Gear Neverest Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    HardwarePlatter theHardwarePlatter;
    double powerSetpoint = 0.1;
    double error;
    private ElapsedTime runtime = new ElapsedTime();

    public ChassisAuton(HardwarePlatter hwPlatter) {
        theHardwarePlatter = hwPlatter;

        theHardwarePlatter.leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        theHardwarePlatter.rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        theHardwarePlatter.leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        theHardwarePlatter.rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

    }

    void driveAuton(double distanceInches, double speed) {

        //double speed = 1;
        double leftInches = distanceInches;
        double rightInches = distanceInches;
        double timeoutS;
        int newLeftFTarget;
        int newRightFTarget;
        int newLeftBTarget;
        int newRightBTarget;


        theHardwarePlatter.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        theHardwarePlatter.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        theHardwarePlatter.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        theHardwarePlatter.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        theHardwarePlatter.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        theHardwarePlatter.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        theHardwarePlatter.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        theHardwarePlatter.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Determine new target position, and pass to motor controller
        newLeftFTarget = theHardwarePlatter.leftFrontDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
        newRightFTarget = theHardwarePlatter.rightFrontDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
        newLeftBTarget = theHardwarePlatter.leftBackDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
        newRightBTarget = theHardwarePlatter.rightBackDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

        theHardwarePlatter.leftFrontDrive.setTargetPosition(newLeftFTarget);
        theHardwarePlatter.rightFrontDrive.setTargetPosition(newRightFTarget);
        theHardwarePlatter.leftBackDrive.setTargetPosition(newLeftBTarget);
        theHardwarePlatter.rightBackDrive.setTargetPosition(newRightBTarget);


        // Turn On RUN_TO_POSITION
        theHardwarePlatter.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        theHardwarePlatter.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        theHardwarePlatter.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        theHardwarePlatter.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // reset the timeout time and start motion.
        //runtime.reset();
        theHardwarePlatter.leftFrontDrive.setPower(Math.abs(speed));
        theHardwarePlatter.rightFrontDrive.setPower(Math.abs(speed));
        theHardwarePlatter.leftBackDrive.setPower(Math.abs(speed));
        theHardwarePlatter.rightBackDrive.setPower(Math.abs(speed));

        while (theHardwarePlatter.leftBackDrive.isBusy() && theHardwarePlatter.rightBackDrive.isBusy()) {
            //telemetry.addLine("Encoders_run");
            //telemetry.update();
        }

    }

    void rotateAuton(double distanceInches, double speed) {

        double leftInches = distanceInches;


        theHardwarePlatter.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        theHardwarePlatter.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        theHardwarePlatter.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        theHardwarePlatter.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Determine new target position, and pass to motor controller

        theHardwarePlatter.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        theHardwarePlatter.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        theHardwarePlatter.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        theHardwarePlatter.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int newLeftBTarget = theHardwarePlatter.leftBackDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);

        theHardwarePlatter.leftBackDrive.setTargetPosition(newLeftBTarget);
        theHardwarePlatter.rightBackDrive.setTargetPosition(-newLeftBTarget);
        theHardwarePlatter.leftFrontDrive.setTargetPosition(newLeftBTarget);
        theHardwarePlatter.rightFrontDrive.setTargetPosition(-newLeftBTarget);

        theHardwarePlatter.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        theHardwarePlatter.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        theHardwarePlatter.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        theHardwarePlatter.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        theHardwarePlatter.leftBackDrive.setPower(Math.abs(speed));
        theHardwarePlatter.rightBackDrive.setPower(Math.abs(speed));
        theHardwarePlatter.leftFrontDrive.setPower(Math.abs(speed));
        theHardwarePlatter.rightFrontDrive.setPower(Math.abs(speed));

        while (theHardwarePlatter.leftBackDrive.isBusy()) {
        }
        theHardwarePlatter.leftBackDrive.setPower(0);
        theHardwarePlatter.rightBackDrive.setPower(0);
        theHardwarePlatter.leftFrontDrive.setPower(0);
        theHardwarePlatter.rightFrontDrive.setPower(0);

    }

    public boolean isDriveBusy() {
        if (!theHardwarePlatter.leftBackDrive.isBusy() || !theHardwarePlatter.rightBackDrive.isBusy()) {
            // Stop all motion;
            theHardwarePlatter.leftFrontDrive.setPower(0);
            theHardwarePlatter.rightFrontDrive.setPower(0);
            // Turn off RUN_TO_POSITION
            theHardwarePlatter.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            theHardwarePlatter.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            return (false);
        } else {
            return (true);
        }
    }

    void rotate(double power) {
        theHardwarePlatter.leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        theHardwarePlatter.rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        theHardwarePlatter.leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        theHardwarePlatter.rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        theHardwarePlatter.leftFrontDrive.setPower(power);
        theHardwarePlatter.rightFrontDrive.setPower(-power);
        theHardwarePlatter.leftBackDrive.setPower(power);
        theHardwarePlatter.rightBackDrive.setPower(-power);
    }

    void display(Telemetry telemetry) {

    }

    void stop() {
        rotate(0);
    }
}
