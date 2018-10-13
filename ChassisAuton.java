package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class ChassisAuton {
    static final double     COUNTS_PER_MOTOR_REV    = 280 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    private ElapsedTime runtime = new ElapsedTime();
    HardwarePlatter theHardwarePlatter;
    double gyroTarget;
    double headingResetValue;

    public ChassisAuton(HardwarePlatter hwPlatter) {
        theHardwarePlatter = hwPlatter;
    }

    void driveAuton(double distanceInches) {

        double speed = 1;
        double leftInches = distanceInches;
        double rightInches = distanceInches;
        double timeoutS;
        int newLeftTarget;
        int newRightTarget;

        // Determine new target position, and pass to motor controller
        newLeftTarget = theHardwarePlatter.leftFrontDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
        newRightTarget = theHardwarePlatter.rightFrontDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

        theHardwarePlatter.leftFrontDrive.setTargetPosition(newLeftTarget);
        theHardwarePlatter.rightFrontDrive.setTargetPosition(newRightTarget);

        // Turn On RUN_TO_POSITION
        theHardwarePlatter.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        theHardwarePlatter.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // reset the timeout time and start motion.
        runtime.reset();
        theHardwarePlatter.leftFrontDrive.setPower(Math.abs(speed));
        theHardwarePlatter.rightFrontDrive.setPower(Math.abs(speed));
    }

    private double getAbsoluteHeading() {
        return theHardwarePlatter.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    private double getRelativeHeading() {
        return this.getAbsoluteHeading() - this.headingResetValue;
    }

    boolean isGyroBusy() {
        double gyroActual = this.getRelativeHeading();
        if(gyroActual < gyroTarget) {
            this.turn(0.1);
            return true;
        } else if(gyroActual > gyroTarget) {
            this.turn(-0.1);
            return true;
        } else {
            this.turn(0.0);
            return false;
        }
    }
    public boolean isDriveBusy() {
        if(!theHardwarePlatter.leftFrontDrive.isBusy() || !theHardwarePlatter.rightFrontDrive.isBusy()) {
            // Stop all motion;
            theHardwarePlatter.leftFrontDrive.setPower(0);
            theHardwarePlatter.rightFrontDrive.setPower(0);
            // Turn off RUN_TO_POSITION
            theHardwarePlatter.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            theHardwarePlatter.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            return(false);
        } else {
            return(true);
        }
    }

    public void turn(double angledegrees) {
        gyroTarget = angledegrees;
        headingResetValue = getAbsoluteHeading();
    }
    void rotate(double power) {
        theHardwarePlatter.leftFrontDrive.setPower(power);
        theHardwarePlatter.rightFrontDrive.setPower(-power);
        theHardwarePlatter.leftBackDrive.setPower(power);
        theHardwarePlatter.rightBackDrive.setPower(-power);
    }
}
