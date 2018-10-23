package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class ChassisAuton {
    //Rev Hex HD Motor 2240 counts per rotation
    static final double     COUNTS_PER_MOTOR_REV    = 2240 ;    // 20 - 537.6, 40 - 1120, 60 - 1680 (cpr) Gear Neverest Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.3;
    static final double     ANGLE_TOLERANCE         = 3;

    HardwarePlatter theHardwarePlatter;
    double gyroTarget;
    double headingResetValue;

    public ChassisAuton(HardwarePlatter hwPlatter) {
        theHardwarePlatter = hwPlatter;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        theHardwarePlatter.imu.initialize(parameters);

		theHardwarePlatter.leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        theHardwarePlatter.rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        theHardwarePlatter.leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        theHardwarePlatter.rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
    }

    void driveAuton(double distanceInches) {

        int newTargetPos;
        int distanceCounts = (int)(distanceInches * COUNTS_PER_INCH);

        // Determine new target position, and pass to motor controller
        for(DcMotor wheelDrive : theHardwarePlatter.wheelDrives) {
            wheelDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            newTargetPos = wheelDrive.getCurrentPosition() + distanceCounts;
            wheelDrive.setTargetPosition(newTargetPos);
            wheelDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        for(DcMotor wheelDrive : theHardwarePlatter.wheelDrives)
            wheelDrive.setPower(Math.abs(DRIVE_SPEED));
    }

    public void turn(double angledegrees) {
        gyroTarget = angledegrees;
        headingResetValue = getAbsoluteHeading();
        isGyroBusy();
    }

    private double getAbsoluteHeading() {
        return theHardwarePlatter.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    private double getRelativeHeading() {
        return this.getAbsoluteHeading() - this.headingResetValue;
    }

    boolean isGyroBusy() {
        double gyroActual = this.getRelativeHeading();
        double error = gyroTarget - gyroActual;
        boolean isBusy;
        if(Math.abs(error) > ANGLE_TOLERANCE) {
            if (error < 0)
                this.rotate(TURN_SPEED);
            else
                this.rotate(-TURN_SPEED);
            isBusy = true;
        } else {
            this.rotate(0.0);
            isBusy = false;
        }
        return(isBusy);
    }

    boolean isDriveBusy() {
        if(!theHardwarePlatter.leftFrontDrive.isBusy() || !theHardwarePlatter.rightFrontDrive.isBusy()) {
            rotate(0); /* stops all four wheels */
            return(false);
        } else {
            return(true);
        }
    }

    private void rotate(double power) {
        for(DcMotor wheelDrive : theHardwarePlatter.wheelDrives)
            wheelDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        theHardwarePlatter.leftFrontDrive.setPower(power);
        theHardwarePlatter.rightFrontDrive.setPower(-power);
        theHardwarePlatter.leftBackDrive.setPower(power);
        theHardwarePlatter.rightBackDrive.setPower(-power);
    }
}
