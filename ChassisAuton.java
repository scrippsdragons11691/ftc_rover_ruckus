package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    static final double     TURN_SPEED              = 0.5;

    private ElapsedTime runtime = new ElapsedTime();
    HardwarePlatter theHardwarePlatter;
    double gyroTarget;
    double headingResetValue;
    boolean isTurning = false;
    double powerSetpoint = 0.1;
    double error;

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
        newLeftFTarget = theHardwarePlatter.leftFrontDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
        newRightFTarget = theHardwarePlatter.rightFrontDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
        newLeftBTarget = theHardwarePlatter.leftBackDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
        newRightBTarget = theHardwarePlatter.rightBackDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
        
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
        
         while(theHardwarePlatter.leftBackDrive.isBusy() && theHardwarePlatter.rightBackDrive.isBusy())
            {
            //telemetry.addLine("Encoders_run");
            //telemetry.update();
            }
        
    }

    private double getAbsoluteHeading() {
        return theHardwarePlatter.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    private double getRelativeHeading() {
        return this.getAbsoluteHeading() - this.headingResetValue;
    }

    boolean isGyroBusy() {
        if(isTurning) {
            error = gyroTarget - getAbsoluteHeading();
            
            if(Math.abs(error) > 1.0) {
                if(error > 0) {
                    rotate(-powerSetpoint);
                } else {
                    rotate(+powerSetpoint);
                }
            } else {
                isTurning = false;
                rotate(0);
            }
        }
        return(isTurning);
    }
    public boolean isDriveBusy() {
        if(!theHardwarePlatter.leftBackDrive.isBusy() || !theHardwarePlatter.rightBackDrive.isBusy()) {
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

    public void turn(double angledegrees, double power) {
        gyroTarget = angledegrees + getAbsoluteHeading();
        isTurning = true;
        powerSetpoint = power;
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
    
    boolean isTurning() {
        return(isTurning);
    }
    
    void display(Telemetry telemetry) {
        telemetry.addData("gyro rel heading - actual", getRelativeHeading());
        telemetry.addData("gyro target", gyroTarget);
        telemetry.addData("gyro abs heading", getAbsoluteHeading());
        telemetry.addData("gyro error", error);
    }
}
