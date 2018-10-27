package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.Set;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Arm {

    static final double INTAKE_DRIVE_SPEED      = 0.1;
    static final double ARM_HOLD_VOLTS          = 0.05;
    static final double POS_TOLERANCE           = 0.05;
    static final double UNFOLD_POSITION_VOLTS   = 3.3;
    static final double PICKUP_POSITION_VOLTS   = 3.05;
    static final double DRIVE_POSITION_VOLTS    = 1.15;
    static final double RELEASE_POSITION_VOLTS  = 0.85;

    HardwarePlatter theHardwarePlatter;
    private double  driveSpeedSetPoint = 0.0;
    private boolean is_moving = false;
    private double  targetPosition;
    private double  armSpeed;

    public Arm(HardwarePlatter hwPlatter) {
        theHardwarePlatter = hwPlatter;
    }

    private void gotoPosition() {

        double error = targetPosition - theHardwarePlatter.armPotentiometer.getVoltage();

        if(Math.abs(error) > POS_TOLERANCE) {
            is_moving = true;
            if(error < 0) {
                armSpeed =INTAKE_DRIVE_SPEED;
                driveArm2();
            } else if(error > 0) {
                armSpeed = -INTAKE_DRIVE_SPEED;
                driveArm2();
            }
        } else {
            is_moving = false;
            stop();
        }
    }
    
    private void controlPosition(double deltaTime) {
/*        static final double Kp = 0.02, Ki = 0.001, Kd = 0;
        static double previousError = 0;
        static double integral = 0;
      
        double error = targetPosition - theHardwarePlatter.armPotentiometer.getVoltage();
        integral = integral + (error * deltaTime);
        double derivative = (error - previousError) / deltaTime;
        
        armSpeed =((Kp*Error) + (Ki*Integral) + (Kd*Derivative))
        driveArm2();
        
        Previous_Error = Error;*/
    }

    void unfold() {
        targetPosition = UNFOLD_POSITION_VOLTS;
        gotoPosition();
    }

    void pickUp() {
        targetPosition = PICKUP_POSITION_VOLTS;
        gotoPosition();
    }

    void drive() {
        targetPosition = DRIVE_POSITION_VOLTS;
        gotoPosition();
    }

    void release() {
        targetPosition = RELEASE_POSITION_VOLTS;
        gotoPosition();
    }

    void backward(double intakeDriveSpeed) {
        intakeDriveSpeed = Math.pow(intakeDriveSpeed, 3) * 0.3;
        driveSpeedSetPoint = intakeDriveSpeed;
        driveArm();
    }

    void forward(double intakeDriveSpeed) {
        intakeDriveSpeed = Math.pow(intakeDriveSpeed, 3) * 0.3;
        driveSpeedSetPoint = intakeDriveSpeed;
        driveArm();
    }
    private void driveArm2() {    // driveArm2 is used to move the arm to the set positions.
        theHardwarePlatter.armDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        theHardwarePlatter.armDrive.setPower(armSpeed);
    }
    private void driveArm() {
        theHardwarePlatter.armDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        theHardwarePlatter.armDrive.setPower(driveSpeedSetPoint);
    }
    void move(double speed){
        driveSpeedSetPoint = Math.pow(speed,3)*0.2;
        driveArm();
    }
    void stop() {
        theHardwarePlatter.armDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if( theHardwarePlatter.armPotentiometer.getVoltage() > DRIVE_POSITION_VOLTS) {
            theHardwarePlatter.armDrive.setPower(ARM_HOLD_VOLTS);
        } else {
            theHardwarePlatter.armDrive.setPower(-ARM_HOLD_VOLTS);
        }
    }

    void moveOrHoldPosition() {
        if(is_moving)
            gotoPosition();
        else
            stop();
    }
    
    boolean isMoving() {
        return(is_moving);
    }

    void display(Telemetry telemetry) {
        telemetry.addData("pot", theHardwarePlatter.armPotentiometer.getVoltage());
    }
}
