package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Arm {

    static final double INTAKE_DRIVE_SPEED      = 0.1;
    static final double ARM_HOLD_VOLTS          = 0.001;
    static final double POS_TOLERANCE           = 0.1;
    static final double UNFOLD_POSITION_VOLTS   = 3.3;
    static final double PICKUP_POSITION_VOLTS   = 3.0;
    static final double DRIVE_POSITION_VOLTS    = 1.1;
    static final double RELEASE_POSITION_VOLTS  = 1.1;

    HardwarePlatter theHardwarePlatter;
    private double  driveSpeedSetPoint = 0.0;
    private boolean is_moving = false;
    private double  targetPosition;

    public Arm(HardwarePlatter hwPlatter) {
        theHardwarePlatter = hwPlatter;
    }

    private void gotoPosition() {

        double error = targetPosition - theHardwarePlatter.armPotentiometer.getVoltage();

        if(Math.abs(error) > POS_TOLERANCE) {
            is_moving = true;
            if(error < POS_TOLERANCE) {
                backward();
            } else {
                forward();
            }
        } else {
            is_moving = false;
        }
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

    void backward() {
        driveSpeedSetPoint = INTAKE_DRIVE_SPEED;
        driveArm();
    }

    void forward() {
        driveSpeedSetPoint = -INTAKE_DRIVE_SPEED;
        driveArm();
    }

    void move(double speed) {
        driveSpeedSetPoint = Math.pow(speed, 3) * 0.3;
        driveArm();
		is_moving = false;
    }

    private void driveArm() {
        theHardwarePlatter.armDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        theHardwarePlatter.armDrive.setPower(driveSpeedSetPoint);
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


