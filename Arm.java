package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Arm {

    static final double INTAKE_DRIVE_SPEED      = 0.1;
    static final double ARM_HOLD_VOLTS          = 0.001;
    static final double POS_TOLERANCE           = 0.1;
    static final double UNFOLD_POSITION_VOLTS   = 2.2;
    static final double PICKUP_POSITION_VOLTS   = 1.85;
    static final double DRIVE_POSITION_VOLTS    = 0.57;
    static final double RELEASE_POSITION_VOLTS  = 0.32;

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
            if(error < 0) {
                theHardwarePlatter.armDrive.setPower(-INTAKE_DRIVE_SPEED);
            } else if(error > 0) {
                theHardwarePlatter.armDrive.setPower(INTAKE_DRIVE_SPEED);
            }
        } else {
            is_moving = false;
        }
    }

    void unfold() {
        targetPosition = UNFOLD_POSITION_VOLTS;
    }

    void pickUp() {
        targetPosition = PICKUP_POSITION_VOLTS;
    }

    void drive() {
        targetPosition = DRIVE_POSITION_VOLTS;
    }

    void release() {
        targetPosition = RELEASE_POSITION_VOLTS;
    }

    void backward() {
        driveSpeedSetPoint = INTAKE_DRIVE_SPEED;
        driveArm();
    }

    void forward() {
        driveSpeedSetPoint = -INTAKE_DRIVE_SPEED;
        driveArm();
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

    void display(Telemetry telemetry) {
        telemetry.addData("pot", theHardwarePlatter.armPotentiometer.getVoltage());
    }
}


