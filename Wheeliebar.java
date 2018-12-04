package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Wheeliebar {

    static final double WHEELIEBAR_DRIVE_SPEED = 0.5;
    static final double WHEELIEBAR_HOLD_VOLTS = 0.1;
    static final double WHEELIEBAR_HOLD_VOLTSDN = 0.25;
    static final double POS_TOLERANCE = 0.05;
    static final double DOWN_POSITION_VOLTS = 2.0;
    static final double UP_POSITION_VOLTS = 0.5;
    static final double MIDDLE_POSITION_VOLTS = 1.25;

    HardwarePlatter theHardwarePlatter;
    private double driveSpeedSetPoint = 0.5;
    private boolean is_moving = false;
    private double targetPosition;

    public Wheeliebar(HardwarePlatter hwPlatter) {
        theHardwarePlatter = hwPlatter;
    }

    private void gotoPosition() {

        double error = targetPosition - theHardwarePlatter.wheeliebarPot.getVoltage();

        if (Math.abs(error) > POS_TOLERANCE) {
            is_moving = true;
            if (error < 0) {
                driveSpeedSetPoint = -WHEELIEBAR_DRIVE_SPEED; //up
                driveWheeliebar2();
            } else if (error > 0) {
                driveSpeedSetPoint = WHEELIEBAR_DRIVE_SPEED; //down
                driveWheeliebar2();
            }
        } else {
            is_moving = false;
            stop();
        }
    }

    void up() {
        targetPosition = UP_POSITION_VOLTS;
        gotoPosition();
    }

    void down() {
        targetPosition = DOWN_POSITION_VOLTS;
        gotoPosition();
    }

    void middle() {
        targetPosition = MIDDLE_POSITION_VOLTS;
        gotoPosition();
    }

    private void driveWheeliebar() {
        theHardwarePlatter.wheelieBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        theHardwarePlatter.wheelieBar.setPower(driveSpeedSetPoint);
        is_moving = false;
    }

    private void driveWheeliebar2() {
        theHardwarePlatter.wheelieBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        theHardwarePlatter.wheelieBar.setPower(driveSpeedSetPoint);
    }

    void move(double speed) {
        driveSpeedSetPoint = Math.pow(speed, 3) * 0.5;  //was 0.2
        driveWheeliebar();
    }

    void stop() {
        theHardwarePlatter.wheelieBar.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        is_moving = false;
        if (theHardwarePlatter.wheeliebarPot.getVoltage() > DOWN_POSITION_VOLTS * .9)
            theHardwarePlatter.wheelieBar.setPower(WHEELIEBAR_HOLD_VOLTSDN);
        else theHardwarePlatter.wheelieBar.setPower(-WHEELIEBAR_HOLD_VOLTS);
    }

    boolean moveOrHoldPosition() {
        if (is_moving)
            gotoPosition();
        else
            stop();
        return (is_moving);
    }

    boolean isMoving() {
        return (is_moving);
    }

    void display(Telemetry telemetry) {
        telemetry.addData("Wheeliebar pot", theHardwarePlatter.wheeliebarPot.getVoltage());
        telemetry.addData("Wheeliebar mtr", theHardwarePlatter.wheelieBar.getPower());
        telemetry.addData("Wheeliebar isMoving", isMoving());

    }
}
