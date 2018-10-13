package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Intake {

    static final double UNFOLD_POSITION = 1;
    static final double PICKUP_POSITION = 1;
    static final double RELEASE_POSITION = 1;
    static final double TOLERENCE = 0.2;
    static final double SLIDEPID_KP = 0.01;

    HardwarePlatter theHardwarePlatter;

    public Intake(HardwarePlatter hwPlatter) {
        theHardwarePlatter = hwPlatter;
    }

    private void setPosition(double target) {

        double error = target - theHardwarePlatter.armPotentiometer.getVoltage();

        while (Math.abs(error) > TOLERENCE) {
            double power = SLIDEPID_KP*error;
            theHardwarePlatter.leftBackDrive.setPower(power);
        }
        theHardwarePlatter.leftBackDrive.setPower(0.0);
    }

    void unfold() {
        setPosition(UNFOLD_POSITION);
    }

    void pickUp() {
        setPosition(PICKUP_POSITION);
    }

    void release() {
        setPosition(RELEASE_POSITION);
    }
}



