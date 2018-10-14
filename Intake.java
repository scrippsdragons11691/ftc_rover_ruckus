package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake {

    static final double INTAKE_DRIVE_SPEED      = 0.05;
    static final double UNFOLD_POSITION         = 1;
    static final double PICKUP_POSITION         = 1;
    static final double RELEASE_POSITION        = 1;
    static final double TOLERENCE               = 0.2;
    static final double SLIDEPID_KP             = 0.01;
    static final double COUNTS_PER_MOTOR_REV    = 1120;    /* Nevrest 40 */
    static final double DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED             = 0.6;
    static final double TURN_SPEED              = 0.5;

    HardwarePlatter theHardwarePlatter;
    private double driveSpeedSetPoint = 0.0;

    public Intake(HardwarePlatter hwPlatter) {
        theHardwarePlatter = hwPlatter;
    }

    private void setPosition(double target) {

        double error = target - theHardwarePlatter.armPotentiometer.getVoltage();

        while (Math.abs(error) > TOLERENCE) {
            double power = SLIDEPID_KP*error;
            theHardwarePlatter.armDrive.setPower(power);
        }
        theHardwarePlatter.armDrive.setPower(0.0);
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

    void backward() {
        driveSpeedSetPoint = INTAKE_DRIVE_SPEED;
        driveIntake();
    }
    void forward() {
        driveSpeedSetPoint = -INTAKE_DRIVE_SPEED;
        driveIntake();
    }
    void driveIntakeOld() {
        if(!theHardwarePlatter.armDrive.isBusy()) {
            double deltaInches = 1;
            double deltaCounts = 133.75 * deltaInches;
            int newTarget = theHardwarePlatter.armDrive.getCurrentPosition() + (int) (deltaCounts);

            theHardwarePlatter.armDrive.setTargetPosition(newTarget);

            theHardwarePlatter.armDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            theHardwarePlatter.armDrive.setPower(driveSpeedSetPoint);
        }
    }

    void driveIntake() {
        theHardwarePlatter.armDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        theHardwarePlatter.armDrive.setPower(driveSpeedSetPoint);
    }

    void stop() {
        theHardwarePlatter.armDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveSpeedSetPoint = 0.001;
        theHardwarePlatter.armDrive.setPower(driveSpeedSetPoint);
    }
    
    void display(Telemetry telemetry) {
        telemetry.addData("pot", theHardwarePlatter.armPotentiometer.getVoltage());
        
    }

}



