package org.firstinspires.ftc.teamcode;

public class ElevatorClimb {
    static final double COUNTS_PER_MOTOR_REV = 1680;    // eg: 60 Gear Neverest Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 0.5;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    HardwarePlatter theHardwarePlatter;

    public ElevatorClimb(HardwarePlatter hwPlatter) {
        theHardwarePlatter = hwPlatter;
    }


    void climbUp() {                                    //elevator runs down
        theHardwarePlatter.elevatorDrive.setPower(1);
    }

    void dropDown() {                                   //elevator goes up
        theHardwarePlatter.elevatorDrive.setPower(-1);
    }

    void climberStop() {                                //stops elevator motor
        theHardwarePlatter.elevatorDrive.setPower(0);
    }

}
