
package org.firstinspires.ftc.teamcode;


public class ElevatorClimb {
    
    HardwarePlatter theHardwarePlatter;
    
    public ElevatorClimb (HardwarePlatter hwPlatter){
            theHardwarePlatter = hwPlatter;
    }
    
    void climbUp() {
        theHardwarePlatter.elevatorDrive.setPower(1);
    }
    void dropDown() {
        theHardwarePlatter.elevatorDrive.setPower(-1);
    }
}
