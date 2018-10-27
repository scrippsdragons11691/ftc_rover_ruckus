package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    HardwarePlatter theHardwarePlatter;

    public Intake(HardwarePlatter hwPlatter) {
        theHardwarePlatter = hwPlatter;
        
        theHardwarePlatter.dumpServo.setPosition(0.4);
    }

    public void driveCombine(boolean forward, boolean backward) {
        if(forward) {
            theHardwarePlatter.combineDrive.setPower(1.0);
        } else if(backward) {
            theHardwarePlatter.combineDrive.setPower(-1.0);
        } else {
            theHardwarePlatter.combineDrive.setPower(0);
        }
    }
    
    void openDumpServo() {
        theHardwarePlatter.dumpServo.setPosition(0);

    }
    
    void closeDumpServo() {
        theHardwarePlatter.dumpServo.setPosition(0.4);
    }

}
