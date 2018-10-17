package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    HardwarePlatter theHardwarePlatter;

    public Intake(HardwarePlatter hwPlatter) {
        theHardwarePlatter = hwPlatter;
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
        double dumpServoPos = theHardwarePlatter.dumpServo.getPosition();
        double newDumpServoPos = dumpServoPos;
        newDumpServoPos += 0.2;
        theHardwarePlatter.dumpServo.setPosition(newDumpServoPos);

    }
    void closeDumpServo() {
        double dumpServoPos = theHardwarePlatter.dumpServo.getPosition();
        double newDumpServoPos = dumpServoPos;
        newDumpServoPos -= 0.2;
        theHardwarePlatter.dumpServo.setPosition(newDumpServoPos);
    }

}
