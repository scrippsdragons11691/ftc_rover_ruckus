package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ClawLatch {
    static final double DELTA_POSITION = 0.1;
    static final double OPENPOSITION = 1;
    static final double CLOSEPOSITION = 0;
    HardwarePlatter theHardwarePlatter;

    ClawLatch(HardwarePlatter hwPlatter) {
        theHardwarePlatter = hwPlatter;
        theHardwarePlatter.clawServo.setPosition(1);
    }

    void open() {
        double newPosition;
        newPosition = theHardwarePlatter.clawServo.getPosition() + DELTA_POSITION;
        if (newPosition > 1.0) {
            newPosition = 1.0;
        }
        theHardwarePlatter.clawServo.setPosition(newPosition);
    }

    void close() {
        double newPosition;
        newPosition = theHardwarePlatter.clawServo.getPosition() - DELTA_POSITION;
        if (newPosition < 0) {
            newPosition = 0;
        }
        theHardwarePlatter.clawServo.setPosition(newPosition);
    }

    void closeAuton() {

        theHardwarePlatter.clawServo.setPosition(0);
    }

    void display(Telemetry telemetry) {
        telemetry.addData("clawsevo pos", theHardwarePlatter.clawServo.getPosition());
    }

    void autoClose() {
        theHardwarePlatter.clawServo.setPosition(CLOSEPOSITION);
    }

    void autoOpen() {
        theHardwarePlatter.clawServo.setPosition(OPENPOSITION);
    }
}
