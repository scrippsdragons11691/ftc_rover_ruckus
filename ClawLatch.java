package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ClawLatch {
    HardwarePlatter theHardwarePlatter;
    static final double DELTA_POSITION = 0.1;

    ClawLatch(HardwarePlatter hwPlatter) {
        theHardwarePlatter =  hwPlatter;
        theHardwarePlatter.clawServo.setPosition(0);
    }

    void open() {
        double newPosition;
        newPosition = theHardwarePlatter.clawServo.getPosition() + DELTA_POSITION;
        if(newPosition > 1.0) {
            newPosition = 1.0;
        }
        theHardwarePlatter.clawServo.setPosition(newPosition);
    }

    void close() {
        double newPosition;
        newPosition = theHardwarePlatter.clawServo.getPosition() - DELTA_POSITION;
        if(newPosition < 0) {
            newPosition = 0;
        }
        theHardwarePlatter.clawServo.setPosition(newPosition);
    }
    void display(Telemetry telemetry) {
        telemetry.addData("clawsevo pos", theHardwarePlatter.clawServo.getPosition());
    }
}
