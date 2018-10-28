package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class AutonDepot extends OpMode {
    HardwarePlatter theHardwarePlatter;
    ChassisAuton theChassis;
    Arm theArm;
    Intake theIntake;
    ElevatorClimb theElevatorClimb;
    ClawLatch clawLatch;
    Wheeliebar wheeliebar;
    double timeout = 0;
    int step = 4;
    ElapsedTime runtime;

    public void init() {
        theHardwarePlatter = new HardwarePlatter(hardwareMap);
        theElevatorClimb = new ElevatorClimb(theHardwarePlatter);
        theChassis = new ChassisAuton(theHardwarePlatter);
        clawLatch = new ClawLatch(theHardwarePlatter);
        theArm = new Arm(theHardwarePlatter);
        theIntake = new Intake(theHardwarePlatter);
        wheeliebar = new Wheeliebar(theHardwarePlatter);
        runtime = new ElapsedTime();

        runtime.reset();
    }

    public void loop() {
        telemetry.addData("Encoders_run", theHardwarePlatter.elevatorDrive.getCurrentPosition());
        telemetry.addData("Step", step);
        telemetry.addData("Timer", runtime.time());
        telemetry.addData("Timeout", timeout);
        telemetry.update();

        if ((runtime.time() > timeout) && !isBusy()) {
            step++;
            runtime.reset();
            timeout = 2;

            if (step == 1) {
                timeout = 0.3;
                clawLatch.open();
            } else if (step == 2) {
                //theElevatorClimb.autonElevatorClimb(0.25, 1);
                theElevatorClimb.dropDown();
                timeout = 0.5;
            } else if (step == 3) {
                theElevatorClimb.climberStop();
            } else if (step == 4) {
                theElevatorClimb.climbUp();
                timeout = 3;
            } else if (step == 5) {
                theElevatorClimb.climberStop();
                timeout = 0;
            } else if (step == 6) {
                theChassis.driveAuton(0.25, 0.5);
            } else if (step == 7) {
                theChassis.driveAuton(-6.5, 0.5);
            } else if (step == 8) wheeliebar.wheeliebar_RB_down();
            else if (step == 9) {
                theArm.drive();
            } else if (step == 10) theChassis.driveAuton(6.5, 0.2);
            else if (step == 11) clawLatch.closeAuton();
            else if (step == 12) {
                theArm.unfold();
                timeout = 3;
            } else if (step == 13) theIntake.openDumpServo();
            else if (step == 14) theIntake.closeDumpServo();
            else if (step == 15) theArm.pickUp();
            else if (step == 16) theIntake.driveCombine(true, false);
            else if (step == 17) theIntake.driveCombine(false, false);
            else if (step == 18) theChassis.driveAuton(6, 0.5);
            else if (step == 19) theChassis.driveAuton(-6, 0.5);
            else if (step == 20) theArm.release();
            else if (step == 21) theIntake.openDumpServo();
            else if (step == 22) timeout = 2000;
            else if (step == 23) theIntake.closeDumpServo();
            else if (step == 24) theArm.drive();
            else if (step == 25) theChassis.driveAuton(1, 0.5);
            else if (step == 26) theArm.pickUp();
                //else if(step == 27) theChassis.markerServo();
                //else if(step == 28) theChassis.markerServo();
            else if (step == 29) theChassis.driveAuton(1, 0.5);
            else if (step == 30) theArm.pickUp();
            else if (step == 31) wheeliebar.wheeliebar_RB_middle();
            else if (step == 32) theChassis.driveAuton(1, 0.5);

        }
    }

    boolean isBusy() {
        theArm.moveOrHoldPosition();
        return (theHardwarePlatter.elevatorDrive.isBusy() || theHardwarePlatter.leftFrontDrive.isBusy() ||
                theArm.isMoving());
    }
}
