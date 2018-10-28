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
    double holdTime;

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

        if (!isBusy()) {
            step++;
            runtime.reset();
            timeout = 5;
            holdTime = 0.0;

            if(     step ==  1) clawLatch.openAuton();
            else if(step ==  2) { theElevatorClimb.dropDown(); holdTime = 0.3; }
            else if(step ==  3)  theElevatorClimb.climberStop();
            else if(step ==  4) { theElevatorClimb.climbUp(); holdTime = 3; }
            else if(step ==  5) theElevatorClimb.climberStop();
            else if(step ==  6) theChassis.driveAuton(-6.5, 0.3);
            else if(step ==  7) theArm.drive();
            else if(step ==  8) theChassis.driveAuton(5.5, 0.3);
            else if(step ==  9) wheeliebar.wheeliebar_RB_down();
            else if(step == 10) theArm.drive();
            else if(step == 11) theChassis.driveAuton(6.5, 0.2);
            else if(step == 12) clawLatch.closeAuton();
            else if(step == 13) theArm.unfold();
            else if(step == 14) theIntake.openDumpServo();
            else if(step == 15) theIntake.closeDumpServo();
            else if(step == 16) theArm.pickUp();
            else if(step == 17) theIntake.driveCombine(true, false);
            else if(step == 18) theIntake.driveCombine(false, false);
            else if(step == 19) theChassis.driveAuton(6, 0.5);
            else if(step == 20) theChassis.driveAuton(-6, 0.5);
            else if(step == 21) theArm.release();
            else if(step == 22) theIntake.openDumpServo();
            else if(step == 23) holdTime = 2;
            else if(step == 24) theIntake.closeDumpServo();
            else if(step == 25) theArm.drive();
            else if(step == 26) theChassis.driveAuton(1, 0.5);
            else if(step == 27) theArm.pickUp();
          //else if(step == 28) theChassis.markerServo();
          //else if(step == 29) theChassis.markerServo();
            else if(step == 30) theChassis.driveAuton(1, 0.5);
            else if(step == 31) theArm.pickUp();
            else if(step == 32) wheeliebar.wheeliebar_RB_middle();
            else if(step == 33) theChassis.driveAuton(1, 0.5);
        }
    }

    boolean isBusy() {
        boolean flag;
        if(runtime.time() < holdTime)
        {
            flag = true;
        } else if(runtime.time() < timeout) {
            theArm.moveOrHoldPosition();
            flag = theHardwarePlatter.elevatorDrive.isBusy() || theHardwarePlatter.leftFrontDrive.isBusy() ||
                    theArm.isMoving();
        } else {
            theArm.stop();
            theChassis.driveStop();
            theElevatorClimb.climberStop();
            flag = false;
        }
        return(flag);
    }
}
