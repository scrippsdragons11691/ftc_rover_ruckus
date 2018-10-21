package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class Auton extends OpMode {

    HardwarePlatter theHardwarePlatter;
    ElevatorClimb theElevatorClimb;
    Arm theArm;
    Sampler theSampler;
    ChassisAuton theChassis;
    private ElapsedTime runtime = new ElapsedTime();
    ClawLatch theClaw;
    Intake intake;
    int step;

    public Auton() {
        theHardwarePlatter = new HardwarePlatter(hardwareMap);

        theSampler = new Sampler(theHardwarePlatter);
        theElevatorClimb = new ElevatorClimb(theHardwarePlatter);
        theChassis = new ChassisAuton(theHardwarePlatter);
        theClaw = new ClawLatch(theHardwarePlatter);
        theArm      = new Arm(theHardwarePlatter);
        intake = new Intake(theHardwarePlatter);
    }

    public void init() {
        telemetry.addData("Robot", "Initialized");
        telemetry.update();
        step = 1;
    }

    public void loop() {
        if(!theChassis.isDriveBusy() && !theArm.isMoving() && step < 6)
        {
            step++;
            if(step == 1)
                theElevatorClimb.dropDown();
            else if(step == 2)
                theChassis.driveAuton(6.5);
            else if(step == 3)
                theArm.drive();
            else if(step == 4)
                theChassis.driveAuton(-6.5);
            else if(step == 5)
                theArm.unfold();
            else if(step == 6) {
                intake.openDumpServo();
                theArm.pickUp();
            }
        }
    }
}