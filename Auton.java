package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class Auton extends LinearOpMode {

    HardwarePlatter theHardwarePlatter;
    ElevatorClimb theElevatorClimb;
    Intake theIntake;
    Sampler theSampler;
    ChassisAuton theChassis;

    public Auton() {
        theHardwarePlatter = new HardwarePlatter(hardwareMap);
        theIntake = new Intake(theHardwarePlatter);
        theSampler = new Sampler(theHardwarePlatter);
        theElevatorClimb = new ElevatorClimb(theHardwarePlatter);
        theChassis = new ChassisAuton(theHardwarePlatter);
    }

    public void runOpMode() {
        theElevatorClimb.dropDown();
        theIntake.unfold();
        theChassis.driveAuton(10);
        theSampler.sampleMinerals();
        theIntake.pickUp();
        theIntake.release();
    }

}