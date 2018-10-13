package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class Auton extends LinearOpMode{

    ElevatorClimb elevatorClimb;
    Intake intake;
    Sampler sampler;
    ChassisAuton theChassis;

    Auton(HardwarePlatter hwPlatter) {
        intake = new Intake(hwPlatter);
        sampler = new Sampler(hwPlatter);
        elevatorClimb = new ElevatorClimb(hwPlatter);
        theChassis = new ChassisAuton(hwPlatter);
    }

    public void runOpMode(){
        elevatorClimb.dropDown();
        intake.unfold();
        theChassis.driveAuton(10);
        sampler.sampleMinerals();
        intake.pickUp();
        intake.release();
    }

}