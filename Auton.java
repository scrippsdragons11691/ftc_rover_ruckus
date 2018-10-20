package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class Auton extends LinearOpMode {

    HardwarePlatter theHardwarePlatter;
    ElevatorClimb theElevatorClimb;
    Arm theArm;
    Sampler theSampler;
    ChassisAuton theChassis;
    private ElapsedTime runtime = new ElapsedTime();
    ClawLatch theClaw;
    Intake intake;

    public Auton() {
        theHardwarePlatter = new HardwarePlatter(hardwareMap);

        //theArm = new Arm(theHardwarePlatter);
        theSampler = new Sampler(theHardwarePlatter);
        theElevatorClimb = new ElevatorClimb(theHardwarePlatter);
        theChassis = new ChassisAuton(theHardwarePlatter);
        theClaw = new ClawLatch(theHardwarePlatter);
        theArm      = new Arm(theHardwarePlatter);
        intake = new Intake(theHardwarePlatter);
    }

    public void runOpMode() {
        telemetry.addData("Robot", "Initialized");
        telemetry.update();

        waitForStart();

        //theElevatorClimb.dropDown();
        theChassis.driveAuton(6.5);

        do {
            theArm.drive();
        } while(theArm.isMoving() && opModeIsActive());

        theChassis.driveAuton(-6.5);

        do {
            theArm.unfold();
        } while(theArm.isMoving() && opModeIsActive());

        intake.openDumpServo();

        do {
            theArm.pickUp();
        } while(theArm.isMoving() && opModeIsActive());

        //while(opModeIsActive() && runtime.time() < UNLATCH_TIME) theClaw.open();

        //theSampler.sampleMinerals();
        //theArm.pickUp();
        //theArm.release();
    }

}