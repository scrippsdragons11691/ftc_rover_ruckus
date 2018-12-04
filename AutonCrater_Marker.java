package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class AutonCrater_Marker extends LinearOpMode {

    ChassisAuton theChassis;
    HardwarePlatter theHardwarePlatter;
    Arm theArm;
    Intake theIntake;
    ElevatorClimb theElevatorClimb;
    ClawLatch theClawLatch;
    Wheeliebar theWheeliebar;
    Sampler theSampler;

    public void runOpMode() {
        theSampler = new Sampler(hardwareMap);
        theHardwarePlatter = new HardwarePlatter(hardwareMap);
        theChassis = new ChassisAuton(theHardwarePlatter);
        theArm = new Arm(theHardwarePlatter);
        theIntake = new Intake(theHardwarePlatter);
        theElevatorClimb = new ElevatorClimb(theHardwarePlatter);
        theClawLatch = new ClawLatch(theHardwarePlatter);
        theWheeliebar = new Wheeliebar(theHardwarePlatter);

        //theWheeliebar.wheeliebar_RB_down();

        telemetry.addData("robot", "initialized");
        telemetry.update();

        theSampler.initVuforia();


        waitForStart();

        int LeftRightCenter = theSampler.sampleMinerals();
        theSampler.deactivate();

        if (opModeIsActive()) {
            if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
                theSampler.initTfod();
            } else {
                telemetry.addData("Sorry!", "This device is not compatible with TFOD");
            }
            theSampler.activate();
            //1) Land and unlatch from lander
 /*           theClawLatch.closeAuton();
            theElevatorClimb.dropDown();        // robot goes up to release the latch
            delay(750);
            theElevatorClimb.climberStop();     //stops the climber motor
            theElevatorClimb.climbUpAuton();    //robot drops down
            delay(5000);
            theElevatorClimb.climberStop();     // Stops motor
 */
            // 2, 3 ) Unfold the arms
            driveAuton(-6.5, 0.5);
            //Drive forward 6.5 inches
            theWheeliebar.down();
            theElevatorClimb.dropDown();         // Reset the climber position     
            delay(2000);                             //Wait 1.0 seconds


            theElevatorClimb.climberStop();
            drive();                         //Set the arm at a 90 degree angle (drive position)

            driveAuton(6.0, 0.5);         //Drive backward 6.5 inches

            unfold();                        //set the arm to unfold position

            theIntake.openDumpServo();              //open the dump servo

            delay(1500);                             //wait 0.5 seconds

            theIntake.closeDumpServo();

            delay(500);

// 4,6  pick up center mineral and deliver

            pickUp();                        //move arm to pick up position*/

            if (LeftRightCenter == -1) {
                rotateAuton(-2.75, 0.5);
            } else if (LeftRightCenter == 1) {
                theChassis.rotate(-1);
                delay(250);
            }

            theIntake.driveCombine(true, false);
            delay(1000);

            driveAuton(-2, 0.5);        //Drive forward 2 inches to pick up mineral             //Wait 1 seconds

            theIntake.driveCombine(false, false);
            delay(500);
         /*   
            //theChassis.turn(30);
            theArm.drive(); 
            delay (1000);

            theChassis.driveAuton(2.0,0.5);         //Drive backward 2 inches to align the delivery
            theArm.drive(); 
            delay(1000);                             //Wait for 1 seconds

           
           //theChassis.turn(-40);
            delay (1000);

            theArm.release();
            delay (1000);

            theArm.release();
            theIntake.openDumpServo();              //open the dump servo
            delay(1000);                             //wait for 1.0 seconds

            theIntake.closeDumpServo();
        */
            middle();
            //delay (2000);                           //wait for 2 seconds

// drive into the crater for the end of the match

            theWheeliebar.middle();
            driveAuton(-12.0, 0.5);

            //Drive backward 22 inches toward the crater
            middle();
            //delay(2000);                             //Wait for 2.0 seconds to make sure wheelie bars are up.

            //theChassis.driveAuton(-5.0,0.5);         //Drive backward 14 inches to go into the crater
            //theArm.middle ();
            delay(10000);                             //Wait for 10 seconds for auton to end
        }
    }

    void delay(int timeout_ms) {
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while (opModeIsActive() && (runtime.time() < timeout_ms / 1000.0)) {
            //sleep(timeout_ms);
            telemetry.addData("Encoders_run", theHardwarePlatter.elevatorDrive.getCurrentPosition());
            telemetry.addData("time", runtime.time());
            telemetry.addData("timeout", timeout_ms);
            telemetry.addData("Pot_Voltage", theHardwarePlatter.armPotentiometer.getVoltage());
            theArm.display(telemetry);
            telemetry.update();
            theArm.moveOrHoldPosition();
            theWheeliebar.moveOrHoldPosition();
        }
        theArm.stop();
        theChassis.stop();
    }

    void unfold() {
        theArm.unfold();
        while (opModeIsActive() && theArm.moveOrHoldPosition()) ;
    }

    void middle() {
        theArm.middle();
        while (opModeIsActive() && theArm.moveOrHoldPosition()) ;
    }

    void release() {
        theArm.release();
        while (opModeIsActive() && theArm.moveOrHoldPosition()) ;
    }

    void pickUp() {
        theArm.pickUp();
        while (opModeIsActive() && theArm.moveOrHoldPosition()) ;
    }

    void drive() {
        theArm.drive();
        while (opModeIsActive() && theArm.moveOrHoldPosition()) ;
    }

    void driveAuton(double distance, double speed) {
        theChassis.driveAuton(distance, speed);
        while (opModeIsActive() && theChassis.isDriveBusy()) {
        }
    }

    void rotateAuton(double distance, double speed) {
        theChassis.rotateAuton(distance, speed);
        while (opModeIsActive() && theChassis.isDriveBusy()) ;
    }
}
