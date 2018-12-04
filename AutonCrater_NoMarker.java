package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class AutonCrater_NoMarker extends LinearOpMode {

    ChassisAuton theChassis;
    HardwarePlatter theHardwarePlatter;
    Arm theArm;
    Intake theIntake;
    ElevatorClimb theElevatorClimb;
    ClawLatch theClawLatch;
    Wheeliebar theWheeliebar;
    MarkerServo theMarkerServo;
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
        theMarkerServo = new MarkerServo(theHardwarePlatter);

        //theWheeliebar.wheeliebar_RB_down();

        telemetry.addData("robot", "initialized");
        telemetry.update();

        waitForStart();
        //int LeftRightCenter = -1;

        if (opModeIsActive()) {

            //0) Sample minerals -1 is left, 1 Right, 0 Center
            theSampler.initVuforia();

            if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
                theSampler.initTfod();
            } else {
                telemetry.addData("Sorry!", "This device is not compatible with TFOD");
            }
            theSampler.activate();
            theSampler.sampleMinerals();
            theSampler.display(telemetry);
            telemetry.update();

            delay(3500);

            int LeftRightCenter = theSampler.sampleMinerals();
            theSampler.deactivate();

            //1) Land and unlatch from lander
            theElevatorClimb.dropDown();        // robot goes up to release the latch
            delay(750);

            theElevatorClimb.climberStop();   //stops the climber motor
            //wheeliebarDown();
            theWheeliebar.down();
            delay(1000);

            if (!theHardwarePlatter.climberLimitSwUp.isPressed())
                theElevatorClimb.climbUp();  //robot drops down
            else
                theElevatorClimb.climberStop();   // Stops motor
            delay(500);

            theClawLatch.autoOpen();                //Reset climber Hook Position
            delay(1000);


            //2.1) Move the Cube for left

            if (LeftRightCenter == -1) {
//              theWheeliebar.down();
//              delay (500);
                theChassis.rotateAuton(-2.75, 0.3);         // Rotate toward the mineral
                delay(100);
                theChassis.driveAuton(-12.0, 0.5);       // Drive backward 20 inches toward the crater
                delay(100);
                theChassis.rotateAuton(3.5, 0.3);          // Rotate to the crater
                delay(100);
                theChassis.driveAuton(-2.0, 0.5);       // Drive backward 20 inches toward the crater
                delay(200);
            }
            //2.2)  Move the Cube for Center

            else if (LeftRightCenter == 0) {
                //             theWheeliebar.down();
                //             delay (500);

                theChassis.driveAuton(-12.0, 0.5);       // Drive backward 20 inches toward the crater
                delay(200);

            }

            //2.1) Move the Cube for right

            else if (LeftRightCenter == 1) {
                //             theWheeliebar.down();
                //             delay (500);
                theChassis.rotateAuton(2.5, 0.4);         // Rotate toward the mineral
                delay(100);
                theChassis.driveAuton(-12, 0.3);       // Drive backward 20 inches toward the crater
                delay(100);
                theChassis.rotateAuton(-2.75, 0.4);         // Rotate toward the mineral
                delay(100);
                theChassis.driveAuton(-2.0, 0.5);       // Drive backward 20 inches toward the crater
                delay(200);
            }

            if (!theHardwarePlatter.climberLimitSwDn.isPressed()) {
                theElevatorClimb.dropDown();
            } else theElevatorClimb.climberStop();   // Stops motor

            theClawLatch.autoClose();              // close the claw latch to allow arm to rotate
            delay(750);

            theArm.drive();                         //Set the arm at a 90 degree angle (drive position)
            delay(100);                             //Wait 2 second

            theArm.unfold();                        //set the arm to unfold position
            delay(3000);                            //wait for 2.0 seconds

            theClawLatch.autoOpen();                //push over the arm if it does not go on it's own
            delay(1000);                            //wait for 1.0 seconds

            theIntake.openDumpServo();              //open the dump servo
            delay(100);                            //wait 1 seconds

            theArm.unfold();                        //set the arm to unfold position
            delay(2000);

            theIntake.openDumpServo();              //open the dump servo
            delay(100);                            //wait 1 seconds

            theIntake.closeDumpServo();

            theArm.middle();
            theClawLatch.autoOpen();
            delay(10000);                             //Wait for 10 seconds or for auton to end

        }
    }

    void delay(int timeout_ms) {
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while (opModeIsActive() && (runtime.time() < timeout_ms / 1000.0)) {
            //sleep(timeout_ms);
            telemetry.addData("Encoders_run", theHardwarePlatter.elevatorDrive.getCurrentPosition());
            telemetry.addData("Distance from wall", theChassis.getDistanceFromWall());
            telemetry.addData("time", runtime.time());
            telemetry.addData("timeout", timeout_ms);
            telemetry.addData("Pot_Voltage", theHardwarePlatter.armPotentiometer.getVoltage());
            theArm.display(telemetry);
            theSampler.display(telemetry);
            telemetry.update();
            if (theArm.isMoving()) theArm.moveOrHoldPosition();
            theWheeliebar.moveOrHoldPosition();
            sleep(100);
        }
        theArm.stop();
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

    void wheeliebarDown() {
        theWheeliebar.down();
        while (opModeIsActive() && theWheeliebar.moveOrHoldPosition()) ;
    }

}

