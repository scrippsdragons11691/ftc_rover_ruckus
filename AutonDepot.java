package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class AutonDepot extends LinearOpMode {

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

        theHardwarePlatter = new HardwarePlatter(hardwareMap);
        theChassis = new ChassisAuton(theHardwarePlatter);
        theArm = new Arm(theHardwarePlatter);
        theIntake = new Intake(theHardwarePlatter);
        theElevatorClimb = new ElevatorClimb(theHardwarePlatter);
        theClawLatch = new ClawLatch(theHardwarePlatter);
        theWheeliebar = new Wheeliebar(theHardwarePlatter);
        theMarkerServo = new MarkerServo(theHardwarePlatter);
        theSampler = new Sampler(hardwareMap);

        theMarkerServo.markerServo_initialized();

        telemetry.addData("robot", "initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

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
            delay(5000);

            int LeftRightCenter = theSampler.sampleMinerals();
            theSampler.deactivate();

            //1) Land and unlatch from lander

            theElevatorClimb.dropDown();        // robot goes up to release the latch
            delay(750);
            theElevatorClimb.climberStop();   //stops the climber motor

            //elevatorClimbUp();
            if (!theHardwarePlatter.climberLimitSwUp.isPressed())
                theElevatorClimb.climbUpAuton();  //robot drops down
            else theElevatorClimb.climberStop();   // Stops motor
            delay(500);

            theClawLatch.autoOpen();                //Reset climber Hook Position
            delay(500);

            wheeliebarDown();

            //2.1) Move the Cube for left

            //int LeftRightCenter = 1;

            if (LeftRightCenter == -1) {
                theChassis.rotateAuton(-2.75, 0.5);         // Rotate toward the mineral
                delay(750);
                driveAuton(-20.0, 0.5);       // Drive backward 20 inches toward the crater
                theChassis.rotateAuton(6, 0.5);          // Rotate to the crater
                delay(750);

                //3.1) Drop Marker after left

                driveAuton(-20.0, 0.5);       // Drive backward 20 inches toward the crater

                theMarkerServo.markerServo_down();      // Drop marker
                delay(750);

                //4) Drive to Crater after left

                driveAuton(33, 0.8);        // Drive forward 35 inches
                theMarkerServo.markerServo_up();      // raise marker
                delay(1500);
                theChassis.rotateAuton(-11.5, 0.5);         // Rotate toward the mineral
                delay(1000);
                driveAuton(-8, 0.8);
            }

            //2.2)  Move the Cube for Center

            else if (LeftRightCenter == 0) {

                driveAuton(-26.0, 0.5);       // Drive backward 20 inches toward the crater

                theChassis.rotateAuton(4.75, 0.5);
                delay(200);
                driveAuton(-5, 0.5);       // Drive backward 20 inches toward the crater

                //3.1) Drop Marker after center

                theMarkerServo.markerServo_down();      // Drop marker
                delay(300);

                //4) Drive to Crater after center
                theMarkerServo.markerServo_up();      // raise marker
                driveAuton(18, 0.4);        // Drive forward 35 inches
                theChassis.rotateAuton(-1.5, 0.5);         // Rotate toward the mineral
                delay(200);
                driveAuton(15, 0.8);        // Drive forward 35 inches
                theMarkerServo.markerServo_up();      // raise marker
                theChassis.rotateAuton(-12, 0.5);         // Rotate toward the mineral
                delay(200);
                driveAuton(-8, 0.8);
            }

            //2.1) Move the Cube for right

            else if (LeftRightCenter == 1) {

                theChassis.rotateAuton(2.5, 0.4);         // Rotate toward the mineral
                delay(100);
                driveAuton(-20, 0.3);       // Drive backward 20 inches toward the crater
                theWheeliebar.down();
                delay(1000);

                //3.1) Drop Marker after right
                theChassis.rotateAuton(-5.25, 0.4);          // Rotate to the crater
                delay(100);
                theWheeliebar.up();
                delay(100);
                driveAuton(-12, 0.3);       // Drive backward 20 inches toward the crater
                driveToWall(29);         // Rotate toward the mineral
                delay(100);
                theChassis.rotateAuton(6.5, 0.4);          // Rotate to the crater
                delay(100);
                theMarkerServo.markerServo_down();      // Drop marker
                delay(500);

                //4) Drive to Crater after right

                theMarkerServo.markerServo_up();      // raise marker
                driveAuton(18, 0.4);        // Drive forward 35 inches
                theChassis.rotateAuton(-1.0, 0.4);         // Rotate toward the mineral
                delay(200);
                driveAuton(14, 0.8);        // Drive forward 35 inches
                theMarkerServo.markerServo_up();      // raise marker
                theChassis.rotateAuton(-12, 0.4);         // Rotate toward the mineral
                delay(200);
                driveAuton(-8, 0.8);
            }
            // 5 ) Unfold the arms into the Crater

            if (!theHardwarePlatter.climberLimitSwDn.isPressed()) {
                theElevatorClimb.climbDownAuton();
            } else theElevatorClimb.climberStop();   // Stops motor

            theClawLatch.autoClose();              // close the claw latch to allow arm to rotate
            delay(750);

            theArm.unfold();                        //set the arm to unfold position
            theClawLatch.autoOpen();                //Reset climber Hook Position
            delay(3000);                            //wait for 1.5 seconds

            theIntake.openDumpServo();              //open the dump servo
            delay(1000); 
/*            
            theArm.unfold();                        //set the arm to unfold position into the crater
            delay(2000);                            //wait 1.5 seconds

            theIntake.openDumpServo();              //open the dump servo
            delay(1000);                            //wait 1 seconds
*/
            theIntake.closeDumpServo();

            theArm.middle();
            theClawLatch.autoOpen();
            delay(10000);                             //Wait for 10 seconds or for auton to end

        }


    }
    // Telemetry and core functions in while loop to update the telemetry, control arm, and cycle gyro

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

    void driveToWall(double distanceCM) {
        while (!theChassis.driveToWall(distanceCM, -0.3) && opModeIsActive()) ;
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

    void elevatorClimbUp() {
        theElevatorClimb.climbUpAuton();  //robot drops down
        while (opModeIsActive() && !theHardwarePlatter.climberLimitSwUp.isPressed()) ;
        theElevatorClimb.climberStop();   // Stops motor
    }

    void wheeliebarDown() {
        theWheeliebar.down();
        while (opModeIsActive() && theWheeliebar.moveOrHoldPosition()) ;
    }
}
