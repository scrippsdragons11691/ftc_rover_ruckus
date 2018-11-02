package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Servo;
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

            
    public void runOpMode() {
        
        theHardwarePlatter = new HardwarePlatter(hardwareMap);
        theChassis = new ChassisAuton(theHardwarePlatter);
        theArm = new Arm(theHardwarePlatter);
        theIntake = new Intake(theHardwarePlatter);
        theElevatorClimb = new ElevatorClimb(theHardwarePlatter);
        theClawLatch = new ClawLatch(theHardwarePlatter);
        theWheeliebar = new Wheeliebar(theHardwarePlatter);
        theMarkerServo = new MarkerServo(theHardwarePlatter);
        
        telemetry.addData("robot", "initialized");
        telemetry.update();

        theMarkerServo.markerServo_initialized();

        waitForStart();
        

        while(opModeIsActive()) {
        
    //1) Land and unlatch from lander
    /*
            theElevatorClimb.dropDown();        // robot goes up to release the latch
            delay(750);
            theElevatorClimb.climberStop();   //stops the climber motor
            theElevatorClimb.climbUpAuton();  //robot drops down
            delay(5000);
            theElevatorClimb.climberStop();   // Stops motor
    */

    //drive to deliver the marker to the Crater 
    
        // to rotate motor speed 0.5 for 0.5 sec = 45 deg

            theChassis.driveAuton(-20.0,0.5);       // Drive backward 22 inches toward the crater
            delay(1500);                            // Wait 1.0 seconds

            theChassis.rotateAuton(7,0.5);         // Rotate to drop the marker
            //theElevatorClimb.dropDown();             // Reset the climber position  
            delay(2000);

            theElevatorClimb.climberStop();
            theMarkerServo.markerServo_down();      // Drop marker
            delay (1000);

            theChassis.rotateAuton(2,0.5);          // Rotate to the crater
            delay(750);

            theChassis.driveAuton(-33,0.8);        // Drive forward 35 inches
            theClawLatch.autoClose();              // close the claw latch to allow arm to rotate
            delay(4000);


        // 2, 3 ) Unfold the arms into the Crater

            theArm.drive();                         //Set the arm at a 90 degree angle (drive position)
            delay(2000);                             //Wait 2 second

            theArm.unfold();                        //set the arm to unfold position
            theClawLatch.autoOpen();                //Reset climber Hook Position
            delay(1500);                            //wait for 1.5 seconds

            theArm.unfold();                        //set the arm to unfold position into the crater
            delay(1500);                            //wait 1.5 seconds

            theIntake.openDumpServo();              //open the dump servo
            delay(1000);                            //wait 1 seconds

            theIntake.closeDumpServo();

            theArm.middle ();
            theClawLatch.autoOpen();
            delay(10000);                             //Wait for 10 seconds or for auton to end
    
            }   
        }

    // Telemetry and core functions in while loop to update the telemetry, control arm, and cycle gyro
    
        void delay(int timeout_ms){
            ElapsedTime runtime = new ElapsedTime();
            runtime.reset();
              while(opModeIsActive() && (runtime.time() < timeout_ms/1000.0)) {
              //sleep(timeout_ms);
              telemetry.addData("Encoders_run", theHardwarePlatter.elevatorDrive.getCurrentPosition());
              telemetry.addData("time", runtime.time());
              telemetry.addData("timeout", timeout_ms);
              telemetry.addData("Pot_Voltage", theHardwarePlatter.armPotentiometer.getVoltage());
              theArm.display(telemetry);
              theChassis.isGyroBusy();
              telemetry.update();
              if(theArm.isMoving()) theArm.moveOrHoldPosition();
              sleep(100);
            }
            theArm.stop();
        }    
        
}
