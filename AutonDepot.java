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
        
    //0) Sample minerals -1 is left, 1 Right, 0 Center
        int LeftRightCenter = -1;
    
    //1) Land and unlatch from lander
    
            theElevatorClimb.dropDown();        // robot goes up to release the latch
            delay(750);
            theElevatorClimb.climberStop();   //stops the climber motor
            theElevatorClimb.climbUpAuton();  //robot drops down
            delay(3500);
            theElevatorClimb.climberStop();   // Stops motor
    
    //2.1) Move the Cube for left
        
        if (LeftRightCenter == -1) {
            theChassis.rotateAuton(-2.75,0.5);         // Rotate toward the mineral
            delay(750);
            theChassis.driveAuton(-20.0,0.5);       // Drive backward 20 inches toward the crater 
            delay(1500); 
            theChassis.rotateAuton(6,0.5);          // Rotate to the crater
            delay(750);
            
    //3.1) Drop Marker after left
    
            theChassis.driveAuton(-20.0,0.5);       // Drive backward 20 inches toward the crater 
            delay(1500); 
            theMarkerServo.markerServo_down();      // Drop marker
            delay(750);
    
    //4) Drive to Crater after left
    
            theChassis.driveAuton(33,0.8);        // Drive forward 35 inches
            theMarkerServo.markerServo_up();      // raise marker
            delay(1500);
            theChassis.rotateAuton(-11.5,0.5);         // Rotate toward the mineral
            delay(1000); 
            theChassis.driveAuton(-8,0.8);
        }
        
    //2.2)  Move the Cube for Center
            
            else if (LeftRightCenter == 0){
            
            theChassis.driveAuton(-26.0,0.5);       // Drive backward 20 inches toward the crater 
            delay(200); 
            theChassis.rotateAuton(4.75,0.5);  
            delay(200);
            theChassis.driveAuton(-5,0.5);       // Drive backward 20 inches toward the crater 
            // Rotate to the crater
            delay(200);
            
    //3.1) Drop Marker after center
    
            theMarkerServo.markerServo_down();      // Drop marker
            delay(300);
    
    //4) Drive to Crater after center
            theMarkerServo.markerServo_up();      // raise marker
            theChassis.driveAuton(18,0.4);        // Drive forward 35 inches
            delay(500);
            theChassis.rotateAuton(-1.5,0.5);         // Rotate toward the mineral
            delay(200); 
            theChassis.driveAuton(15,0.8);        // Drive forward 35 inches
            delay(200);
            theMarkerServo.markerServo_up();      // raise marker
            theChassis.rotateAuton(-12,0.5);         // Rotate toward the mineral
            delay(200); 
            theChassis.driveAuton(-8,0.8);
        }
        
        //2.1) Move the Cube for right
        
        else if (LeftRightCenter == 1) {
            
            theChassis.rotateAuton(2.5,0.4);         // Rotate toward the mineral
            delay(100);
            theChassis.driveAuton(-21.5,0.3);       // Drive backward 20 inches toward the crater 
            delay(100); 

            
    //3.1) Drop Marker after right
            theChassis.rotateAuton(-5.25,0.4);          // Rotate to the crater
            delay(100);
            theChassis.driveAuton(-14,0.3);       // Drive backward 20 inches toward the crater 
            delay(100); 
            theChassis.rotateAuton(7.,.4);          // Rotate to the crater
            delay(100);
            theMarkerServo.markerServo_down();      // Drop marker
            delay(500);
    
    //4) Drive to Crater after right
    
            theMarkerServo.markerServo_up();      // raise marker
            theChassis.driveAuton(18,0.4);        // Drive forward 35 inches
            delay(500);
            theChassis.rotateAuton(-1,0.4);         // Rotate toward the mineral
            delay(200); 
            theChassis.driveAuton(16,0.8);        // Drive forward 35 inches
            delay(200);
            theMarkerServo.markerServo_up();      // raise marker
            theChassis.rotateAuton(-12,0.4);         // Rotate toward the mineral
            delay(200); 
            theChassis.driveAuton(-8,0.8);
        }
          
        // 5 ) Unfold the arms into the Crater
            
            theClawLatch.autoClose();              // close the claw latch to allow arm to rotate
            delay(750);

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
