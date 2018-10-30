package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

        waitForStart();
        

        while(opModeIsActive()) {
        
        //1) Land and unlatch from lander
            //theClawLatch.closeAuton();
            theElevatorClimb.dropDown();        // robot goes up to release the latch
            delay(500);
            theElevatorClimb.climberStop();   //stops the climber motor
            theElevatorClimb.climbUpAuton();  //robot drops down
            delay(4500);
            theElevatorClimb.climberStop();   // Stops motor


    //drive to deliver the marker to the Crater 
    // to rotate motor speed 0.5 for 0.5 sec = 45 deg

            theChassis.driveAuton(-20.0,0.5);         //Drive backward 22 inches toward the crater
            delay (1500);
            
            theChassis.rotate(0.5);
            delay(1075);
            theChassis.rotate(0.0);
            
            theMarkerServo.markerServo_down();
            delay (1000);

            theChassis.rotate(0.5);
            delay(325);
            theChassis.rotate(0.0);
            theChassis.driveAuton(-33,0.8);        //Drive forward 35 inches
            delay(4000);
            
        // 2, 3 ) Unfold the arms


            theArm.drive();                         //Set the arm at a 90 degree angle (drive position)
            delay(2000);                             //Wait 1 second

            theArm.unfold();                        //set the arm to unfold position
            delay(2500);                             //wait for 0.5 seconds

            theIntake.openDumpServo();              //open the dump servo
            delay(1000);                             //wait 0.5 seconds

            theIntake.closeDumpServo();

            theArm.middle ();
            delay(10000);                             //Wait for 10 seconds for auton to end
    
        }   
        }
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
