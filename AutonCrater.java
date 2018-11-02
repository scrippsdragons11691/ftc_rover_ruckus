package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous

public class AutonCrater extends LinearOpMode{

    ChassisAuton theChassis;
    HardwarePlatter theHardwarePlatter;
    Arm theArm;
    Intake theIntake;
    ElevatorClimb theElevatorClimb;
    ClawLatch theClawLatch;
    Wheeliebar theWheeliebar;
    
    public void runOpMode() {
        
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

        waitForStart();
        

        while(opModeIsActive()) {
        
        //1) Land and unlatch from lander
            theClawLatch.closeAuton();
            theElevatorClimb.dropDown();        // robot goes up to release the latch
            delay(750);
            theElevatorClimb.climberStop();     //stops the climber motor
            theElevatorClimb.climbUpAuton();    //robot drops down
            delay(5000);
            theElevatorClimb.climberStop();     // Stops motor
            
        // 2, 3 ) Unfold the arms
            theChassis.driveAuton(-6.5,0.5);        //Drive forward 6.5 inches
            theWheeliebar.wheeliebar_RB_down();
            theElevatorClimb.dropDown();         // Reset the climber position     
            delay(2000);                             //Wait 1.0 seconds

            theElevatorClimb.climberStop();
            theArm.drive();                         //Set the arm at a 90 degree angle (drive position)
            delay(2000);                             //Wait 1 second

            theChassis.driveAuton(6.0,0.5);         //Drive backward 6.5 inches
            delay(1000);                             //Wait for 0.5 seconds

            theArm.unfold();                        //set the arm to unfold position
            delay(1500);                             //wait for 1.5 seconds

            theArm.unfold();                        //set the arm to unfold position
            delay(1500);                             //wait 1.5 seconds
            
            theIntake.openDumpServo();              //open the dump servo
            //theArm.unfold();                        //set the arm to unfold position
            delay(1500);                             //wait 0.5 seconds

            theIntake.closeDumpServo();
            //theArm.unfold();                        //set the arm to unfold position
            sleep(500); 
            
            
// 4,6  pick up center mineral and deliver

            theArm.pickUp();                        //move arm to pick up position*/
            delay (1000);

            theIntake.driveCombine(true,false);
            delay (1000);

            theChassis.driveAuton(-2,0.5);        //Drive forward 2 inches to pick up mineral
            delay(1000);                             //Wait 1 seconds

            theIntake.driveCombine(false,false);
            delay (500);
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
            theArm.middle ();
            delay (2000);                           //wait for 2 seconds

// drive into the crater for the end of the match

            theWheeliebar.wheeliebar_RB_middle();
            theChassis.driveAuton(-12.0,0.5);         //Drive backward 22 inches toward the crater
            theArm.middle ();
            delay(2000);                             //Wait for 2.0 seconds to make sure wheelie bars are up.
            
            //theChassis.driveAuton(-5.0,0.5);         //Drive backward 14 inches to go into the crater
            //theArm.middle ();
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
              telemetry.update();
              if(theArm.isMoving()) theArm.moveOrHoldPosition();
              sleep(100);
            }
            theArm.stop();
        }    
        
}
