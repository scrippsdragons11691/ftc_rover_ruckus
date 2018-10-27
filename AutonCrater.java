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
            //theClawLatch.closeAuton();
            //theElevatorClimb.dropDown();
            //delay(500);
            //theElevatorClimb.climberStop();
            //theElevatorClimb.climbUpAuton();
            //delay(3000);
            //theElevatorclimb.autonElevatorClimb(30,0.3);
            //theClawLatch.open();
            //delay(200);
            theElevatorClimb.climberStop();
            
        // 2, 3 ) Unfold the arms
            theChassis.driveAuton(-6.5,0.3);        //Drive forward 6.5 inches
            delay(1500);                             //Wait 0.5 seconds

            theArm.drive();                         //Set the arm at a 90 degree angle (drive position)
            delay(3000);                             //Wait 1 second

            theChassis.driveAuton(5.5,0.3);         //Drive backward 6.5 inches
            delay(1500);                             //Wait for 0.5 seconds

            theArm.unfold();                        //set the arm to unfold position
            delay(1000);                             //wait for 0.5 seconds

            //theArm.stop();
            //delay(2000);
            
            theIntake.openDumpServo();              //open the dump servo
            delay(1000);                             //wait 0.5 seconds

            theIntake.closeDumpServo();
            sleep(100); 
            
            theArm.pickUp();                        //move arm to pick up position*/
            delay (500);

            theIntake.driveCombine(true,false);
            delay (500);
            
            theChassis.driveAuton(-5,0.3);        //Drive forward 6.5 inches
            delay(2000);                             //Wait 0.5 seconds

            theChassis.driveAuton(5.5,0.3);         //Drive backward 6.5 inches
            delay(5000);                             //Wait for 0.5 seconds

            theArm.release();                         //Set the arm at a 90 degree angle (drive position)
            delay (1000);

            theArm.release();
            theIntake.openDumpServo();              //open the dump servo
            delay(1000);                             //wait for 0.5 seconds
            //delay(2000);                             //wait 0.5 seconds

            theIntake.closeDumpServo();
            delay (500);
            
            theArm.drive ();
            delay (2000);                           //wait for 2 seconds
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
