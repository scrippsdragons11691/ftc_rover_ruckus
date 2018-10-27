package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous

public class AutonCrater extends LinearOpMode{
        HardwarePlatter theHardwarePlatter;
        
        public void runOpMode() {
        theHardwarePlatter = new HardwarePlatter(hardwareMap);
        
        ChassisAuton theChassis = new ChassisAuton(theHardwarePlatter);
        Arm theArm = new Arm(theHardwarePlatter);
        Intake theIntake = new Intake(theHardwarePlatter);
        ElevatorClimb theElevatorclimb= new ElevatorClimb(theHardwarePlatter);
        ClawLatch theClawLatch=new ClawLatch(theHardwarePlatter);
        Wheeliebar theWheeliebar = new Wheeliebar(theHardwarePlatter);
        waitForStart();
        

        while(opModeIsActive()){ //&& theChassis.isBusy() == true){
        
        //1) Land and unlatch from lander
            theClawLatch.close();
            theElevatorclimb.dropDown();
            delay(300);
            theElevatorclimb.climberStop();
            theElevatorclimb.climbUpAuton();
            delay(3000);
            //theElevatorclimb.autonElevatorClimb(30,0.3);
            theClawLatch.open();
            delay(5000);
            theElevatorclimb.climberStop();
            
        // 2, 3 ) Unfold the arms
            theChassis.driveAuton(-6.5,0.3);        //Drive forward 6.5 inches
            delay(1000);                             //Wait 0.5 seconds

            theArm.drive();                         //Set the arm at a 90 degree angle (drive position)
            delay(1000);                             //Wait 1 second

            theChassis.driveAuton(5.5,0.3);         //Drive backward 6.5 inches
            delay(1000);                             //Wait for 0.5 seconds
            theArm.unfold();                        //set the arm to unfold position

            delay(2000);                             //wait for 0.5 seconds
            theIntake.openDumpServo();              //open the dump servo

            delay(2000);                             //wait 0.5 seconds
            theIntake.closeDumpServo();

            sleep(2000); 
            theArm.pickUp();                        //move arm to pick up position*/
            delay(10000);
            

            //sleep(500);                           //wait for 2 seconds
        }
            
        }
        void delay(int timeout_ms){
              sleep(timeout_ms);
              telemetry.addData("Encoders_run", theHardwarePlatter.elevatorDrive.getCurrentPosition());
              telemetry.addData("Pot_Voltage", theHardwarePlatter.armPotentiometer.getVoltage());
              telemetry.update();
        }    
        
}
