package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
/*
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DistanceSensor;
*/

@Autonomous

public class Auton1_1 extends LinearOpMode {
    public void runOpMode() {
        HardwarePlatter theHardwarePlatter = new HardwarePlatter(hardwareMap);
        
        ChassisAuton theChassis = new ChassisAuton(theHardwarePlatter);
        Arm theArm = new Arm(theHardwarePlatter);
        Intake theIntake = new Intake(theHardwarePlatter);
        ElevatorClimb theElevatorclimb= new ElevatorClimb(theHardwarePlatter);
        ClawLatch theClawLatch=new ClawLatch(theHardwarePlatter);
        waitForStart();
        

        while(opModeIsActive()){ //&& theChassis.isBusy() == true){
        
            //theElevatorclimb.autonElevatorClimbDown(4,0.3);
            //theClawLatch.open();
            theChassis.driveAuton(-6.5,0.3);        //Drive forward 6.5 inches
            sleep(2000);                             //Wait 0.5 seconds
            theArm.drive();                         //Set the arm at a 90 degree angle (drive position)
            sleep(2000);                             //Wait 1 second
            theChassis.driveAuton(5.5,0.3);         //Drive backward 6.5 inches
            sleep(5000);                             //Wait for 0.5 seconds
            theArm.unfold();                        //set the arm to unfold position
            sleep(2000);                             //wait for 0.5 seconds
            theIntake.openDumpServo();              //open the dump servo
            sleep(2000);                             //wait 0.5 seconds
            theIntake.closeDumpServo();
             sleep(2000); 
            theArm.pickUp();                        //move arm to pick up position
            sleep(15000);                           //wait for 2 seconds
            
        } 
    }
    

}
