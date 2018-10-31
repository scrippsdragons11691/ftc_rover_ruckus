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
<<<<<<< HEAD
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
        
=======
public class AutonDepot extends OpMode {
    private HardwarePlatter theHardwarePlatter;
    private ChassisAuton theChassis;
    private Arm theArm;
    private Intake theIntake;
    private ElevatorClimb theElevatorClimb;
    private ClawLatch clawLatch;
    private Wheeliebar wheeliebar;
    private double timeout;
    private int step;
    private ElapsedTime runtime;
    private double holdTime;

    public AutonDepot() {
        step = 4;
        timeout = 0;
    }

    public void init() {
>>>>>>> b8a405f1173ae338a307203681dc5d081ce8e13d
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

<<<<<<< HEAD

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
        
=======
        if (!isBusy()) {
            step++;
            runtime.reset();
            timeout = 5;
            holdTime = 0.0;

            if(     step ==  1) clawLatch.open();
            else if(step ==  2) { theElevatorClimb.dropDown(); holdTime = 0.3; }
            else if(step ==  3)  theElevatorClimb.climberStop();
            else if(step ==  4) { theElevatorClimb.climbUp(); holdTime = 3; }
            else if(step ==  5) theElevatorClimb.climberStop();
            else if(step ==  6) theChassis.driveAuton(-6.5, 0.3);
            else if(step ==  7) theArm.drive();
            else if(step ==  8) theChassis.driveAuton(5.5, 0.3);
            else if(step ==  9) wheeliebar.wheeliebar_RB_down();
            else if(step == 10) theArm.drive();
            else if(step == 11) theChassis.driveAuton(6.5, 0.2);
            else if(step == 12) clawLatch.closeAuton();
            else if(step == 13) theArm.unfold();
            else if(step == 14) theIntake.openDumpServo();
            else if(step == 15) theIntake.closeDumpServo();
            else if(step == 16) theArm.pickUp();
            else if(step == 17) theIntake.driveCombine(true, false);
            else if(step == 18) theIntake.driveCombine(false, false);
            else if(step == 19) theChassis.driveAuton(6, 0.5);
            else if(step == 20) theChassis.driveAuton(-6, 0.5);
            else if(step == 21) theArm.release();
            else if(step == 22) theIntake.openDumpServo();
            else if(step == 23) holdTime = 2;
            else if(step == 24) theIntake.closeDumpServo();
            else if(step == 25) theArm.drive();
            else if(step == 26) theChassis.driveAuton(1, 0.5);
            else if(step == 27) theArm.pickUp();
          //else if(step == 28) theChassis.markerServo();
          //else if(step == 29) theChassis.markerServo();
            else if(step == 30) theChassis.driveAuton(1, 0.5);
            else if(step == 31) theArm.pickUp();
            else if(step == 32) wheeliebar.wheeliebar_RB_middle();
            else if(step == 33) theChassis.driveAuton(1, 0.5);
        }
    }

    private boolean isBusy() {
        boolean flag;
        theArm.moveOrHoldPosition();
        if(runtime.time() < holdTime)
        {
            flag = true;
        } else if(runtime.time() < timeout) {
            flag = theHardwarePlatter.elevatorDrive.isBusy() || theHardwarePlatter.leftFrontDrive.isBusy() ||
                    theArm.isMoving();
        } else {
            theArm.stop();
            theChassis.rotate(0);
            theElevatorClimb.climberStop();
            flag = false;
        }
        return(flag);
    }
>>>>>>> b8a405f1173ae338a307203681dc5d081ce8e13d
}
