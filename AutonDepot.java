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
    AnalogInput potentiometer;

    Gyroscope imu;
    Gyroscope imu_1;
    HardwarePlatter theHardwarePlatter;
    DcMotor left_back_drive;
    DcMotor combine_drive;
    DcMotor right_back_drive;
    DcMotor arm_drive;
    private DcMotor right_front_drive;
    private DcMotor elevator_climb;
    private DcMotor left_front_drive;

    private Blinker expansion_Hub_1;
    private Blinker expansion_Hub_2;
    private Servo wheeliebarRight;
    private ElapsedTime runtime = new ElapsedTime();
    private Servo wheeliebarLeft;
    private Servo marker_servo;
    private Servo claw_servo;
    private Servo dump_servo;
    ChassisAuton theChassis;

    Arm theArm;
    Intake theIntake;
    ElevatorClimb theElevatorClimb;
    ClawLatch clawLatch;
    Wheeliebar wheeliebar;
    double timeout = 0;
    public void runOpMode() {
        theHardwarePlatter = new HardwarePlatter(hardwareMap);


        theElevatorClimb = new ElevatorClimb(theHardwarePlatter);
        theChassis = new ChassisAuton(theHardwarePlatter);
        clawLatch = new ClawLatch(theHardwarePlatter);
        theArm = new Arm(theHardwarePlatter);
        theIntake = new Intake(theHardwarePlatter);
        wheeliebar = new Wheeliebar(theHardwarePlatter);

        waitForStart();
        
        int nextStep = 1;
        int step = 4;
        runtime.reset();
        timeout = 0;
        while(opModeIsActive() && (step < 15)) {
            telemetry.addData("Encoders_run", theHardwarePlatter.elevatorDrive.getCurrentPosition());
            telemetry.addData("Step", step);
            telemetry.addData("Timer", runtime.time());
            telemetry.addData("Timeout", timeout);
            telemetry.update();  
            
            if((runtime.time() > timeout) && !theHardwarePlatter.leftBackDrive.isBusy() && !theArm.isMoving()){
                step++;
                runtime.reset();
                timeout = 2;
                
                if (step == 1) { timeout = 0.3; clawLatch.open(); }
                else if (step == 2) { 
                    //theElevatorClimb.autonElevatorClimb(0.25, 1); 
                    theElevatorClimb.dropDown();
                    sleep(500); 
                    timeout = 0.5;
                    
                } // up
                else if (step == 3) { theElevatorClimb.climberStop(); }
                else if (step == 4) {
                    theElevatorClimb.climbUp();
                    sleep(3000);
                    timeout = 3;
                }
                else if (step == 5) { theElevatorClimb.climberStop();timeout=0; }
                else if (step == 6) { theChassis.driveAuton(0.25, 0.5); }
                else if (step == 7) { theChassis.driveAuton(-6.5, 0.5); }
                else if (step == 8) wheeliebar.wheeliebar_RB_down();
                else if (step == 9) { theArm.drive(); }
                else if (step == 10) theChassis.driveAuton(6.5, 0.2);
                else if (step == 11) clawLatch.closeAuton();
                else if (step == 12) { theArm.unfold();timeout=3; }
                else if (step == 13) theIntake.openDumpServo();
                else if (step == 14) theIntake.closeDumpServo();
                else if (step == 15) theArm.pickUp();
                else if (step == nextStep++) theIntake.driveCombine(true, false);
                else if (step == nextStep++) theIntake.driveCombine(false, false);
                else if (step == nextStep++) theChassis.driveAuton(6, 0.5);
                else if (step == nextStep++) theChassis.driveAuton(-6, 0.5);
                else if (step == nextStep++) theArm.release();
                else if (step == nextStep++) theIntake.openDumpServo();
                else if (step == nextStep++) timeout = 2000;
                else if (step == nextStep++) theIntake.closeDumpServo();
                else if (step == nextStep++) theArm.drive();
                else if (step == nextStep++) theChassis.driveAuton(1, 0.5);
                else if (step == nextStep++) theArm.pickUp();
                    //else if(step == nextStep++) theChassis.markerServo();
                    //else if(step == nextStep++) theChassis.markerServo();
                else if (step == nextStep++) theChassis.driveAuton(1, 0.5);
                else if (step == nextStep++) theArm.pickUp();
                else if (step == nextStep++) wheeliebar.wheeliebar_RB_middle();
                else if (step == nextStep++) theChassis.driveAuton(1, 0.5);

                sleep(20);
            }


        }
    }
    
    boolean isBusy() {
        return(theHardwarePlatter.elevatorDrive.isBusy() || theHardwarePlatter.leftFrontDrive.isBusy() || 
          theArm.isMoving());
    }
}
