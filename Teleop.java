package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp
public class Teleop extends LinearOpMode {

    Chassis theChassis;
    public HardwarePlatter theHardwarePlatter;
    Arm theArm;
    Intake theIntake;
    ElevatorClimb theElevatorClimb;
    ClawLatch theClawLatch;
    Wheeliebar theWheeliebar;
    MarkerServo theMarkerServo;
    
    public void runOpMode() {
        theHardwarePlatter = new HardwarePlatter(hardwareMap);
        theChassis = new Chassis(theHardwarePlatter);
        theArm = new Arm(theHardwarePlatter);
        theIntake = new Intake(theHardwarePlatter);
        theElevatorClimb = new ElevatorClimb(theHardwarePlatter);
        theClawLatch = new ClawLatch(theHardwarePlatter);
        theWheeliebar = new Wheeliebar(theHardwarePlatter);
        theMarkerServo = new MarkerServo(theHardwarePlatter);
        
        boolean elevatorClimbset = false;

        telemetry.addData("robot", "initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {


            
        //Drive Control (Split-Arcade)
            
            double boost;  // Slow down drive for better control
            if(gamepad1.left_bumper)
                boost = 0.5;
            else
                boost = 1.0;
            double speed = Math.pow(-gamepad1.left_stick_y, 1) * boost;
            double turn  = Math.pow(gamepad1.right_stick_x, 1) * boost;
            theChassis.drive(speed, turn);
            

        //Drive Control (Tank Drive)
        /*    double boost;  // Slow down drive for better control
            if(gamepad1.left_bumper)
                boost = 0.5;
            else
                boost = 1.0;
            double leftDrive = Math.pow(gamepad1.left_stick_y, 3) * boost;
            double rightDrive  = Math.pow(gamepad1.right_stick_y, 3) * boost;
            theChassis.driveTank(leftDrive, rightDrive);
            */
            theIntake.driveCombine(gamepad2.dpad_up, gamepad2.dpad_down);

        //Arm Control
            double boostArm;                
            if(gamepad2.left_trigger > 0.1)
                boostArm = 0.5;
            else
                boostArm = 1.0;
            if(gamepad2.right_stick_y < -0.1){
                theArm.move(-gamepad2.right_stick_y * boostArm);
            } else if(gamepad2.right_stick_y > 0.1) {
                theArm.move(-gamepad2.right_stick_y * boostArm);
            } else if(gamepad2.right_stick_button) {
                theArm.middle();
            } else if(gamepad2.x) {
                theArm.release();
            } else if(gamepad2.b) {
                theArm.drive();
            } else if(gamepad2.a) {
                theArm.pickUp();
            } else if(gamepad2.y){
                theArm.unfold();
            }  else {
                theArm.moveOrHoldPosition();
            }
        // Auto Claw Control            
            if(gamepad1.a) {
                theClawLatch.autoOpen();
            } else if(gamepad1.b){
                theClawLatch.autoClose();

        // Marker Servo Control
            }else if(gamepad1.y){
                theMarkerServo.markerServo_down();
            } else if(gamepad1.x){
                theMarkerServo.markerServo_up();
            }
        // Claw latch control
            if(gamepad1.dpad_left)
                theClawLatch.open();
            else if(gamepad1.dpad_right)
                theClawLatch.close();

        // Dump Servo Control
            if(gamepad2.left_bumper) {
                theIntake.openDumpServo();
            }else {
                theIntake.closeDumpServo();
            }
            
        // Elevator control
            if(gamepad1.dpad_up){
                theElevatorClimb.climbUp();
            }
            else{
                theElevatorClimb.climberStop();
            }
            if(gamepad1.dpad_down){
                theElevatorClimb.dropDown();
            }
            else{
                theElevatorClimb.climberStop();
            }
        // Auto Up Control for the Elevator
        
//            if(!theHardwarePlatter.climberLimitSw .isPressed())
//                {
//                if(gamepad1.left_trigger > 0.1 || elevatorClimbset){
//                    theElevatorClimb.dropDownAuto();
//                    elevatorClimbset = true;
//                    }
//                }
//            else 
//                theElevatorClimb.climberStop();
//                elevatorClimbset = false;
                
           if (gamepad1.left_trigger > 0.1 ) elevatorClimbset = true;
           if (elevatorClimbset)
               if (!theHardwarePlatter.climberLimitSw .isPressed())
                  theElevatorClimb.dropDownAuto();
                  else elevatorClimbset = false;

        // Wheelie Bar Control
            if(gamepad1.right_bumper) {
                theWheeliebar.wheeliebar_RB_down();
            }
            else{
                theWheeliebar.wheeliebar_RB_middle();
            }
            
        // Telemetry
            telemetry.addData("dump servo", theHardwarePlatter.dumpServo.getPosition());
            telemetry.addData("arm", gamepad2.right_stick_y);
            theArm.display(telemetry);
            theClawLatch.display(telemetry);
            telemetry.update();
        }
    }
}
