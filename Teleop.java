package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Teleop extends LinearOpMode {

    public HardwarePlatter theHardwarePlatter;
    Chassis theChassis;
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

        boolean elevatorClimbsetUp = false;
        boolean elevatorClimbsetDn = false;

        telemetry.addData("robot", "initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {


            //Drive Control
            double boost;  // Slow down drive for better control
            if (gamepad1.left_bumper)
                boost = 0.5;
            else
                boost = 1.0;
            double speed = Math.pow(-gamepad1.left_stick_y, 3) * boost;
            double turn = Math.pow(gamepad1.right_stick_x, 3) * boost;
            theChassis.drive(speed, turn);

            theIntake.driveCombine(gamepad2.dpad_up, gamepad2.dpad_down);

            //Arm Control
            double boostArm;
            if (gamepad2.left_trigger > 0.1)
                boostArm = 0.5;
            else
                boostArm = 1.0;
            if (gamepad2.right_stick_y < -0.1) {
                theArm.move(-gamepad2.right_stick_y * boostArm);
            } else if (gamepad2.right_stick_y > 0.1) {
                theArm.move(-gamepad2.right_stick_y * boostArm);
            } else if (gamepad2.right_stick_button) {
                theArm.middle();
            } else if (gamepad2.x) {
                theArm.release();
            } else if (gamepad2.b) {
                theArm.drive();
            } else if (gamepad2.a) {
                theArm.pickUp();
            } else if (gamepad2.y) {
                theArm.unfold();
            } else {
                theArm.moveOrHoldPosition();
            }
            // Auto Claw Control
            if (gamepad1.a) {
                theClawLatch.autoOpen();
            } else if (gamepad1.b) {
                theClawLatch.autoClose();

                // Marker Servo Control
            } else if (gamepad1.y) {
                theMarkerServo.markerServo_down();
            } else if (gamepad1.x) {
                theMarkerServo.markerServo_up();
            }
            // Claw latch control
            if (gamepad1.dpad_left)
                theClawLatch.close();
            else if (gamepad1.dpad_right)
                theClawLatch.open();

            // Dump Servo Control
            if (gamepad2.left_bumper) {
                theIntake.openDumpServo();
            } else {
                theIntake.closeDumpServo();
            }

            // Elevator control

            // Auto Up Control for the Elevator

            if (gamepad1.left_trigger > 0.1) elevatorClimbsetUp = true;
            if (gamepad1.right_trigger > 0.1) elevatorClimbsetDn = true;

            if (gamepad1.dpad_up) {
                //if (!theHardwarePlatter.climberLimitSwUp.isPressed()) {
                theElevatorClimb.climbUp();
                elevatorClimbsetUp = false;
                elevatorClimbsetDn = false;
                //}
            } else if (gamepad1.dpad_down) {
                //if (!theHardwarePlatter.climberLimitSwDn.isPressed()){
                theElevatorClimb.dropDown();
                elevatorClimbsetDn = false;
                elevatorClimbsetUp = false;
                //}
            } else if (elevatorClimbsetDn) {
                if (!theHardwarePlatter.climberLimitSwDn.isPressed()) {
                    theElevatorClimb.dropDown();
                } else {
                    theElevatorClimb.climberStop();
                    elevatorClimbsetDn = false;
                    elevatorClimbsetUp = false;
                }
            } else if (elevatorClimbsetUp) {
                if (!theHardwarePlatter.climberLimitSwUp.isPressed()) {
                    theElevatorClimb.climbUp();
                } else {
                    elevatorClimbsetUp = false;
                    elevatorClimbsetDn = false;
                    theElevatorClimb.climberStop();

                }
            } else if (elevatorClimbsetDn) {
                if (!theHardwarePlatter.climberLimitSwDn.isPressed()) {
                    theElevatorClimb.dropDown();
                } else {
                    theElevatorClimb.climberStop();
                    elevatorClimbsetDn = false;
                    elevatorClimbsetUp = false;
                }
            } else {
                theElevatorClimb.climberStop();
            }

            // Wheelie Bar Control
            if (gamepad1.right_bumper) {
                theWheeliebar.down();
            } else if (gamepad1.left_bumper) {
                theWheeliebar.middle();
            } else theWheeliebar.moveOrHoldPosition();

            // Telemetry
            telemetry.addData("Elevator Drive", theHardwarePlatter.elevatorDrive.getPower());
            telemetry.addData("dump servo", theHardwarePlatter.dumpServo.getPosition());
            telemetry.addData("arm", gamepad2.right_stick_y);
            theArm.display(telemetry);
            theWheeliebar.display(telemetry);
            theClawLatch.display(telemetry);
            telemetry.update();
        }
    }
}
