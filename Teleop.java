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
    
    public void runOpMode() {
        theHardwarePlatter = new HardwarePlatter(hardwareMap);
        theChassis = new Chassis(theHardwarePlatter);
        theArm = new Arm(theHardwarePlatter);
        theIntake = new Intake(theHardwarePlatter);
        theElevatorClimb = new ElevatorClimb(theHardwarePlatter);
        theClawLatch = new ClawLatch(theHardwarePlatter);


        telemetry.addData("robot", "initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {

            double speed = Math.pow(-gamepad1.left_stick_y, 3) ;
            double turn  = Math.pow(gamepad1.right_stick_x, 3) ;
            theChassis.drive(speed, turn);

            theIntake.driveCombine(gamepad2.dpad_up, gamepad2.dpad_down);

            if(gamepad2.right_stick_y < -0.1){
                theArm.move(-gamepad2.right_stick_y);
            } else if(gamepad2.right_stick_y > 0.1) {
                theArm.move(-gamepad2.right_stick_y);
            } else if(gamepad2.x) {
                theArm.release();
            } else if(gamepad2.b) {
                theArm.drive();
            } else if(gamepad2.a) {
                theArm.pickUp();
            } else if(gamepad2.y){
                theArm.unfold();
            } else {
                theArm.moveOrHoldPosition();
            }

            if(gamepad2.left_bumper) {
                theIntake.openDumpServo();
            } else if(gamepad2.right_bumper) {
                theIntake.closeDumpServo();
            }

            if(gamepad1.dpad_up)
                theElevatorClimb.climbUp();
            else if(gamepad1.dpad_down)
                theElevatorClimb.dropDown();

            if(gamepad2.dpad_left)
                theClawLatch.open();
            else if(gamepad2.dpad_right)
                theClawLatch.close();

            telemetry.addData("dump servo", theHardwarePlatter.dumpServo.getPosition());
            telemetry.addData("arm", gamepad2.right_stick_y);
            theArm.display(telemetry);
            telemetry.update();
        }
    }
}
