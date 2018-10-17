package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp
public class Teleop extends LinearOpMode {

    Chassis theChassis;
    public HardwarePlatter theHardwarePlatter;
    Arm theArm;
    Intake theIntake;

    public void runOpMode() {
        theHardwarePlatter = new HardwarePlatter(hardwareMap);
        theChassis = new Chassis(theHardwarePlatter);
        theArm = new Arm(theHardwarePlatter);
        theIntake = new Intake(theHardwarePlatter);

        theHardwarePlatter.dumpServo.setPosition(0.0);

        telemetry.addData("robot", "initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {

            double speed = Math.pow(gamepad1.left_stick_y, 3) * 0.6;
            double turn  = Math.pow(-gamepad1.left_stick_x, 3) * 0.6;
            theChassis.drive(speed, turn);

            theIntake.driveCombine(gamepad1.dpad_left, gamepad1.dpad_right);

            if(gamepad1.dpad_up){
                theArm.backward();
            } else if(gamepad1.dpad_down) {
                theArm.forward();
            } else if(gamepad1.x) {
                theArm.release();
            } else if(gamepad1.b) {
                theArm.drive();
            } else if(gamepad1.a) {
                theArm.pickUp();
            } else if(gamepad1.y){
                theArm.unfold();
            } else {
                theArm.moveOrHoldPosition();
            }

            if(gamepad1.left_bumper) {
                theIntake.openDumpServo();
            } else if(gamepad1.right_bumper) {
                theIntake.closeDumpServo();
            }

            telemetry.addData("dump servo", theHardwarePlatter.dumpServo.getPosition());
            theArm.display(telemetry);
            telemetry.update();
        }
    }
}