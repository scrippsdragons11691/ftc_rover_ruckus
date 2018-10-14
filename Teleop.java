package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp
public class Teleop extends LinearOpMode {
    Chassis theChassis;
    public HardwarePlatter hwPlatter;
    Intake intake;
    public void runOpMode() {
        hwPlatter = new HardwarePlatter(hardwareMap);
        theChassis = new Chassis(hwPlatter);
        intake    = new Intake(hwPlatter);
        telemetry.addData("robot", "initialized");
        telemetry.update();
        
        hwPlatter.dumpServo.setPosition(0.0);

        waitForStart();

        while(opModeIsActive()) {

            double speed = Math.pow(gamepad1.left_stick_y,3) * 0.6;
            double turn  = Math.pow(-gamepad1.left_stick_x,3) *0.6;
            theChassis.drive(speed,turn);

            theChassis.driveCombine(gamepad1.a, gamepad1.b);
            if(gamepad1.y){
                intake.backward();
            } else if(gamepad1.x) {
                intake.forward();
            } else {
                intake.stop();
            }
            double dumpServoPos = hwPlatter.dumpServo.getPosition();
            double newDumpServoPos = dumpServoPos;
            if(gamepad1.left_bumper) {
                newDumpServoPos += 0.2;
            } else if(gamepad1.right_bumper) {
                newDumpServoPos -= 0.2;
            } else {
                newDumpServoPos = 0;
            }
            hwPlatter.dumpServo.setPosition(newDumpServoPos);
            telemetry.addData("dump servo", hwPlatter.dumpServo.getPosition());
            intake.display(telemetry);
            telemetry.update();
        }   
    }
}
