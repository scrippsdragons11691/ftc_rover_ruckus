package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp
public class Teleop extends LinearOpMode {
    Chassis theChassis;
    public HardwarePlatter hwPlatter;

    public void runOpMode() {
        hwPlatter = new HardwarePlatter(hardwareMap);
        theChassis = new Chassis(hwPlatter);
        telemetry.addData("robot", "initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {

            double speed = Math.pow(gamepad1.left_stick_y,3) * 0.6;
            double turn  = Math.pow(-gamepad1.left_stick_x,3) *0.6;
            theChassis.drive(speed,turn);

            theChassis.driveCombine(gamepad1.a, gamepad1.b);
        }
    }
}