package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class JustTurn extends LinearOpMode {

    HardwarePlatter theHardwarePlatter;
    ChassisAuton theChassis;
    boolean turnCommanded = false;
    ElapsedTime runtime = new ElapsedTime();


    public void runOpMode() {
        theHardwarePlatter = new HardwarePlatter(hardwareMap);
        theChassis = new ChassisAuton(theHardwarePlatter);

        waitForStart();

        waitForStart();
        boolean done = false;
        while (opModeIsActive() && !done) {
            theChassis.rotate(0.5);  //0.5 power for 0.500 sec = 45 Deg
            sleep(500);
            done = true;
        }

        telemetry.addData("Gyro busy", theChassis.isGyroBusy());
        theChassis.display(telemetry);
        telemetry.update();
    }


    // todo: write your code here
}
