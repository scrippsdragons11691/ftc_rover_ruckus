package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

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

        boolean done = false;
        theChassis.rotateAuton(14, 0.5);  //0.5 power for 0.500 sec = 45 Deg
        // 10 inch 65 deg
        while(opModeIsActive() && !done) {
            
            //sleep(500);
            done = !theHardwarePlatter.leftBackDrive.isBusy();
            telemetry.addData("Counts", theHardwarePlatter.leftBackDrive.getCurrentPosition());
            theChassis.display(telemetry);
            telemetry.update();        }
            

    }


    // todo: write your code here
}
