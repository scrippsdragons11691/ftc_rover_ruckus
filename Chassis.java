package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class Chassis {
    
    HardwarePlatter theHardwarePlatter;
    
    public Chassis(HardwarePlatter hwPlatter){
            theHardwarePlatter = hwPlatter;
            // Most robots need the motor on one side to be reversed to drive forward
            // Reverse the motor that runs backwards when connected directly to the battery
            theHardwarePlatter.leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            theHardwarePlatter.rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            theHardwarePlatter.leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
            theHardwarePlatter.rightBackDrive.setDirection(DcMotor.Direction.REVERSE);            
    }
    
    public void drive(double speed, double turn){
            
        double leftFrontPower = Range.clip(speed + turn, -1.0, 1.0);
        double rightFrontPower = Range.clip(speed - turn, -1.0, 1.0);
        double leftBackPower = Range.clip(speed + turn, -1.0, 1.0);
        double rightBackPower = Range.clip(speed - turn, -1.0, 1.0);
                
        theHardwarePlatter.leftFrontDrive.setPower(leftFrontPower);
        theHardwarePlatter.rightFrontDrive.setPower(rightFrontPower);
        theHardwarePlatter.leftBackDrive.setPower(leftBackPower);
        theHardwarePlatter.rightBackDrive.setPower(rightBackPower);
    }
    public void driveTank(double leftDrive, double rightDrive){
            
        double leftFrontPower = Range.clip(leftDrive, -1.0, 1.0);
        double rightFrontPower = Range.clip(rightDrive, -1.0, 1.0);
        double leftBackPower = Range.clip(leftDrive, -1.0, 1.0);
        double rightBackPower = Range.clip(rightDrive, -1.0, 1.0);
                
        theHardwarePlatter.leftFrontDrive.setPower(leftFrontPower);
        theHardwarePlatter.rightFrontDrive.setPower(rightFrontPower);
        theHardwarePlatter.leftBackDrive.setPower(leftBackPower);
        theHardwarePlatter.rightBackDrive.setPower(rightBackPower);
    }
    
    void display(Telemetry telemetry) {
        telemetry.addData("LF", theHardwarePlatter.leftFrontDrive.getPower());
    }
    
    
}
