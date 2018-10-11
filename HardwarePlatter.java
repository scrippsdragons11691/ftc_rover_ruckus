package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class HardwarePlatter {
    public DcMotor leftFrontDrive;
    public DcMotor rightFrontDrive;
    public DcMotor leftBackDrive;
    public DcMotor rightBackDrive;
    public DcMotor combineDrive;
    
    //public DcMotor elevatorClimb;
    public Servo clawServo;
    
    public HardwarePlatter(HardwareMap hwPlatter){
        leftFrontDrive = (DcMotor)hwPlatter.get("left_front_drive");
        rightFrontDrive = (DcMotor)hwPlatter.get("right_front_drive");
        leftBackDrive = (DcMotor)hwPlatter.get("left_back_drive");
        rightBackDrive = (DcMotor)hwPlatter.get("right_back_drive");
        //elevatorClimb = (DcMotor)hwPlatter.get("elevator_climb");
        combineDrive = (DcMotor)hwPlatter.get("combine_drive");
        clawServo = (Servo)hwPlatter.get("claw_servo");
    }
}
