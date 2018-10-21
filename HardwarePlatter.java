package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.AnalogInput;

public class HardwarePlatter {

    public DcMotor leftFrontDrive;
    public DcMotor rightFrontDrive;
    public DcMotor leftBackDrive;
    public DcMotor rightBackDrive;
    public DcMotor wheelDrives[] = { leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive };

    public DcMotor combineDrive;

    public AnalogInput armPotentiometer;
    public DcMotor     armDrive;

    public DcMotor elevatorDrive;
    public Servo   clawServo;

    public Servo dumpServo;

    public Servo wheeliebarServo;

    public BNO055IMU imu;

    public HardwarePlatter(HardwareMap hMap){
        leftFrontDrive = (DcMotor)hMap.get("left_front_drive");
        rightFrontDrive = (DcMotor)hMap.get("right_front_drive");
        armPotentiometer = (AnalogInput)hMap.get("potentiometer");
        leftBackDrive = (DcMotor)hMap.get("left_back_drive");
        rightBackDrive = (DcMotor)hMap.get("right_back_drive");
        armDrive = (DcMotor)hMap.get("arm_drive");
        dumpServo = (Servo)hMap.get("dump_servo");
        elevatorDrive = (DcMotor)hMap.get("elevator_climb");
        combineDrive = (DcMotor)hMap.get("combine_drive");
        clawServo = (Servo)hMap.get("claw_servo");
        imu          = hMap.get(BNO055IMU.class, "imu");
    }
}
