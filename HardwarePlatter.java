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
        //elevatorClimb = (DcMotor)theHardwarePlatter.get("elevator_climb");
        //combineDrive = (DcMotor)theHardwarePlatter.get("combine_drive");
        //clawServo = (Servo)theHardwarePlatter.get("claw_servo");
        armDrive = (DcMotor)hMap.get("arm_drive");
        imu          = hMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);
    }
}
