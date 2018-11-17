package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.TouchSensor;
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
    public TouchSensor  climberLimitSwUp;
    public TouchSensor  climberLimitSwDn;


    public Servo dumpServo;

    public Servo wheeliebarLeftServo;
    public Servo wheeliebarRightServo;
    
    public BNO055IMU imu;
    
    public Servo markerServo;
    
    public DcMotor wheelieBar;
    
    public AnalogInput wheeliebarPot;
    
    
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
        climberLimitSwUp = (TouchSensor)hMap.get("climber_limit_sw_up");
        climberLimitSwDn = (TouchSensor)hMap.get("climber_limit_sw_dn");
        clawServo = (Servo)hMap.get("claw_servo");
        //wheeliebarRightServo = (Servo)hMap.get("wheeliebarRight");
        //wheeliebarLeftServo = (Servo)hMap.get("wheeliebarLeft");
        imu          = hMap.get(BNO055IMU.class, "imu");
        markerServo = (Servo)hMap.get("marker_servo");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);
        wheelieBar = (DcMotor)hMap.get("wheelie_bar");
        wheeliebarPot = (AnalogInput)hMap.get("wheeliebar_pot");
        
    }
}
