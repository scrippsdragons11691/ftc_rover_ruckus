package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
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
    public DcMotor armDrive;

    public DcMotor elevatorDrive;
    public Servo clawServo;
    public TouchSensor climberLimitSwUp;
    public TouchSensor climberLimitSwDn;


    public Servo dumpServo;

    public Servo markerServo;

    public DcMotor wheelieBar;

    public AnalogInput wheeliebarPot;

    public ModernRoboticsI2cRangeSensor rangeSensor;


    public HardwarePlatter(HardwareMap hMap) {
        leftFrontDrive = (DcMotor) hMap.get("left_front_drive");
        rightFrontDrive = (DcMotor) hMap.get("right_front_drive");
        armPotentiometer = (AnalogInput) hMap.get("potentiometer");
        leftBackDrive = (DcMotor) hMap.get("left_back_drive");
        rightBackDrive = (DcMotor) hMap.get("right_back_drive");
        armDrive = (DcMotor) hMap.get("arm_drive");
        dumpServo = (Servo) hMap.get("dump_servo");
        elevatorDrive = (DcMotor) hMap.get("elevator_climb");
        combineDrive = (DcMotor) hMap.get("combine_drive");
        climberLimitSwUp = (TouchSensor) hMap.get("climber_limit_sw_up");
        climberLimitSwDn = (TouchSensor) hMap.get("climber_limit_sw_dn");
        clawServo = (Servo) hMap.get("claw_servo");
        markerServo = (Servo) hMap.get("marker_servo");
        wheelieBar = (DcMotor) hMap.get("wheelie_bar");
        wheeliebarPot = (AnalogInput) hMap.get("wheeliebar_pot");
        rangeSensor = hMap.get(ModernRoboticsI2cRangeSensor.class, "range2");
    }
}
