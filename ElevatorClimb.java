
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;


public class ElevatorClimb {
    static final double     COUNTS_PER_MOTOR_REV    = 1680 ;    // eg: 60 Gear Neverest Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 0.5 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    
    HardwarePlatter theHardwarePlatter;
    
    public ElevatorClimb (HardwarePlatter hwPlatter){
            theHardwarePlatter = hwPlatter;
    }
    
    void climbUpAuton(){                                //elevator runs down to release the latch
        theHardwarePlatter.elevatorDrive.setPower(0.75);
    }
    
    void climbDownAuton(){                                //elevator runs down to release the latch
        if (!theHardwarePlatter.climberLimitSwDn.isPressed())
            theHardwarePlatter.elevatorDrive.setPower(-0.75);
        else 
            theHardwarePlatter.elevatorDrive.setPower(0);
    }   
    void climbUp() {                                    //elevator runs down
        theHardwarePlatter.elevatorDrive.setPower(1);
    }
    void dropDown() {                                   //elevator goes up
        theHardwarePlatter.elevatorDrive.setPower(-1);
    }    
    void dropDownAuto() {                                   //elevator goes up
        if (!theHardwarePlatter.climberLimitSwUp.isPressed())
            theHardwarePlatter.elevatorDrive.setPower(1);
        else 
            climberStop(); 
    }
    void climberStop() {                                //stops elevator motor
    if(theHardwarePlatter.climberLimitSwDn.isPressed())
        theHardwarePlatter.elevatorDrive.setPower(0);
    }
    
         /*       if (!theHardwarePlatter.climberLimitSwUp.isPressed())
                  theElevatorClimb.dropDownAuto();
            if(theHardwarePlatter.climberLimitSwDn.isPressed()){
                theElevatorClimb.climberStop();
            } */
    void autonElevatorClimb(double distInches,double speed){
        double elevInches = distInches;
       // double timeoutS;
        int elevTarget;
        //theHardwarePlatter.elevatorDrive.setDirection(DcMotor.Direction.FORWARD);
        theHardwarePlatter.elevatorDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       
        theHardwarePlatter.elevatorDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // Determine new target position, and pass to motor controller
        elevTarget = theHardwarePlatter.elevatorDrive.getCurrentPosition() + (int)(elevInches * COUNTS_PER_INCH);
        
        theHardwarePlatter.elevatorDrive.setTargetPosition(elevTarget);
       
        // Turn On RUN_TO_POSITION
        theHardwarePlatter.elevatorDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      
        // reset the timeout time and start motion.
        //runtime.reset();
        theHardwarePlatter.elevatorDrive.setPower(Math.abs(speed));
  
         //while(theHardwarePlatter.elevatorDrive.isBusy())
           // {
            //telemetry.addLine("Encoders_run");
            //telemetry.update();
           // }
           }
}
