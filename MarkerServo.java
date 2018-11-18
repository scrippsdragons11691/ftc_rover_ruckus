package org.firstinspires.ftc.teamcode;


public class MarkerServo {


        static final double INITIALIZE_POSITION              = 0.5;
        static final double MARKER_DOWN_POSITION      = 1;
        static final double MARKER_UP_POSITION    = 0.5;
        
    HardwarePlatter theHardwarePlatter;
    
    public MarkerServo(HardwarePlatter hwPlatter) {
        theHardwarePlatter = hwPlatter;
    }
    
    void markerServo_initialized(){
        theHardwarePlatter.markerServo.setPosition(INITIALIZE_POSITION);
    }
    
    void markerServo_down(){
        theHardwarePlatter.markerServo.setPosition(MARKER_DOWN_POSITION);
    }
    
    void markerServo_up(){
        theHardwarePlatter.markerServo.setPosition(MARKER_UP_POSITION);
    }
    
}
