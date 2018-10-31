package org.firstinspires.ftc.teamcode;


public class Wheeliebar {


    static final double INITIALIZERIGHT_POSITION = 0;
    static final double WHEELIEBARRIGHT_RB_DOWN_POSITION = 0.6;
    static final double WHEELIEBARRIGHT_RB_MIDDLE_POSITION = 0.3;
    static final double INITIALIZELEFT_POSITION = 1;
    static final double WHEELIEBARLEFT_RB_DOWN_POSITION = 0.4;
    static final double WHEELIEBARLEFT_RB_MIDDLE_POSITION = 0.7;

    HardwarePlatter theHardwarePlatter;

    public Wheeliebar(HardwarePlatter hwPlatter) {
        theHardwarePlatter = hwPlatter;
    }

    void wheeliebar_initialized() {
        theHardwarePlatter.wheeliebarRightServo.setPosition(INITIALIZERIGHT_POSITION);
        theHardwarePlatter.wheeliebarLeftServo.setPosition(INITIALIZELEFT_POSITION);
    }

    void wheeliebar_RB_down() {
        theHardwarePlatter.wheeliebarRightServo.setPosition(WHEELIEBARRIGHT_RB_DOWN_POSITION);
        theHardwarePlatter.wheeliebarLeftServo.setPosition(WHEELIEBARLEFT_RB_DOWN_POSITION);
    }

    void wheeliebar_RB_middle() {
        theHardwarePlatter.wheeliebarRightServo.setPosition(WHEELIEBARRIGHT_RB_MIDDLE_POSITION);
        theHardwarePlatter.wheeliebarLeftServo.setPosition(WHEELIEBARLEFT_RB_MIDDLE_POSITION);
    }

}
