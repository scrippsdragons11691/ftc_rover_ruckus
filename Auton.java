package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class Auton extends OpMode {

    HardwarePlatter theHardwarePlatter;
    ElevatorClimb theElevatorClimb;
    Arm theArm;
    Sampler theSampler;
    ChassisAuton theChassis;
    private ElapsedTime runtime = new ElapsedTime();
    ClawLatch theClaw;
    Intake intake;
    int step;
    private GoldAlignDetector detector;
    double timeout = 0;

    public Auton() {
        theHardwarePlatter = new HardwarePlatter(hardwareMap);

        theSampler = new Sampler(theHardwarePlatter);
        theElevatorClimb = new ElevatorClimb(theHardwarePlatter);
        theChassis = new ChassisAuton(theHardwarePlatter);
        theClaw = new ClawLatch(theHardwarePlatter);
        theArm      = new Arm(theHardwarePlatter);
        intake = new Intake(theHardwarePlatter);

        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        // Optional Tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;

        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.enable();
    }

    public void init() {
        telemetry.addData("Robot", "Initialized");
        telemetry.update();
        step = 1;
    }

    public void loop() {
        if (step < 6) {
            if (!theChassis.isDriveBusy() && !theArm.isMoving() || (runtime.time() > timeout))
            {
                step++;
                if (step == 1) {
                    theElevatorClimb.dropDown();
                    timeout = 2000;
                } else if (step == 2) {
                        theChassis.driveAuton(6.5);
                    timeout = 2000;
                } else if (step == 3) {
                    theArm.drive();
                    timeout = 2000;
                } else if (step == 4) {
                    theChassis.driveAuton(-6.5);
                    timeout = 2000;
                } else if (step == 5) {
                    theArm.unfold();
                    timeout = 2000;
                } else if (step == 6) {
                    intake.openDumpServo();
                    theArm.pickUp();
                    timeout = 2000;
                }
                runtime.reset();
            }
        }
        telemetry.addData("IsAligned" , detector.getAligned()); // Is the bot aligned with the gold mineral
        telemetry.addData("X Pos" , detector.getXPosition()); // Gold X pos.
    }

    public void stop() {
        detector.disable();
    }
}