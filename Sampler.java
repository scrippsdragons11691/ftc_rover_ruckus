package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class Sampler {

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AflGIy7/////AAABmZXaLZsv6UzSvUDR0RWPvVUzj8tTQmJLenJJdIMfK4GIxwpSxXn0AI1YWVh5fXHcH1Rb8s7xjynA0e7+eNrn0x/yYbU3O09RIgqZLPCcq3eeGMFoCn5IdADLdWMXG6CGCpgE+o5OVdbQMJO2Z0a4vTqr5rVymQk88lt/dlq1q4Fhbr/1iGMxecXvYx4D1SJ+d3Eqdy+ss9zO45pqkFR1nhN01vTN1LgB1ZKPfNoQBl+F/Z1gfg5BFgqHu5y0ziqu9cx1mVuBr7/ZaMPAtwqh8BzOTt2YlH641a12GQ7K3OtqsMU9btnwjaz6fBjWz8JeZ/TQmO+kprKi0gHT2PYRCX6UASN4Cn6WKflYW8/eaBGl ";
    String goldPosition;
    HardwareMap theHardwareMap;
    int goldMineralPosition = 0;
    int numMinerals = 0;
    private VuforiaLocalizer vuforia;       //the variable we will use to store our instance of the Vuforia localization engine.
    private TFObjectDetector tfod;          //the variable we will use to store our instance of the Tensor Flow Object Detection engine.

    public Sampler(HardwareMap hwMap) {
        theHardwareMap = hwMap;
    }

    void activate() {
        /** Activate Tensor Flow Object Detection. */
        if (tfod != null) {
            tfod.activate();
        }
    }

    void deactivate() {
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    int sampleMinerals() {


        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                //telemetry.addData("# Object Detected", updatedRecognitions.size());
                numMinerals = updatedRecognitions.size();

                if (updatedRecognitions.size() == 2) {
                    float firstLeft = updatedRecognitions.get(0).getLeft();
                    float secondLeft = updatedRecognitions.get(1).getLeft();
                    int first_index = 0, second_index = 1;
                    if (firstLeft > secondLeft) {
                        first_index = 1;
                        second_index = 0;
                    }
                    if (updatedRecognitions.get(first_index).getLabel().equals(LABEL_GOLD_MINERAL))
                    {
                        goldPosition = "left";
                        goldMineralPosition = -1;
                    }
                    else if (updatedRecognitions.get(second_index).getLabel().equals(LABEL_GOLD_MINERAL)) {
                        goldPosition = "center";
                        goldMineralPosition = 0;
                    }
                    else {
                        goldPosition = "right";
                        goldMineralPosition = 1;
                    }

                }
            }

        }
        return(goldMineralPosition);
    }


    /**
     * Initialize the Vuforia localization engine.
     */
    void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    void initTfod() {
        int tfodMonitorViewId = theHardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", theHardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    void display(Telemetry telemetry) {
        telemetry.addData("Gold Mineral Position", goldPosition);
        telemetry.addData("Gold Mineral Position", goldMineralPosition);
        telemetry.addData("Minerals detected:", numMinerals);
    }
}
