package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Odometry.OdometryGlobalCoordinatePosition;

import java.util.List;

/**
 * This 2019-2020 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Skystone game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@TeleOp(name = "Tensor Distance", group = "Concept")

public class TensorDistance extends LinearOpMode{

    private static final String TFOD_MODEL_ASSET = "newBall.tflite";
    private static final String LABEL_FIRST_ELEMENT = "S";
    //private static final String LABEL_SECOND_ELEMENT = "Skystone";
    double knownDistance = 24;
    double widthPixels;
    //double knownWidth = 6;
    //double focalLength = 575.0;
    double perWidth;
    boolean first = true;
    double heightRatio;
    double imageHeight;
    double objectHeight;
    double objectAngle;
    boolean blockFound = false;
    boolean sideways = true;
    double leftPower = 0;
    double rightPower = 0;
    final double COUNTS_PER_INCH = 306.382978;


    Hardware hardware;
    OdometryGlobalCoordinatePosition globalPositionUpdate;
    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "ATJz83b/////AAABmXLUTP+vOUYCo0NmIag1kv1Jd8vnhGXWh8i8OCr0TDgxzpjmscoRy56Nppi/nBpMJJ/GLnK2wVQQDQGut12bCgdwdWAndSnNDfj8fbpv3KiCrm+mgV5YqzOOAaih00CkGf2EYSov9LkE/s6yjK5965lkpFvvLL5TtV7fWcbW6Ez/aSTYnIbvS2Q3yJOr7ycLqgOOqiRRpmbb26WL94TZr2IzG6mZacM2IbFHMKIyw4vKJv3gTTMqXAt6es24VYu5Jt3Rmr0t4jbGrb2hpo1JwtNVr4+F8FVLuMmkNIcCrLqrVQ0bl3KMM/kYq8Hav7UR1yQrbwjdIVAm9gv7WXSYYj0rFv5wncI7K/rzt7K4FRJQ";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    WebcamName webcam;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.

        hardware = new Hardware();
        hardware.initHardware(hardwareMap);

        globalPositionUpdate = new OdometryGlobalCoordinatePosition(hardware.verticalLeft, hardware.verticalRight, hardware.horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        webcam = hardwareMap.get(WebcamName.class, "Webcam");

        initVuforia();

        initTfod();

        //if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
        // initTfod();
        //} else {
        //    telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        //}

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    Recognition recognition = null;
                    float confidenceLevel = 0;
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());

                        for (int i = 0; i < updatedRecognitions.size(); i++)
                        {
                            Recognition currRecognition = updatedRecognitions.get(i);
                            if (currRecognition.getConfidence() > confidenceLevel)
                            {
                                confidenceLevel=currRecognition.getConfidence();
                                recognition=currRecognition;
                            }

                        }

                        // step through the list of recognitions and display boundary info.
                        //int i = 0;
                        if (recognition != null) {
                            //telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            //telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            //        recognition.getLeft(), recognition.getTop());
                            //telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            //        recognition.getRight(), recognition.getBottom());
                            if (first)
                            {
                                sleep(2000);
                                first=false;
                            }
                            imageHeight = (double) recognition.getImageHeight();
                            objectHeight = (double) recognition.getHeight();
                            heightRatio = objectHeight / imageHeight;
                            objectAngle = recognition.estimateAngleToObject(AngleUnit.DEGREES);


                            //telemetry.addData("Left Power", leftPower);
                            //telemetry.addData("Right Power", rightPower);
                            //telemetry.addData("Height Ratio", heightRatio);
                            //telemetry.addData("Estimated Angle", recognition.estimateAngleToObject(AngleUnit.DEGREES));

                            // Camera lines itself up with the block with a +/- 3 leeway
                            if (Math.abs(recognition.estimateAngleToObject(AngleUnit.DEGREES)) > 3 && sideways) {
                                //objectAngle = recognition.estimateAngleToObject(AngleUnit.DEGREES);
                                leftPower = 0.12;
                                rightPower = 0.12;
                                if (objectAngle > 0)
                                {
                                    leftPower*=-1;
                                }
                                else
                                {
                                    rightPower*=-1;
                                }
                                hardware.left_back.setPower(leftPower);
                                hardware.left_front.setPower(rightPower);
                                hardware.right_back.setPower(rightPower);
                                hardware.right_front.setPower(leftPower);
                                telemetry.addData("Left Power", leftPower);
                                telemetry.addData("Right Power", rightPower);
                                telemetry.addData("Height Ratio", heightRatio);
                                telemetry.addData("Estimated Angle", recognition.estimateAngleToObject(AngleUnit.DEGREES));
                                telemetry.update();
                            } else if (sideways) {
                                hardware.left_back.setPower(0);
                                hardware.left_front.setPower(0);
                                hardware.right_back.setPower(0);
                                hardware.right_front.setPower(0);
                                sleep(1000);
                                sideways = false;
                            }

                            if (sideways == false && blockFound == false)
                            {
                                widthPixels = recognition.getWidth();
                                //perWidth = recognition.getRight()-recognition.getLeft();
                                //focalLength = (perWidth * knownDistance) / knownWidth;
                                //telemetry.addData("Focal Length", focalLength);
                                //telemetry.update();
                                //sleep(2000);
                                OdometryHelper.goToPosition(globalPositionUpdate, hardware, this, globalPositionUpdate.returnXCoordinate(), globalPositionUpdate.returnYCoordinate()+2*COUNTS_PER_INCH, 0.3, 0.5 * COUNTS_PER_INCH, 0);
                                perWidth = recognition.getWidth();
                                telemetry.addData("Pixel Width", perWidth);
                                telemetry.addData("Distance", Distance(perWidth,knownDistance,perWidth));
                                telemetry.addData("X", globalPositionUpdate.returnXCoordinate());
                                telemetry.addData("Y", globalPositionUpdate.returnYCoordinate());
                                telemetry.update();
                                sleep(2000);
                                OdometryHelper.goToPosition(globalPositionUpdate, hardware, this, globalPositionUpdate.returnXCoordinate(), globalPositionUpdate.returnYCoordinate()+(Distance(widthPixels,knownDistance,perWidth)-8)*COUNTS_PER_INCH, 0.3, 0.5 * COUNTS_PER_INCH, 0);
                                blockFound = true;
                                perWidth = recognition.getWidth();
                                telemetry.addData("Distance From Origin", 24-globalPositionUpdate.returnYCoordinate()/COUNTS_PER_INCH);
                                telemetry.update();
                            }

                            if (blockFound) {
                                sleep(10000);
                                OdometryHelper.goToPosition(globalPositionUpdate, hardware, this, 0, 0, 0.3, 0.5 * COUNTS_PER_INCH, 0);
                                break;
                            }
                        }
                    }
                    if (tfod.getRecognitions() == null)
                    {
                        telemetry.addData("Check1", "check1");
                        leftPower = 0;
                        rightPower = 0;
                        hardware.left_back.setPower(leftPower);
                        hardware.left_front.setPower(leftPower);
                        hardware.right_back.setPower(rightPower);
                        hardware.right_front.setPower(rightPower);
                        telemetry.update();
                    }
                    telemetry.update();
                    if (blockFound)
                        break;
                }
                if (tfod.getRecognitions() == null)
                {
                    telemetry.addData("Check", "check");
                    leftPower = 0;
                    rightPower = 0;
                    hardware.left_back.setPower(leftPower);
                    hardware.left_front.setPower(leftPower);
                    hardware.right_back.setPower(rightPower);
                    hardware.right_front.setPower(rightPower);
                    telemetry.update();
                }
                if (blockFound)
                    break;
            }
        }
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = webcam;
        //parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.75;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        //tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET,LABEL_FIRST_ELEMENT);
    }

    private double Distance(double origPixels, double knownDistance, double pixels)
    {
        return (origPixels*knownDistance)/pixels;
    }


}