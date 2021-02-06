package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.Odometry.OdometryGlobalCoordinatePosition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

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
@TeleOp(name = "TensorFlow", group = "Concept")

public class TensorDetection extends LinearOpMode{

    private static final String TFOD_MODEL_ASSET = "newBall.tflite";
    private static final String LABEL_FIRST_ELEMENT = "S";
    //private static final String LABEL_SECOND_ELEMENT = "Skystone";
    int INCREMENT = 6;
    int y_increment = 8;
    int count = 0;
    double knownDistance = 24;
    double widthPixels;
    double perWidth;
    boolean target = false;
    double heightRatio;
    double imageHeight;
    double objectHeight;
    double objectAngle;
    //double targetRatio = 0.5;
    double targetRatio = 0.5;
    boolean blockFound = false;
    boolean sideways = true;
    double leftPower = 0;
    double rightPower = 0;
    boolean first = true;
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
                            if (Math.abs(recognition.estimateAngleToObject(AngleUnit.DEGREES)) > 5 && sideways) {
                                //objectAngle = recognition.estimateAngleToObject(AngleUnit.DEGREES);
                                leftPower = Range.clip(Math.abs(0.3 * (objectAngle / 20)), 0.15, 0.2);
                                rightPower = leftPower;
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

                            // Robot moves toward block while the height ratio is < 0.45
                            if ((double) recognition.getHeight() / (double) recognition.getImageHeight() < targetRatio - 0.05 && sideways == false) {
                                heightRatio = (double) recognition.getHeight() / (double) recognition.getImageHeight();
                                telemetry.addData("Height Ratio", heightRatio);
                                leftPower = Range.clip(0.055 + 0.5 * (targetRatio - heightRatio - 0.05), 0.15, 0.2);
                                rightPower = leftPower;
                                telemetry.addData("Left Power", leftPower);
                                telemetry.addData("Right Power", rightPower);
                                telemetry.update();
                            }

                            // Robot moves back while the height ratio is > 0.55
                            else if ((double) recognition.getHeight() / (double) recognition.getImageHeight() > targetRatio + 0.05 && sideways == false) {
                                heightRatio = (double) recognition.getHeight() / (double) recognition.getImageHeight();
                                telemetry.addData("Height Ratio", heightRatio);
                                leftPower = -0.12;
                                rightPower = leftPower;
                                telemetry.addData("Left Power", leftPower);
                                telemetry.addData("Right Power", rightPower);
                                telemetry.addData("Estimated Angle", recognition.estimateAngleToObject(AngleUnit.DEGREES));
                                telemetry.update();
                            }

                            // Robot stops when the height ratio is between 0.45 & 0.55
                            else if (sideways == false) {
                                telemetry.addData("Height Ratio", (double) recognition.getHeight() / (double) recognition.getImageHeight());
                                leftPower = 0;
                                rightPower = 0;
                                blockFound = true;
                                telemetry.addData("Left Power", leftPower);
                                telemetry.addData("Right Power", rightPower);
                                telemetry.addData("Estimated Angle", recognition.estimateAngleToObject(AngleUnit.DEGREES));
                                telemetry.update();
                            }

                            // Sets power to motors if the robot is aligned with the block
                            if (sideways == false) {
                                hardware.left_back.setPower(leftPower);
                                hardware.left_front.setPower(leftPower);
                                hardware.right_back.setPower(rightPower);
                                hardware.right_front.setPower(rightPower);
                            }

                            // If the height ratio is between 0.45 & 0.55, the robot will go back to its starting position
                            double x, y, ratio;
                            if (blockFound) {
                                perWidth = recognition.getWidth();
                                telemetry.addData("Pixel Width", perWidth);
                                //telemetry.addData("Distance", Distance(perWidth, knownDistance, perWidth));
                                telemetry.addData("X", globalPositionUpdate.returnXCoordinate());
                                telemetry.addData("Y", globalPositionUpdate.returnYCoordinate());
                                telemetry.addData("Height", (double) recognition.getHeight() / (double) recognition.getImageHeight());

                                x = globalPositionUpdate.returnYCoordinate() / OdometryHelper.COUNTS_PER_INCH;
                                ratio = 5.5 / ((double) recognition.getHeight() / (double) recognition.getImageHeight());
                                telemetry.addData("Ratio", ratio);
                                telemetry.update();
                                OdometryHelper.goToPosition(globalPositionUpdate,hardware, this, globalPositionUpdate.returnXCoordinate()+10*OdometryHelper.COUNTS_PER_INCH,globalPositionUpdate.returnYCoordinate(),0.2,0.5*OdometryHelper.COUNTS_PER_INCH,0);

                                hardware.rightShoot.setPower(-0.5);
                                sleep(1000);
                                //OdometryHelper.goToPosition(globalPositionUpdate, hardware, this, globalPositionUpdate.returnXCoordinate(), globalPositionUpdate.returnYCoordinate()+(Distance(widthPixels,knownDistance,perWidth)-8)*OdometryHelper.COUNTS_PER_INCH, 0.2, 0.5 * OdometryHelper.COUNTS_PER_INCH, 0);
                                OdometryHelper.goToPosition(globalPositionUpdate, hardware, this, globalPositionUpdate.returnXCoordinate(), globalPositionUpdate.returnYCoordinate() + (ratio+1) * OdometryHelper.COUNTS_PER_INCH, 0.2, 0.5 * OdometryHelper.COUNTS_PER_INCH, 0);
                                sleep(3000);
                                hardware.rightShoot.setPower(0.0);
                                OdometryHelper.goToPosition(globalPositionUpdate, hardware, this, globalPositionUpdate.returnXCoordinate(), globalPositionUpdate.returnYCoordinate() -2 * OdometryHelper.COUNTS_PER_INCH, 0.2, 0.5 * OdometryHelper.COUNTS_PER_INCH, 0);
                                y = globalPositionUpdate.returnYCoordinate() / OdometryHelper.COUNTS_PER_INCH;
                                blockFound = true;
                                perWidth = recognition.getWidth();
                                telemetry.addData("Distance From Origin", 24 - globalPositionUpdate.returnYCoordinate() / OdometryHelper.COUNTS_PER_INCH);

                                telemetry.addData("Height", (double) recognition.getHeight() / (double) recognition.getImageHeight());
                                telemetry.update();

                                hardware.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                                hardware.lift.setTargetPosition(1100);
                                hardware.lift.setPower(0.4);
                                while (opModeIsActive() && hardware.lift.getCurrentPosition() < hardware.lift.getTargetPosition())
                                {
                                    telemetry.addData("encoder counts", hardware.lift.getCurrentPosition());
                                    telemetry.update();
                                    idle();
                                }
                                hardware.lift.setPower(0.0);

                                //hardware.throwing.setPosition(0.4);

                                sleep(2000);



                                hardware.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                                hardware.lift.setTargetPosition(0);
                                hardware.lift.setPower(-0.4);
                                while (opModeIsActive() && hardware.lift.getCurrentPosition() > hardware.lift.getTargetPosition())
                                {
                                    telemetry.addData("encoder counts", hardware.lift.getCurrentPosition());
                                    telemetry.update();
                                    idle();
                                }
                                hardware.lift.setPower(0.0);

                                sleep(5000);

                                hardware.choo.setTargetPosition(hardware.choo.getCurrentPosition()+1935);
                                hardware.choo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                hardware.choo.setPower(1.0);
                                while (opModeIsActive() && hardware.choo.isBusy())
                                {
                                    telemetry.addData("encoder counts", hardware.choo.getCurrentPosition());
                                    telemetry.update();
                                    idle();
                                }
                                hardware.choo.setPower(0.0);

                                break;
                            }

                            if (tfod.getRecognitions() == null || updatedRecognitions.size()==0)
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

    // todo: write your code here
}