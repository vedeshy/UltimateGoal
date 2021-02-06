package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.IMUHelper;
import org.firstinspires.ftc.teamcode.Odometry.OdometryGlobalCoordinatePosition;
import org.firstinspires.ftc.teamcode.OdometryHelper;

import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

/**
 * Created by Sarthak on 6/1/2019.
 * Example OpMode that runs the GlobalCoordinatePosition thread and accesses the (x, y, theta) coordinate values
 */
@TeleOp(name = "Global Coordinate Position Test", group = "Calibration")
public class GlobalCoordinatePositionUpdateSample extends LinearOpMode {

    //private static final String TFOD_MODEL_ASSET = "newBall.tflite";
    //private static final String TFOD_MODEL_ASSET = "Soccer.tflite";
    //private static final String LABEL_FIRST_ELEMENT = "S";
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    //private static final String TFOD_MODEL_ASSET = "Soccer.tflite";
    //private static final String LABEL_FIRST_ELEMENT = "S";
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
    double targetRatio = 0.4;
    boolean blockFound = false;
    boolean sideways = true;
    double leftPower = 0;
    double rightPower = 0;

    double RF = 0; double LF = 0; double RR = 0; double LR = 0;
    // declare joystick position variables
    double X1 = 0; double Y1 = 0; double X2 = 0; double Y2 = 0;
    // operational constants
    double joyScale = 0.8;
    double motorMax = 1.0; // Limit motor power to this value for Andymark RUN_USING_ENCODER mode
    Hardware hardware;

    OdometryGlobalCoordinatePosition globalPositionUpdate;

    private static final String VUFORIA_KEY = "ATJz83b/////AAABmXLUTP+vOUYCo0NmIag1kv1Jd8vnhGXWh8i8OCr0TDgxzpjmscoRy56Nppi/nBpMJJ/GLnK2wVQQDQGut12bCgdwdWAndSnNDfj8fbpv3KiCrm+mgV5YqzOOAaih00CkGf2EYSov9LkE/s6yjK5965lkpFvvLL5TtV7fWcbW6Ez/aSTYnIbvS2Q3yJOr7ycLqgOOqiRRpmbb26WL94TZr2IzG6mZacM2IbFHMKIyw4vKJv3gTTMqXAt6es24VYu5Jt3Rmr0t4jbGrb2hpo1JwtNVr4+F8FVLuMmkNIcCrLqrVQ0bl3KMM/kYq8Hav7UR1yQrbwjdIVAm9gv7WXSYYj0rFv5wncI7K/rzt7K4FRJQ";

    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;



    @Override
    public void runOpMode() throws InterruptedException {
        hardware = new Hardware();
        hardware.initHardware(hardwareMap);

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

        waitForStart();

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions\
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(hardware.verticalLeft, hardware.verticalRight, hardware.horizontal, OdometryHelper.COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        //globalPositionUpdate.reverseRightEncoder();
        //globalPositionUpdate.reverseNormalEncoder();
        //globalPositionUpdate.reverseLeftEncoder();

        while(opModeIsActive()){
            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / OdometryHelper.COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / OdometryHelper.COUNTS_PER_INCH);
            telemetry.addData("Vertical Left Position", hardware.verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical Right Position", hardware.verticalRight.getCurrentPosition());
            telemetry.addData("Horizontal Position", hardware.horizontal.getCurrentPosition());
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());
            telemetry.addData("Thread Active", positionThread.isAlive());
            telemetry.update();

            LF = 0; RF = 0; LR = 0; RR = 0;

            // Get joystick values
            Y1 = -gamepad1.right_stick_y * joyScale; // invert so up is positive
            X1 = gamepad1.right_stick_x * joyScale;
            Y2 = -gamepad1.left_stick_y * joyScale; // Y2 is not used at present
            X2 = gamepad1.left_stick_x * joyScale;

            // Forward/back movement
            LF += Y1; RF += Y1; LR += Y1; RR += Y1;

            // Side to side movement
            LF += X1; RF -= X1; LR -= X1; RR += X1;

            // Rotation movement
            LF += X2; RF -= X2; LR += X2; RR -= X2;


            // Clip motor power values to +-motorMax
            LF = Math.max(-motorMax, Math.min(LF, motorMax));
            RF = Math.max(-motorMax, Math.min(RF, motorMax));
            LR = Math.max(-motorMax, Math.min(LR, motorMax));
            RR = Math.max(-motorMax, Math.min(RR, motorMax));

            // Send values to the motors
            hardware.left_front.setPower(LF);
            hardware.right_front.setPower(RF);
            hardware.left_back.setPower(LR);
            hardware.right_back.setPower(RR);



            if (gamepad1.x)
            {
                hardware.leftShoot.setPower(-1.0);
                hardware.rightShoot.setPower(1.0);
            }




            if (gamepad1.y)
            {
                hardware.lift.setTargetPosition(500);
                hardware.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hardware.lift.setPower(0.3);
                while (opModeIsActive() && hardware.lift.isBusy())
                {
                    telemetry.addData("encoder counts", hardware.lift.getCurrentPosition());
                    telemetry.update();
                    idle();
                }
                hardware.lift.setPower(0.0);
            }

            telemetry.addData("encoder counts", hardware.lift.getCurrentPosition());
            telemetry.update();

            if (gamepad1.a)
            {
                hardware.lift.setTargetPosition(0);
                hardware.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hardware.lift.setPower(-0.3);
                while (opModeIsActive() && hardware.lift.isBusy())
                {
                    telemetry.addData("encoder counts", hardware.lift.getCurrentPosition());
                    telemetry.update();
                    idle();
                }
                hardware.lift.setPower(0.0);
            }

            if (gamepad1.dpad_up)
            {
                hardware.lift.setPower(0.0);
            }



            if (gamepad1.dpad_down)
            {
                OdometryHelper.goToPosition(globalPositionUpdate, hardware, this,64 * OdometryHelper.COUNTS_PER_INCH,4*OdometryHelper.COUNTS_PER_INCH,0.8,0.5*OdometryHelper.COUNTS_PER_INCH,0);
                IMUHelper.gyroTurn(this,hardware,0.0);
                sleep(1000);
                findBall();
                sleep(250);

                OdometryHelper.goToPosition(globalPositionUpdate,hardware,this,10*OdometryHelper.COUNTS_PER_INCH,10*OdometryHelper.COUNTS_PER_INCH,0.6,1*OdometryHelper.COUNTS_PER_INCH,0.0);
                IMUHelper.gyroTurn(this,hardware,90.0);

                sleep(250);
                OdometryHelper.goToPosition(globalPositionUpdate,hardware,this,globalPositionUpdate.returnXCoordinate()-8*OdometryHelper.COUNTS_PER_INCH,globalPositionUpdate.returnYCoordinate(),0.6,1*OdometryHelper.COUNTS_PER_INCH,0.0);
                sleep(250);
                hardware.lift.setTargetPosition(1075);
                hardware.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hardware.lift.setPower(0.3);
                while (opModeIsActive() && hardware.lift.isBusy())
                {
                    telemetry.addData("encoder counts", hardware.lift.getCurrentPosition());
                    telemetry.update();
                    idle();
                }
                hardware.lift.setPower(0.0);
                sleep(250);

                hardware.lift.setTargetPosition(0);
                hardware.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hardware.lift.setPower(-0.3);
                while (opModeIsActive() && hardware.lift.isBusy())
                {
                    telemetry.addData("encoder counts", hardware.lift.getCurrentPosition());
                    telemetry.update();
                    idle();
                }
                hardware.lift.setPower(0.0);
                OdometryHelper.goToPosition(globalPositionUpdate,hardware,this,globalPositionUpdate.returnXCoordinate()+8*OdometryHelper.COUNTS_PER_INCH,globalPositionUpdate.returnYCoordinate(),0.6,1*OdometryHelper.COUNTS_PER_INCH,0.0);
                sleep(250);

                IMUHelper.gyroTurn(this,hardware,0.0);
                sleep(250);
                OdometryHelper.goToPosition(globalPositionUpdate, hardware, this,0* OdometryHelper.COUNTS_PER_INCH,0*OdometryHelper.COUNTS_PER_INCH,0.2,0.5*OdometryHelper.COUNTS_PER_INCH,0);
                sleep(250);
                IMUHelper.gyroTurn(this,hardware,0.0);
            }


        }

        //Stop the thread
        globalPositionUpdate.stop();
    }

    public void findBall()
    {
        while (true)
        {
            Recognition recognition = null;
            float confidenceLevel = 0;
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());

                for (int i = 0; i < updatedRecognitions.size(); i++) {
                    Recognition currRecognition = updatedRecognitions.get(i);
                    if (currRecognition.getConfidence() > confidenceLevel) {
                        confidenceLevel = currRecognition.getConfidence();
                        recognition = currRecognition;
                    }

                }

                // step through the list of recognitions and display boundary info.

                if (recognition != null)
                {
                    //telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    //telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                    //        recognition.getLeft(), recognition.getTop());
                    //telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                    //        recognition.getRight(), recognition.getBottom());
                    imageHeight = (double) recognition.getImageHeight();
                    objectHeight = (double) recognition.getHeight();
                    heightRatio = objectHeight / imageHeight;
                    objectAngle = recognition.estimateAngleToObject(AngleUnit.DEGREES);
                    telemetry.update();


                    //telemetry.addData("Left Power", leftPower);
                    //telemetry.addData("Right Power", rightPower);
                    //telemetry.addData("Height Ratio", heightRatio);
                    //telemetry.addData("Estimated Angle", recognition.estimateAngleToObject(AngleUnit.DEGREES));

                    // Camera lines itself up with the block with a +/- 3 leeway
                    if (Math.abs(recognition.estimateAngleToObject(AngleUnit.DEGREES)) > 1 && sideways) {
                        //objectAngle = recognition.estimateAngleToObject(AngleUnit.DEGREES);
                        leftPower = Range.clip(Math.abs(0.2 * (objectAngle / 20)), 0.15, 0.2);
                        rightPower = Range.clip(Math.abs(0.2 * (objectAngle / 20)), 0.15, 0.2);
                        if (objectAngle > 0) {
                            leftPower *= -1;
                        } else {
                            rightPower *= -1;
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
                        widthPixels = recognition.getWidth();
                        sideways = false;
                    }

                    // Robot moves toward block while the height ratio is < 0.45
                    //if ((double) recognition.getHeight() / (double) recognition.getImageHeight() < targetRatio - 0.05 && sideways == false) {
                    //    heightRatio = (double) recognition.getHeight() / (double) recognition.getImageHeight();
                    //    telemetry.addData("Height Ratio", heightRatio);
                    //    leftPower = Range.clip(0.055 + 0.5 * (targetRatio - heightRatio - 0.05), 0.15, 0.2);
                    //    rightPower = leftPower;
                    //    telemetry.addData("Left Power", leftPower);
                    //    telemetry.addData("Right Power", rightPower);
                    //    telemetry.update();
                    //}

                    // Robot moves back while the height ratio is > 0.55
                    //else if ((double) recognition.getHeight() / (double) recognition.getImageHeight() > targetRatio + 0.05 && sideways == false) {
                    //    heightRatio = (double) recognition.getHeight() / (double) recognition.getImageHeight();
                    //    telemetry.addData("Height Ratio", heightRatio);
                    //    leftPower = -0.15;
                    //    rightPower = leftPower;
                    //    telemetry.addData("Left Power", leftPower);
                    //    telemetry.addData("Right Power", rightPower);
                    //    telemetry.addData("Estimated Angle", recognition.estimateAngleToObject(AngleUnit.DEGREES));
                    //    telemetry.update();
                    //}

                    // Robot stops when the height ratio is between 0.45 & 0.55
                    if (sideways == false) {
                        target = true;
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
                    if (blockFound && target) {
                        perWidth = recognition.getWidth();
                        telemetry.addData("Pixel Width", perWidth);
                        //telemetry.addData("Distance", Distance(perWidth, knownDistance, perWidth));
                        telemetry.addData("X", globalPositionUpdate.returnXCoordinate());
                        telemetry.addData("Y", globalPositionUpdate.returnYCoordinate());
                        telemetry.addData("Height", (double) recognition.getHeight() / (double) recognition.getImageHeight());
                        telemetry.update();
                        x = globalPositionUpdate.returnYCoordinate() / OdometryHelper.COUNTS_PER_INCH;
                        ratio = 5.5 / ((double) recognition.getHeight() / (double) recognition.getImageHeight());
                        OdometryHelper.goToPosition(globalPositionUpdate,hardware, this, globalPositionUpdate.returnXCoordinate()+5*OdometryHelper.COUNTS_PER_INCH,globalPositionUpdate.returnYCoordinate(),0.2,0.5*OdometryHelper.COUNTS_PER_INCH,0);

                        //OdometryHelper.goToPosition(globalPositionUpdate, hardware, this, globalPositionUpdate.returnXCoordinate(), globalPositionUpdate.returnYCoordinate()+(Distance(widthPixels,knownDistance,perWidth)-8)*OdometryHelper.COUNTS_PER_INCH, 0.2, 0.5 * OdometryHelper.COUNTS_PER_INCH, 0);
                        OdometryHelper.goToPosition(globalPositionUpdate, hardware, this, globalPositionUpdate.returnXCoordinate(), globalPositionUpdate.returnYCoordinate() + (ratio+1) * OdometryHelper.COUNTS_PER_INCH, 0.2, 0.5 * OdometryHelper.COUNTS_PER_INCH, 0);

                        sleep(250);
                        y = globalPositionUpdate.returnYCoordinate() / OdometryHelper.COUNTS_PER_INCH;
                        blockFound = true;
                        perWidth = recognition.getWidth();
                        telemetry.addData("Distance From Origin", 24 - globalPositionUpdate.returnYCoordinate() / OdometryHelper.COUNTS_PER_INCH);
                        telemetry.addData("Distance Traveled", y - x);
                        telemetry.addData("Height", (double) recognition.getHeight() / (double) recognition.getImageHeight());
                        telemetry.update();
                        target = false;
                        break;
                    }

                    if (tfod.getRecognitions() == null || updatedRecognitions.size() == 0) {
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
        }
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardware.webcam;
        //parameters.cameraDirection = CAMERA_CHOICE;
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
        tfodParameters.minimumConfidence = 0.9;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        //tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
        //tfod.loadModelFromAsset(TFOD_MODEL_ASSET,LABEL_FIRST_ELEMENT);
    }

}


