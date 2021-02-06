package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Odometry.OdometryGlobalCoordinatePosition;

import java.util.List;


@TeleOp(name="New Auto", group="Linear Opmode")
//@Disabled

public class NewAuto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    Hardware hardware;
    // declare motor speed variables
    double RF;
    double LF;
    double RR;
    double LR;
    // declare joystick position variables
    double X1;
    double Y1;
    double X2;
    double Y2;
    // operational constants
    double joyScale = 0.65;
    double motorMax = 0.5; // Limit motor power to this value for Andymark RUN_USING_ENCODER mode

    OdometryGlobalCoordinatePosition globalPositionUpdate;

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private static final String VUFORIA_KEY =
            "ATJz83b/////AAABmXLUTP+vOUYCo0NmIag1kv1Jd8vnhGXWh8i8OCr0TDgxzpjmscoRy56Nppi/nBpMJJ/GLnK2wVQQDQGut12bCgdwdWAndSnNDfj8fbpv3KiCrm+mgV5YqzOOAaih00CkGf2EYSov9LkE/s6yjK5965lkpFvvLL5TtV7fWcbW6Ez/aSTYnIbvS2Q3yJOr7ycLqgOOqiRRpmbb26WL94TZr2IzG6mZacM2IbFHMKIyw4vKJv3gTTMqXAt6es24VYu5Jt3Rmr0t4jbGrb2hpo1JwtNVr4+F8FVLuMmkNIcCrLqrVQ0bl3KMM/kYq8Hav7UR1yQrbwjdIVAm9gv7WXSYYj0rFv5wncI7K/rzt7K4FRJQ";

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

        hardware = new Hardware();
        hardware.initHardware(hardwareMap);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        hardware.claw.setPosition(0.5);

        webcam = hardwareMap.get(WebcamName.class, "Webcam");

        initVuforia();
        initTfod();


        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();


            /* Initialize the hardware variables. Note that the strings used here as parameters
             * to 'get' must correspond to the names assigned during the robot configuration
             * step (using the FTC Robot Controller app on the phone).
             */
            // Wait for the game to start (driver presses PLAY)
            waitForStart();
            runtime.reset();

            for (LynxModule module : allHubs) {
                module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }

            //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
            globalPositionUpdate = new OdometryGlobalCoordinatePosition(hardware.verticalLeft, hardware.verticalRight, hardware.horizontal, OdometryHelper.COUNTS_PER_INCH, 75);
            Thread positionThread = new Thread(globalPositionUpdate);
            positionThread.start();


            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {

                telemetry.update();

                String code = "";
                // Reset speed variables
                LF = 0;
                RF = 0;
                LR = 0;
                RR = 0;

                // Get joystick values
                Y1 = -gamepad1.right_stick_y * joyScale; // invert so up is positive
                X1 = gamepad1.right_stick_x * joyScale;
                Y2 = -gamepad1.left_stick_y * joyScale; // Y2 is not used at present
                X2 = gamepad1.left_stick_x * joyScale;

                // Forward/back movement
                LF += Y1;
                RF += Y1;
                LR += Y1;
                RR += Y1;

                // Side to side movement
                LF += X1;
                RF -= X1;
                LR -= X1;
                RR += X1;

                // Rotation movement
                LF += X2;
                RF -= X2;
                LR += X2;
                RR -= X2;


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

                telemetry.addData("Current Heading", (IMUHelper.getHeading(hardware) + 360) % 360);
                telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / OdometryHelper.COUNTS_PER_INCH);
                telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / OdometryHelper.COUNTS_PER_INCH);
                telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());
                telemetry.addData("Vertical left encoder position", hardware.verticalLeft.getCurrentPosition());
                telemetry.addData("Vertical right encoder position", hardware.verticalRight.getCurrentPosition());
                telemetry.addData("horizontal encoder position", hardware.horizontal.getCurrentPosition());

                if (gamepad1.y) {
                    hardware.turn.setPosition(0.0);
                    OdometryHelper.goToPosition(globalPositionUpdate, hardware, this, 0 * OdometryHelper.COUNTS_PER_INCH, -50 * OdometryHelper.COUNTS_PER_INCH, 0.6, 1 * OdometryHelper.COUNTS_PER_INCH, 0.0);
                    IMUHelper.gyroTurn(this, hardware, 0.0);
                    hardware.lift.setPower(1.0);
                    sleep(2500);
                    hardware.throwing.setPower(0.5);
                    sleep(8000);
                    hardware.lift.setPower(0.0);
                    hardware.throwing.setPower(0.0);
                    OdometryHelper.goToPosition(globalPositionUpdate, hardware, this, -6 * OdometryHelper.COUNTS_PER_INCH, -86 * OdometryHelper.COUNTS_PER_INCH, 0.6, 1 * OdometryHelper.COUNTS_PER_INCH, 0.0);
                    IMUHelper.gyroTurn(this, hardware, 0.0);
                    hardware.claw.setPosition(1.0);
                    OdometryHelper.goToPosition(globalPositionUpdate, hardware, this, -13 * OdometryHelper.COUNTS_PER_INCH, -33 * OdometryHelper.COUNTS_PER_INCH, 0.6, 1 * OdometryHelper.COUNTS_PER_INCH, 0.0);
                    //IMUHelper.gyroTurn(this, hardware, 0.0);
                    //IMUHelper.gyroTurn(this,hardware, 90.0);
                    IMUHelper.gyroTurn(this, hardware, 180.0);
                    hardware.turn.setPosition(0.1);
                    OdometryHelper.goToPosition(globalPositionUpdate, hardware, this, -11.5 * OdometryHelper.COUNTS_PER_INCH, -15 * OdometryHelper.COUNTS_PER_INCH, 0.6, 1 * OdometryHelper.COUNTS_PER_INCH, 0.0);
                    hardware.claw.setPosition(0.5);
                    IMUHelper.gyroTurn(this, hardware, 0.0);
                    OdometryHelper.goToPosition(globalPositionUpdate, hardware, this, -17 * OdometryHelper.COUNTS_PER_INCH, -80.5 * OdometryHelper.COUNTS_PER_INCH, 0.6, 1 * OdometryHelper.COUNTS_PER_INCH, 0.0);
                    hardware.claw.setPosition(1.0);
                    OdometryHelper.goToPosition(globalPositionUpdate, hardware, this, -14.5 * OdometryHelper.COUNTS_PER_INCH, -60.5 * OdometryHelper.COUNTS_PER_INCH, 0.6, 1 * OdometryHelper.COUNTS_PER_INCH, 0.0);
                }
                if (gamepad1.b) {
                    //hardware.turn.setPosition(0.0);
                    OdometryHelper.goToPosition(globalPositionUpdate, hardware, this, 0 * OdometryHelper.COUNTS_PER_INCH, -15 * OdometryHelper.COUNTS_PER_INCH, 0.3, 1 * OdometryHelper.COUNTS_PER_INCH, 0.0);
                    telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / OdometryHelper.COUNTS_PER_INCH);
                    telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / OdometryHelper.COUNTS_PER_INCH);
                    telemetry.update();
                    sleep(5000);
                    IMUHelper.gyroTurn(this, hardware, 0.0);
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        if (updatedRecognitions.size() > 0) {
                            code = updatedRecognitions.get(0).getLabel();
                        } else {
                            code = "none";
                        }
                    }
                    else
                    {
                        code = "none";
                    }
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    telemetry.addData("Object Label", code);
                    telemetry.update();
                    sleep(2500);
                    IMUHelper.gyroTurn(this, hardware, 0.0);
                    OdometryHelper.goToPosition(globalPositionUpdate, hardware, this, 10 * OdometryHelper.COUNTS_PER_INCH, -63 * OdometryHelper.COUNTS_PER_INCH, 0.3, 0.5 * OdometryHelper.COUNTS_PER_INCH, 0.0);
                    telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / OdometryHelper.COUNTS_PER_INCH);
                    telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / OdometryHelper.COUNTS_PER_INCH);
                    telemetry.update();
                    sleep(5000);
                    IMUHelper.gyroTurn(this, hardware, 0.0);
                    hardware.lift.setPower(-0.75);
                    sleep(3000);
                    hardware.throwing.setPower(-0.5);
                    sleep(7000);
                    hardware.lift.setPower(0.0);
                    hardware.throwing.setPower(0.0);
                    telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / OdometryHelper.COUNTS_PER_INCH);
                    telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / OdometryHelper.COUNTS_PER_INCH);
                    telemetry.update();
                    sleep(5000);
                    if (code == "none") {
                        OdometryHelper.goToPosition(globalPositionUpdate, hardware, this, -2 * OdometryHelper.COUNTS_PER_INCH, -66 * OdometryHelper.COUNTS_PER_INCH, 0.3, 0.5 * OdometryHelper.COUNTS_PER_INCH, 0.0);
                        IMUHelper.gyroTurn(this, hardware, 0.0);
                        telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / OdometryHelper.COUNTS_PER_INCH);
                        telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / OdometryHelper.COUNTS_PER_INCH);
                        telemetry.update();
                        sleep(5000);
                        hardware.turn.setPosition(0.1);
                        sleep(500);
                        hardware.claw.setPosition(1.0);
                        sleep(500);
                        hardware.turn.setPosition(1.0);
                        OdometryHelper.goToPosition(globalPositionUpdate, hardware, this, 10 * OdometryHelper.COUNTS_PER_INCH, -60 * OdometryHelper.COUNTS_PER_INCH, 0.3, 0.5 * OdometryHelper.COUNTS_PER_INCH, 0.0);
                        hardware.turn.setPosition(0.3);
                        IMUHelper.gyroTurn(this, hardware, 0.0);
                    } else if (code == "Single") {
                        OdometryHelper.goToPosition(globalPositionUpdate, hardware, this, 24 * OdometryHelper.COUNTS_PER_INCH, -84 * OdometryHelper.COUNTS_PER_INCH, 0.3, 0.5 * OdometryHelper.COUNTS_PER_INCH, 0.0);
                        IMUHelper.gyroTurn(this, hardware, 0.0);
                        telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / OdometryHelper.COUNTS_PER_INCH);
                        telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / OdometryHelper.COUNTS_PER_INCH);
                        telemetry.update();
                        sleep(5000);
                        hardware.turn.setPosition(0.1);
                        sleep(500);
                        hardware.claw.setPosition(1.0);
                        sleep(500);
                        hardware.turn.setPosition(1.0);
                        OdometryHelper.goToPosition(globalPositionUpdate, hardware, this, 24 * OdometryHelper.COUNTS_PER_INCH, -68 * OdometryHelper.COUNTS_PER_INCH, 0.3, 0.5 * OdometryHelper.COUNTS_PER_INCH, 0.0);
                        hardware.turn.setPosition(0.3);
                        IMUHelper.gyroTurn(this, hardware, 0.0);
                    } else if (code == "Quad") {
                        OdometryHelper.goToPosition(globalPositionUpdate, hardware, this, 0 * OdometryHelper.COUNTS_PER_INCH, -108 * OdometryHelper.COUNTS_PER_INCH, 0.3, 0.5 * OdometryHelper.COUNTS_PER_INCH, 0.0);
                        IMUHelper.gyroTurn(this, hardware, 0.0);
                        telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / OdometryHelper.COUNTS_PER_INCH);
                        telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / OdometryHelper.COUNTS_PER_INCH);
                        telemetry.update();
                        sleep(5000);
                        hardware.turn.setPosition(0.1);
                        sleep(500);
                        hardware.claw.setPosition(1.0);
                        sleep(500);
                        hardware.turn.setPosition(1.0);
                        OdometryHelper.goToPosition(globalPositionUpdate, hardware, this, 0 * OdometryHelper.COUNTS_PER_INCH, -68 * OdometryHelper.COUNTS_PER_INCH, 0.3, 0.5 * OdometryHelper.COUNTS_PER_INCH, 0.0);
                        hardware.turn.setPosition(0.3);
                        IMUHelper.gyroTurn(this, hardware, 0.0);
                    }

                }

                if (gamepad1.a) {
                    IMUHelper.gyroTurn(this, hardware, 180.0);
                }
            }
        }

            }

    private void initVuforia () {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = webcam;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod () {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.7f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}