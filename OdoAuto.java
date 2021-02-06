package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class OdoAuto extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private static final String VUFORIA_KEY =
            "ATJz83b/////AAABmXLUTP+vOUYCo0NmIag1kv1Jd8vnhGXWh8i8OCr0TDgxzpjmscoRy56Nppi/nBpMJJ/GLnK2wVQQDQGut12bCgdwdWAndSnNDfj8fbpv3KiCrm+mgV5YqzOOAaih00CkGf2EYSov9LkE/s6yjK5965lkpFvvLL5TtV7fWcbW6Ez/aSTYnIbvS2Q3yJOr7ycLqgOOqiRRpmbb26WL94TZr2IzG6mZacM2IbFHMKIyw4vKJv3gTTMqXAt6es24VYu5Jt3Rmr0t4jbGrb2hpo1JwtNVr4+F8FVLuMmkNIcCrLqrVQ0bl3KMM/kYq8Hav7UR1yQrbwjdIVAm9gv7WXSYYj0rFv5wncI7K/rzt7K4FRJQ";


    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    WebcamName webcam;
    public void runOpMode() throws InterruptedException {
        Hardware hardware = new Hardware();
        hardware.initHardware(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        webcam = hardwareMap.get(WebcamName.class, "Webcam");

        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
        }

        hardware.claw.setPosition(0.5);

        Trajectory traj = drive.trajectoryBuilder(new Pose2d(), true)
                .splineTo(new Vector2d(-15, 0), Math.toRadians(180))
                .build();
        Trajectory traj1 = drive.trajectoryBuilder(traj.end(), true)
                .splineTo(new Vector2d(-60, 7), Math.toRadians(180))
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToLinearHeading(new Pose2d(-9, -2, Math.toRadians(90)))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end(), true)
                .splineToConstantHeading(new Vector2d(-8.5, -15), Math.toRadians(90))
                .build();
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end(), true)
                .lineToLinearHeading(new Pose2d(-60,7, Math.toRadians(17)))
                .build();
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end(), true)
                .lineToLinearHeading(new Pose2d(-63,-18, Math.toRadians(0)))
                .build();

        Trajectory traj6 = drive.trajectoryBuilder(traj.end(), true)
                .splineTo(new Vector2d(-108, 7), Math.toRadians(180))
                .build();
        Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
                .lineToLinearHeading(new Pose2d(-9, -2, Math.toRadians(90)))
                .build();
        Trajectory traj8 = drive.trajectoryBuilder(traj7.end(), true)
                .splineToConstantHeading(new Vector2d(-8.5, -15), Math.toRadians(90))
                .build();
        Trajectory traj9 = drive.trajectoryBuilder(traj8.end(), true)
                .lineToLinearHeading(new Pose2d(-48, 7, Math.toRadians(0)))
                .build();
        Trajectory traj10 = drive.trajectoryBuilder(traj9.end(), true)
                .lineTo(new Vector2d(-108, 7))
                .build();
        Trajectory traj11 = drive.trajectoryBuilder(traj10.end(), true)
                .lineToLinearHeading(new Pose2d(-63,-18, Math.toRadians(0)))
                .build();

        Trajectory traj12 = drive.trajectoryBuilder(traj.end(), true)
                .splineTo(new Vector2d(-84, -13 ), Math.toRadians(180))
                .build();
        Trajectory traj13 = drive.trajectoryBuilder(traj12.end())
                .lineToLinearHeading(new Pose2d(-84, -2, Math.toRadians(90)))
                .build();
        Trajectory traj14 = drive.trajectoryBuilder(traj13.end())
                .lineToLinearHeading(new Pose2d(-9, -2, Math.toRadians(90)))
                .build();
        Trajectory traj15 = drive.trajectoryBuilder(traj14.end(), true)
                .splineToConstantHeading(new Vector2d(-8.5, -15), Math.toRadians(90))
                .build();
        Trajectory traj16 = drive.trajectoryBuilder(traj15.end(), true)
                .lineToLinearHeading(new Pose2d(-48, 7, Math.toRadians(0)))
                .build();
        Trajectory traj17 = drive.trajectoryBuilder(traj16.end(), true)
                .lineTo(new Vector2d(-84, -18))
                .build();
        Trajectory traj18 = drive.trajectoryBuilder(traj17.end(), true)
                .lineToLinearHeading(new Pose2d(-63,-18, Math.toRadians(0)))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        String code = "";
        drive.followTrajectory(traj);
        sleep(1000);
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
        sleep(1000);

        if (code == "none") {
            drive.followTrajectory(traj1);
            hardware.turn.setPosition(0.1);
            sleep(500);
            hardware.claw.setPosition(1.0);
            sleep(500);
            hardware.turn.setPosition(0.8);
            drive.followTrajectory(traj2);
            hardware.turn.setPosition(0.1);
            sleep(500);
            drive.followTrajectory(traj3);
            hardware.claw.setPosition(0.5);
            sleep(500);
            drive.followTrajectory(traj4);
            hardware.claw.setPosition(1.0);
            sleep(500);
            hardware.turn.setPosition(0.8);
            hardware.lift.setPower(-0.925);
            drive.followTrajectory(traj5);
            sleep(1500);
            hardware.throwing.setPower(-1.0);
            sleep(4000);
            hardware.lift.setPower(0.0);
            hardware.throwing.setPower(0.0);
            hardware.turn.setPosition(0.2);
            sleep(5000);
        }

        else if (code == "Quad")
        {
            drive.followTrajectory(traj6);
            hardware.turn.setPosition(0.1);
            sleep(500);
            hardware.claw.setPosition(1.0);
            sleep(500);
            hardware.turn.setPosition(0.8);
            drive.followTrajectory(traj7);
            hardware.turn.setPosition(0.1);
            sleep(500);
            drive.followTrajectory(traj8);
            hardware.claw.setPosition(0.5);
            sleep(500);
            drive.followTrajectory(traj9);
            drive.followTrajectory(traj10);
            hardware.lift.setPower(-0.925);
            hardware.claw.setPosition(1.0);
            sleep(500);
            hardware.turn.setPosition(0.8);
            drive.followTrajectory(traj11);
            //sleep(500);
            hardware.throwing.setPower(-1.0);
            sleep(3500);
            hardware.lift.setPower(0.0);
            hardware.throwing.setPower(0.0);
            hardware.turn.setPosition(0.2);
            sleep(5000);
        }
        else if (code == "Single")
        {
            drive.followTrajectory(traj12);
            hardware.turn.setPosition(0.1);
            sleep(500);
            hardware.claw.setPosition(1.0);
            sleep(500);
            hardware.turn.setPosition(0.8);
            drive.followTrajectory(traj13);
            drive.followTrajectory(traj14);
            hardware.turn.setPosition(0.1);
            sleep(500);
            drive.followTrajectory(traj15);
            hardware.claw.setPosition(0.5);
            sleep(500);
            drive.followTrajectory(traj16);
            drive.followTrajectory(traj17);
            hardware.lift.setPower(-0.9);
            hardware.claw.setPosition(1.0);
            sleep(500);
            hardware.turn.setPosition(0.8);
            drive.followTrajectory(traj18);
            hardware.throwing.setPower(-1.0);
            sleep(3500);
            hardware.lift.setPower(0.0);
            hardware.throwing.setPower(0.0);
            hardware.turn.setPosition(0.2);
            sleep(5000);
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
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
