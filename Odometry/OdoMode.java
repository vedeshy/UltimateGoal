package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.IMUHelper;
import org.firstinspires.ftc.teamcode.OdometryHelper;

import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


@TeleOp(name = "OdoMode")
public class OdoMode extends LinearOpMode {
    
    Hardware hardware;
    double RF; double LF; double RR; double LR;
    // declare joystick position variables
    double X1; double Y1; double X2; double Y2;
    // operational constants
    double joyScale = 0.65;
    double motorMax = 0.5; // Limit motor power to this value for Andymark RUN_USING_ENCODER mode

    OdometryGlobalCoordinatePosition globalPositionUpdate;

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        hardware = new Hardware();

        hardware.initHardware(hardwareMap);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(hardware.verticalLeft, hardware.verticalRight, hardware.horizontal, OdometryHelper.COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        while(opModeIsActive()){
            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate()/OdometryHelper.COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate()/OdometryHelper.COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());
            telemetry.addData("Vertical left encoder position", hardware.verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", hardware.verticalRight.getCurrentPosition());
            telemetry.addData("horizontal encoder position", hardware.horizontal.getCurrentPosition());

            // Reset speed variables
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

            if (gamepad1.right_bumper)
            {
                OdometryHelper.goToPosition(globalPositionUpdate, hardware, this,0 * OdometryHelper.COUNTS_PER_INCH,0*OdometryHelper.COUNTS_PER_INCH,0.2,1*OdometryHelper.COUNTS_PER_INCH,0);
                telemetry.update();
            }
            if (gamepad1.y)
            {
                OdometryHelper.goToPosition(globalPositionUpdate, hardware, this,0 * OdometryHelper.COUNTS_PER_INCH,18*OdometryHelper.COUNTS_PER_INCH,0.2,1*OdometryHelper.COUNTS_PER_INCH,0);
                telemetry.update();
            }
            if (gamepad1.b)
            {
                OdometryHelper.goToPosition(globalPositionUpdate, hardware, this,18 * OdometryHelper.COUNTS_PER_INCH,0*OdometryHelper.COUNTS_PER_INCH,0.2,1*OdometryHelper.COUNTS_PER_INCH,0);
                telemetry.update();
            }
            if (gamepad1.a)
            {
                OdometryHelper.goToPosition(globalPositionUpdate, hardware, this,0 * OdometryHelper.COUNTS_PER_INCH,-18*OdometryHelper.COUNTS_PER_INCH,0.2,1*OdometryHelper.COUNTS_PER_INCH,0);
                telemetry.update();
            }
            if (gamepad1.x)
            {
                OdometryHelper.goToPosition(globalPositionUpdate, hardware, this,-18 * OdometryHelper.COUNTS_PER_INCH,0*OdometryHelper.COUNTS_PER_INCH,0.2,1*OdometryHelper.COUNTS_PER_INCH,0);
                telemetry.update();
            }
            if (gamepad1.dpad_up)
            {
                OdometryHelper.goToPosition(globalPositionUpdate, hardware, this,-18 * OdometryHelper.COUNTS_PER_INCH,-18*OdometryHelper.COUNTS_PER_INCH,0.3,1*OdometryHelper.COUNTS_PER_INCH,0);
                sleep(500);
                OdometryHelper.goToPosition(globalPositionUpdate, hardware, this,-18 * OdometryHelper.COUNTS_PER_INCH,18*OdometryHelper.COUNTS_PER_INCH,0.3,1*OdometryHelper.COUNTS_PER_INCH,0);
                sleep(500);
                OdometryHelper.goToPosition(globalPositionUpdate, hardware, this,18 * OdometryHelper.COUNTS_PER_INCH,18*OdometryHelper.COUNTS_PER_INCH,0.3,1*OdometryHelper.COUNTS_PER_INCH,0);
                sleep(500);
                OdometryHelper.goToPosition(globalPositionUpdate, hardware, this,18 * OdometryHelper.COUNTS_PER_INCH,-18*OdometryHelper.COUNTS_PER_INCH,0.3,1*OdometryHelper.COUNTS_PER_INCH,0);
                sleep(500);
                OdometryHelper.goToPosition(globalPositionUpdate, hardware, this,0 * OdometryHelper.COUNTS_PER_INCH,0*OdometryHelper.COUNTS_PER_INCH,0.3,1*OdometryHelper.COUNTS_PER_INCH,0);
            }

            telemetry.update();
        }

        if (gamepad1.left_bumper)
        {
            OdometryHelper.goToPosition(globalPositionUpdate, hardware, this,0 * OdometryHelper.COUNTS_PER_INCH,-18*OdometryHelper.COUNTS_PER_INCH,0.2,0.5*OdometryHelper.COUNTS_PER_INCH,0);
            sleep(1000);
            OdometryHelper.goToPosition(globalPositionUpdate, hardware, this, -18 * OdometryHelper.COUNTS_PER_INCH,-18*OdometryHelper.COUNTS_PER_INCH,0.2,0.5*OdometryHelper.COUNTS_PER_INCH,0);
            sleep(1000);
            OdometryHelper.goToPosition(globalPositionUpdate, hardware, this,-18 * OdometryHelper.COUNTS_PER_INCH,0*OdometryHelper.COUNTS_PER_INCH,0.2,0.5*OdometryHelper.COUNTS_PER_INCH,0);
            sleep(1000);
            OdometryHelper.goToPosition(globalPositionUpdate, hardware, this,-18 * OdometryHelper.COUNTS_PER_INCH,18*OdometryHelper.COUNTS_PER_INCH,0.2,0.5*OdometryHelper.COUNTS_PER_INCH,0);
            sleep(1000);
            OdometryHelper.goToPosition(globalPositionUpdate, hardware, this, 0 * OdometryHelper.COUNTS_PER_INCH,18*OdometryHelper.COUNTS_PER_INCH,0.2,0.5*OdometryHelper.COUNTS_PER_INCH,0);
            sleep(1000);
            OdometryHelper.goToPosition(globalPositionUpdate, hardware, this, 18 * OdometryHelper.COUNTS_PER_INCH,18*OdometryHelper.COUNTS_PER_INCH,0.2,0.5*OdometryHelper.COUNTS_PER_INCH,0);
            sleep(1000);
            OdometryHelper.goToPosition(globalPositionUpdate, hardware, this,18 * OdometryHelper.COUNTS_PER_INCH,0*OdometryHelper.COUNTS_PER_INCH,0.2,0.5*OdometryHelper.COUNTS_PER_INCH,0);
            sleep(1000);
            OdometryHelper.goToPosition(globalPositionUpdate, hardware, this,18 * OdometryHelper.COUNTS_PER_INCH,-18*OdometryHelper.COUNTS_PER_INCH,0.2,0.5*OdometryHelper.COUNTS_PER_INCH,0);
            sleep(1000);
            OdometryHelper.goToPosition(globalPositionUpdate, hardware, this, 0 * OdometryHelper.COUNTS_PER_INCH,-18*OdometryHelper.COUNTS_PER_INCH,0.2,0.5*OdometryHelper.COUNTS_PER_INCH,0);
            sleep(1000);
            OdometryHelper.goToPosition(globalPositionUpdate, hardware, this, 0 * OdometryHelper.COUNTS_PER_INCH,0*OdometryHelper.COUNTS_PER_INCH,0.2,0.5*OdometryHelper.COUNTS_PER_INCH,0);
            sleep(1000);
        }

        //Stop the thread
        globalPositionUpdate.stop();

    }
}
