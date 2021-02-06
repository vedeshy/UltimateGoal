package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Odometry.OdometryGlobalCoordinatePosition;

import java.util.List;


@TeleOp(name="New TeleOp ", group="Linear Opmode")
//@Disabled

public class NewTeleOP extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    Hardware hardware;
    // declare motor speed variables
    double RF; double LF; double RR; double LR;
    // declare joystick position variables
    double X1; double Y1; double X2; double Y2;
    // operational constants
    double joyScale = 0.85;
    double motorMax = 0.75; // Limit motor power to this value for Andymark RUN_USING_ENCODER mode
    boolean collect = false, collectReverse = false, shoot = false, push = false, claw = false, turn = false;

    OdometryGlobalCoordinatePosition globalPositionUpdate;

    @Override
    public void runOpMode() {

        hardware = new Hardware();
        hardware.initHardware(hardwareMap);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

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

            telemetry.addData("Current Heading", (IMUHelper.getHeading(hardware)+360)%360);
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / OdometryHelper.COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / OdometryHelper.COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());
            telemetry.addData("Vertical left encoder position", hardware.verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", hardware.verticalRight.getCurrentPosition());
            telemetry.addData("horizontal encoder position", hardware.horizontal.getCurrentPosition());

            if (gamepad1.y) {
                if (claw == false)
                {
                    hardware.claw.setPosition(0.5);
                    claw = true;
                }
                else
                {
                    hardware.claw.setPosition(1.0);
                    claw = false;
                }
            }

            if (gamepad1.dpad_left) {
                hardware.throwing.setPower(-1.0);
            }

            if (gamepad1.dpad_right) {
                hardware.throwing.setPower(0.0);
            }

            if (gamepad1.dpad_up)
            {
                hardware.choo.setPower(0.0);
                hardware.leftShoot.setPower(0.0);
                hardware.rightShoot.setPower(0.0);
            }

            if (gamepad1.a) {
                hardware.turn.setPosition(0.1);
            }

            if (gamepad1.dpad_down) {
                hardware.turn.setPosition(0.8);
            }

            if (gamepad1.x) {
                hardware.choo.setPower(1.0);
                hardware.leftShoot.setPower(1.0);
                hardware.rightShoot.setPower(1.0);
            }

            if (gamepad1.b) {
                hardware.choo.setPower(-1.0);
                hardware.leftShoot.setPower(-1.0);
                hardware.rightShoot.setPower(-1.0);
            }

            if (gamepad1.right_bumper)
            {
                hardware.lift.setPower(-1.0);
            }
            if (gamepad1.left_bumper)
            {
                hardware.lift.setPower(0.0);
            }
        }

    }

}
