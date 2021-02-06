package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="IMU Heading TeleOp ", group="Linear Opmode")
//@Disabled

public class ImuHeadingTestTeleOp extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    Hardware hardware;
    // declare motor speed variables
    double RF; double LF; double RR; double LR;
    // declare joystick position variables
    double X1; double Y1; double X2; double Y2;
    // operational constants
    double joyScale = 0.8;
    double motorMax = 1; // Limit motor power to this value for Andymark RUN_USING_ENCODER mode

    @Override
    public void runOpMode() {

        hardware = new Hardware();
        hardware.initHardware(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /* Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

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


            if (gamepad1.a)
            {
                IMUHelper.gyroTurn(this, hardware,0.0);
            }
            if (gamepad1.b)
            {
                IMUHelper.gyroTurn(this, hardware,30.0);
            }
            if (gamepad1.x)
            {
                IMUHelper.gyroTurn(this, hardware,60.0);
            }
            if (gamepad1.y)
            {
                IMUHelper.gyroTurn(this, hardware,90.0);
            }
        }
    }
}





