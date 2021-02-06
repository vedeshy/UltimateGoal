package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.robotcore.hardware.configuration.Utility;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import java.util.Locale;
import android.app.Activity;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import java.util.Locale;


import java.util.Locale;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="IMU Check", group="Linear Opmode")
//@Disabled

public class IMUTurn extends LinearOpMode {
     private ElapsedTime runtime = new ElapsedTime();
 DcMotor leftFrontMotor = null;
 DcMotor rightFrontMotor = null;
 DcMotor leftRearMotor = null;
 DcMotor rightRearMotor = null;
 BNO055IMU imu;
 Orientation lastAngles = new Orientation();
 Orientation angel = new Orientation();
 double globalAngle, power = .05, correction;
 boolean aButton, bButton, xButton, yButton;

    // State used for updating telemetry
 Orientation angles;
 Acceleration gravity;

 // declare motor speed variables
 double RF; double LF; double RR; double LR;
 // declare joystick position variables
 double X1; double Y1; double X2; double Y2;
 // operational constants
 double joyScale = 0.65;
 double motorMax = 0.5; // Limit motor power to this value for Andymark RUN_USING_ENCODER mode

 @Override
 public void runOpMode() {
 telemetry.addData("Status", "Initialized");
 telemetry.update();

 /* Initialize the hardware variables. Note that the strings used here as parameters
 * to 'get' must correspond to the names assigned during the robot configuration
 * step (using the FTC Robot Controller app on the phone).
 */
 leftFrontMotor = hardwareMap.dcMotor.get("left front");
 rightFrontMotor = hardwareMap.dcMotor.get("right front");
 leftRearMotor = hardwareMap.dcMotor.get("left back");
 rightRearMotor = hardwareMap.dcMotor.get("right back");
 BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
 parameters.mode = BNO055IMU.SensorMode.IMU;
 parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
 parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
 parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
 parameters.loggingEnabled      = true;
 parameters.loggingTag          = "IMU";
 parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
 
 // Set the drive motor direction:
 // "Reverse" the motor that runs backwards when connected directly to the battery
 // These polarities are for the Neverest 20 motors
 leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
 rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
 leftRearMotor.setDirection(DcMotor.Direction.REVERSE);
 rightRearMotor.setDirection(DcMotor.Direction.FORWARD);

 // Set the drive motor run modes:
 // "RUN_USING_ENCODER" causes the motor to try to run at the specified fraction of full velocity
 // Note: We were not able to make this run mode work until we switched Channel A and B encoder wiring into
 // the motor controllers. (Neverest Channel A connects to MR Channel B input, and vice versa.)
 leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
 rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
 leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
 rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

 leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
 rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
 leftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
 rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
 
 imu = hardwareMap.get(BNO055IMU.class, "imu");
 imu.initialize(parameters);

 while (!isStopRequested() && !imu.isGyroCalibrated())
    {
     sleep(50);
     idle();
    }
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
 leftFrontMotor.setPower(LF);
 rightFrontMotor.setPower(RF);
 leftRearMotor.setPower(LR);
 rightRearMotor.setPower(RR);
 
 if (gamepad1.a)
 {
     gyroTurn(60.0);
 }
 if (gamepad1.b)
 {
     gyroTurn(30.0);
 }
 if (gamepad1.x)
 {
     gyroTurn(90.0);
 }
 if (gamepad1.y)
 {
     gyroTurn(0.0);
 }
 
 
 telemetry.update();
 }
 }
 private float getHeading()
 {
     return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
 }
 
 private void gyroTurn(double deg)
 {
     double errorDegrees = 360;
     double motorOutput = 0;
     double targetAngle = deg;//((360+deg+(getHeading() + 360) % 360) % 360);
     //while ((360+targetAngle-((getHeading()+360)%360))%360 !=0 && opModeIsActive())
     while (Math.round((360+targetAngle-((getHeading()+360)%360))%360) !=0 && opModeIsActive())
     {
         errorDegrees = (360+targetAngle - (getHeading() + 360) % 360)%360;
         telemetry.addData("Target Angle", targetAngle);
         telemetry.addData("Current Heading", (getHeading()+360)%360);
         motorOutput = Range.clip((targetAngle-getHeading())%360 * 0.1,-0.1,0.1);
         telemetry.addData("Speed", motorOutput);
         telemetry.addData("Error", errorDegrees);
         telemetry.update();
         leftFrontMotor.setPower(-1*motorOutput);
         rightFrontMotor.setPower(motorOutput);
         leftRearMotor.setPower(-1*motorOutput);
         rightRearMotor.setPower(motorOutput);
     }
     telemetry.addData("Speed", motorOutput);
     leftFrontMotor.setPower(0.0);
     rightFrontMotor.setPower(0.0);
     leftRearMotor.setPower(0.0);
     rightRearMotor.setPower(0.0);
     sleep(1000);
     telemetry.addData("Current Heading", (getHeading()+360)%360);
     telemetry.update();
 }
}





