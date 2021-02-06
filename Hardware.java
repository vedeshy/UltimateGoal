package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;


public class Hardware {
    public DcMotor right_front, right_back, left_front, left_back, lift, leftShoot, rightShoot, choo;
    public DcMotorEx verticalLeft, verticalRight, horizontal;
    public CRServo left, throwing;
    public Servo right, claw, turn;
    public BNO055IMU imu;
    public WebcamName webcam;
    HardwareMap hwMap;

    String rfName = "rightFront", rbName = "rightRear", lfName = "leftFront", lbName = "leftRear";
    String verticalLeftEncoderName = lbName, verticalRightEncoderName = rfName, horizontalEncoderName = lfName;

    public void initHardware(HardwareMap map)
    {
        hwMap = map;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        right_front = hwMap.dcMotor.get(rfName);
        right_back = hwMap.dcMotor.get(rbName);
        left_front = hwMap.dcMotor.get(lfName);
        left_back = hwMap.dcMotor.get(lbName);
        lift = hwMap.dcMotor.get("lift");
        leftShoot = hwMap.dcMotor.get("frontCollection");
        rightShoot = hwMap.dcMotor.get("backCollection");
        choo = hwMap.dcMotor.get("choo");



        left = hwMap.crservo.get("left");
        right = hwMap.servo.get("right");
        throwing = hwMap.crservo.get("throw");
        claw = hwMap.servo.get("claw");
        turn = hwMap.servo.get("turn");

        verticalLeft = hwMap.get(DcMotorEx.class,verticalLeftEncoderName);
        verticalRight = hwMap.get(DcMotorEx.class,verticalRightEncoderName);
        horizontal = hwMap.get(DcMotorEx.class,horizontalEncoderName);

        webcam = hwMap.get(WebcamName.class, "Webcam");

        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftShoot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightShoot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        choo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftShoot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightShoot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        choo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftShoot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightShoot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        choo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        left_front.setDirection(DcMotorSimple.Direction.REVERSE);
        left_back.setDirection(DcMotorSimple.Direction.REVERSE);
        //horizontal.setDirection(DcMotorEx.Direction.REVERSE);
        verticalLeft.setDirection(DcMotorEx.Direction.REVERSE);

    }
}
