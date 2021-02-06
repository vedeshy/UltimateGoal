package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Odometry.OdometryGlobalCoordinatePosition;

public class OdometryHelper {
    public static double COUNTS_PER_INCH = 306.38164202785072637277329100595; // 306.382978;

    public static void goToPosition(OdometryGlobalCoordinatePosition globalPositionUpdate,Hardware hardware, LinearOpMode opMode, double targetXPosition, double targetYPosition, double robotPower, double allowableDistanceError,double desiredRobotOrientation)
    {
        double RF; double LF; double RR; double LR;
        boolean check = false;


        double distanceToXTarget = targetXPosition-globalPositionUpdate.returnXCoordinate();
        double distanceToYTarget = targetYPosition-globalPositionUpdate.returnYCoordinate();

        double distance = Math.hypot(distanceToXTarget,distanceToYTarget);
        while (distance> allowableDistanceError)
        {
            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToXTarget,distanceToYTarget));



            if (Math.abs(globalPositionUpdate.returnOrientation())<15)
            {
                check = false;
            }
            else
            {
                check = true;
            }

            double robotMovementX = calculateX(robotMovementAngle,robotPower, globalPositionUpdate.returnOrientation(),check);
            double robotMovementY = calculateY(robotMovementAngle,robotPower, globalPositionUpdate.returnOrientation(),check);

            if (distance < 5*COUNTS_PER_INCH)
            {
                if (Math.abs(robotMovementX/3)>0.15)
                {
                    robotMovementX/=3;
                }

                if (Math.abs(robotMovementY/3)>0.15)
                {
                    robotMovementY/=3;
                }
            }


            LF = 0; RF = 0; LR = 0; RR = 0;

            LF += robotMovementY; RF += robotMovementY; LR += robotMovementY; RR += robotMovementY;
            LF += robotMovementX; RF -= robotMovementX; LR -= robotMovementX; RR += robotMovementX;

            hardware.left_front.setPower(LF);
            hardware.right_front.setPower(RF);
            hardware.left_back.setPower(LR);
            hardware.right_back.setPower(RR);

            distanceToXTarget = targetXPosition-globalPositionUpdate.returnXCoordinate();
            distanceToYTarget = targetYPosition-globalPositionUpdate.returnYCoordinate();

            distance = Math.hypot(distanceToXTarget,distanceToYTarget);
        }
        hardware.left_front.setPower(0.0);
        hardware.right_front.setPower(0.0);
        hardware.left_back.setPower(0.0);
        hardware.right_back.setPower(0.0);
    }

    public static void goToPositionOld(OdometryGlobalCoordinatePosition globalPositionUpdate,Hardware hardware, LinearOpMode opMode, double targetXPosition, double targetYPosition, double robotPower, double allowableDistanceError,double desiredRobotOrientation)
    {
        double RF; double LF; double RR; double LR;
        final double COUNTS_PER_INCH = 306.382978;
        //double[] speedX = new double[500];
        //double[] speedY = new double[500];
        //double[] angle = new double[500];
        //double[] distanceTarget = new double[500];
        //double pivotCorrection = 0;
        double scalar = 0.3;
        int count = 0;
        double distanceToXTarget = targetXPosition-globalPositionUpdate.returnXCoordinate();
        double distanceToYTarget = targetYPosition-globalPositionUpdate.returnYCoordinate();
        //double XTarget = targetXPosition-globalPositionUpdate.returnXCoordinate();
        //double YTarget = targetYPosition-globalPositionUpdate.returnYCoordinate();
        double distance = Math.hypot(distanceToXTarget,distanceToYTarget);

        while (distance > allowableDistanceError)
        //while (Math.abs(distanceToXTarget) > allowableDistanceError*2 || Math.abs(distanceToYTarget) > allowableDistanceError*2)
        {
            //if (count == 2000) break;

            distanceToXTarget = targetXPosition-globalPositionUpdate.returnXCoordinate();
            distanceToYTarget = targetYPosition-globalPositionUpdate.returnYCoordinate();

            distance = Math.hypot(distanceToXTarget,distanceToYTarget);

            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToXTarget,distanceToYTarget));

            if (count > 0 && scalar < 1)
            {
                scalar+=0.02;
            }

            boolean check = false;

            if (Math.abs(globalPositionUpdate.returnOrientation())<15)
            {
                check = false;
            }
            else
            {
                check = true;
            }

            double robotMovementX = calculateX(robotMovementAngle,robotPower*1.2, globalPositionUpdate.returnOrientation(), check)*scalar;
            double robotMovementY = calculateY(robotMovementAngle,robotPower*1.2, globalPositionUpdate.returnOrientation(), check)*scalar;

            if (distance < 2.5*COUNTS_PER_INCH)
            {
                robotMovementX/=5;
                robotMovementY/=5;
            }
            else if (distance < Math.hypot(distanceToXTarget,robotMovementY)/2)
            {
                robotMovementX/=2;
                robotMovementY/=2;
            }
            LF = 0; RF = 0; LR = 0; RR = 0;

            LF += robotMovementY; RF += robotMovementY; LR += robotMovementY; RR += robotMovementY;
            LF += robotMovementX; RF -= robotMovementX; LR -= robotMovementX; RR += robotMovementX;

            //telemetry.addData("Distance", globalPositionUpdate.returnYCoordinate());
            //telemetry.addData("Left Back Speed", LR);
            //telemetry.addData("Right Front Speed", RF);
            //telemetry.addData("Right Back Speed", RR);
            //telemetry.update();
//            if (distance < COUNTS_PER_INCH && robotMovementX > 0.1 && robotMovementY > 0.1) {
//                LF/=2;
//                LR/=2;
//                RF/=2;
//                RR/=2;
//            }
            hardware.left_front.setPower(LF);
            hardware.right_front.setPower(RF);
            hardware.left_back.setPower(LR);
            hardware.right_back.setPower(RR);

            //speedX[count%500] = robotMovementX;
            //speedY[count%500] = robotMovementY;
            //angle[count%500] = robotMovementAngle;
            //distanceTarget[count%500] = distance;
            count++;
            //pivotCorrection = desiredRobotOrientation;
        }
        hardware.left_front.setPower(0.0);
        hardware.right_front.setPower(0.0);
        hardware.left_back.setPower(0.0);
        hardware.right_back.setPower(0.0);
        opMode.sleep(500);

        //for (int i = 0; i < count%500; i++)
        //{
        //telemetry.addData("Speed X", speedX[i]);
        //    telemetry.addData("Speed Y", speedY[i]);
        //telemetry.addData("Angle", angle[i]);
        //    telemetry.addData("Distance", distanceTarget[i]);
        //}
        //telemetry.update();
    }

    /**
     * Calculate the power in the x direction
     * @param desiredAngle angle on the x axis
     * @param speed robot's speed
     * @return the x vector
     */
    private static double calculateX(double desiredAngle, double speed, double orientation, boolean check) {
        if (check == true)
        {
            return Math.sin(Math.toRadians(desiredAngle-orientation)) * speed;
        }
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Calculate the power in the y direction
     * @param desiredAngle angle on the y axis
     * @param speed robot's speed
     * @return the y vector
     */
    private static double calculateY(double desiredAngle, double speed, double orientation, boolean check) {
        if (check == true)
        {
            return Math.cos(Math.toRadians(desiredAngle-orientation)) * speed;
        }
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }
}
