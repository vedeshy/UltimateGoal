package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;



public class IMUHelper {

    public static float getHeading(Hardware hardware)
    {
        return hardware.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
    public static void gyroTurn(LinearOpMode opMode, Hardware hardware, double deg)
    {
        double errorDegrees;
        double diff = 0;
        //double[] head = new double[1000];
        //double[] err = new double[1000];
        //double[] speed = new double[1000];
         int count = 0;
        //double heading = getHeading();
        double motorOutput = 1;

        double targetAngle = deg;//((360+deg+(getHeading() + 360) % 360) % 360);
        //while ((360+targetAngle-((getHeading()+360)%360))%360 !=0 && opModeIsActive())
        while (opMode.opModeIsActive())
        //while((errorDegrees > 1 || errorDegrees < -1) && opModeIsActive())
        {
            double heading = Math.round((getHeading(hardware) + 360)) % 360;
            if (Math.round((360+targetAngle-heading)%360) ==0 || Math.abs(motorOutput) < 0.005 || count > 1000) {
                if (Math.round((360 + targetAngle - ((getHeading(hardware) + 360) % 360)) % 360) == 0 || Math.abs(motorOutput) < 0.005 || count > 50) {
                    opMode.telemetry.addData("Count", count);
                    opMode.telemetry.addData("Speed", motorOutput);
                    opMode.telemetry.addData("Diff", diff);
                    break;
                }
            }
            errorDegrees = Math.abs(targetAngle - heading) %360;

            diff = heading - targetAngle;


            //telemetry.addData("Target Angle", targetAngle);
            //telemetry.addData("Current Heading", heading);
            opMode.telemetry.addData("Current Heading", (getHeading(hardware)+360)%360);
            opMode.telemetry.addData("Error Degrees", errorDegrees);
            opMode.telemetry.update();

            if (Math.abs(motorOutput) < 0.30 && (diff>=0 && diff <=180))
            {
                motorOutput=-0.30;
            }
            else if (diff > 180)
            {
                motorOutput=0.30;
            }
            else
            {
                motorOutput = Range.clip((errorDegrees)/(20), -0.3,0.3); //Range.clip((targetAngle-heading) * 0.05,-0.1,0.1);
                if(diff>=0 && diff <=180) {
                    if(motorOutput > 0 )motorOutput *= -1;
                }
                else {
                    if(motorOutput < 0 )motorOutput *= -1;
                }
            }

            //head[count%1000] = heading;
            //err[count%1000] = errorDegrees;
            //speed[count%1000] = motorOutput;
            count++;
            //telemetry.addData("Speed", motorOutput);
            //telemetry.addData("Error", errorDegrees);
            //telemetry.update();
            hardware.left_front.setPower(-motorOutput);
            hardware.right_front.setPower(motorOutput);
            hardware.left_back.setPower(-motorOutput);
            hardware.right_back.setPower(motorOutput);
            //sleep(50);

        }
        //telemetry.addData("Speed", motorOutput);
        hardware.left_front.setPower(0.0);
        hardware.right_front.setPower(0.0);
        hardware.left_back.setPower(0.0);
        hardware.right_back.setPower(0.0);

        opMode.telemetry.addData("Current Heading", (getHeading(hardware)+360)%360);
        //for (int i = 0; i < count%1000; i++)
        //{
        //    telemetry.addLine()
        //        .addData("Heading", head[i])
        //        .addData("Error", err[i])
        //        .addData("Speed", speed[i]);
        //}
        opMode.sleep(1000);
        opMode.telemetry.addData("Current Heading", (getHeading(hardware)+360)%360);
        opMode.telemetry.addData("Count", count);
        opMode.telemetry.addData("Diff", diff);
        opMode.telemetry.update();
    }
}
