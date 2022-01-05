package org.firstinspires.ftc.teamcode.helper;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.helper.PIDController;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class SensorHelper {

    Orientation lastAngles = new Orientation();
    double globalAngle;

    public boolean isRed(ColorSensor pFrontColor, Telemetry telemetry){
        //special scale factor for Red so that Red can be recognized via the "value" value.
        //NOTE: THIS AFFECTS ALL VALUE READINGS
        final float RED_SCALE_FACTOR = 2;

        // hsvValues is an array that will hold the hue, saturation, and value information.
        // the "value" value is multiplied by 2
        float hsvValues[] = {0F, 0F, 0F * RED_SCALE_FACTOR};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        //unecessary but it looks cool and makes things consistent
        final double SCALE_FACTOR = 1;

        // convert the RGB values to HSV values.
        // multiply by the SCALE_FACTOR (for fun)
        // then cast it back to int (because SCALE_FACTOR is a double)

        Color.RGBToHSV((int) (pFrontColor.red() * SCALE_FACTOR),
                (int) (pFrontColor.green() * SCALE_FACTOR),
                (int) (pFrontColor.blue() * SCALE_FACTOR),
                hsvValues);

        // if Statements - These Should Be Tested and Adjusted Accordingly
        //you could also make an additional class for this but I just put it in one
        if (pFrontColor.red() > 59) {
            telemetry.addLine("LINE COLOR: RED");
            telemetry.update();
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            return true;
        } else if (pFrontColor.red() < 59) {
            telemetry.addLine("LINE COLOR: BLUE");
            telemetry.update();
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            return false;
        } else {
            return false;
        }
    }

    public boolean isWhite(ColorSensor pFrontColor, Telemetry telemetry){
        //special scale factor for Red so that Red can be recognized via the "value" value.
        //NOTE: THIS AFFECTS ALL VALUE READINGS
        final float RED_SCALE_FACTOR = 2;

        // hsvValues is an array that will hold the hue, saturation, and value information.
        // the "value" value is multiplied by 2
        float hsvValues[] = {0F, 0F, 0F * RED_SCALE_FACTOR};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        //unecessary but it looks cool and makes things consistent
        final double SCALE_FACTOR = 1;

        // convert the RGB values to HSV values.
        // multiply by the SCALE_FACTOR (for fun)
        // then cast it back to int (because SCALE_FACTOR is a double)

        Color.RGBToHSV((int) (pFrontColor.red() * SCALE_FACTOR),
                (int) (pFrontColor.green() * SCALE_FACTOR),
                (int) (pFrontColor.blue() * SCALE_FACTOR),
                hsvValues);

        // if Statements - These Should Be Tested and Adjusted Accordingly
        //you could also make an additional class for this but I just put it in one
        if (pFrontColor.red() > 30) {
            telemetry.addLine("LINE COLOR: WHITE");
            telemetry.update();
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            return true;
        } else if (pFrontColor.red() < 30) {
            telemetry.addLine("LINE COLOR: BLACK");
            telemetry.update();
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            return false;
        } else {
            return false;
        }
    }

    public void distanceReading(DistanceSensor pFrontDistance, double distanceAmount){
        distanceAmount = pFrontDistance.getDistance(DistanceUnit.INCH);
    }

    public void moveUntilColor(double pSpeed, double pStopDist, boolean isRed, DcMotor pBackLeft, DcMotor pBackRight, DcMotor pFrontRight, DcMotor pFrontLeft, Telemetry telemetry, BNO055IMU pImu, ColorSensor pFrontColor) {

        pFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ElapsedTime runtime = new ElapsedTime();

        PIDController pidDrive = new PIDController(.05, 0, 0);
        lastAngles = new Orientation();

        //Variables used for converting inches to Encoder dounts
        final double COUNTS_PER_MOTOR_DCMOTOR = 1120;    // eg: TETRIX Motor Encoder
        final double DRIVE_GEAR_REDUCTION = 0.5;     // This is < 1.0 if geared UP
        final double WHEEL_DIAMETER_INCHES = 3.93701;     // For figuring circumference
        final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_DCMOTOR * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415);

        // newTargetPosition is the target position after it has been converted
        int newTargetPositionRight;
        int newTargetPositionLeft;
        // Sets all encoder values to 0 as we are moving robot with all 4 encoders
        pBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Determine new target position, converts it, and pass to motor controller
        newTargetPositionLeft = pBackLeft.getCurrentPosition() + (int) (pStopDist * COUNTS_PER_INCH);
        newTargetPositionRight = pBackRight.getCurrentPosition() + (int) (pStopDist * COUNTS_PER_INCH);
        newTargetPositionLeft = pFrontLeft.getCurrentPosition() + (int) (pStopDist * COUNTS_PER_INCH);
        newTargetPositionRight = pFrontRight.getCurrentPosition() + (int) (pStopDist * COUNTS_PER_INCH);
        pBackLeft.setTargetPosition(newTargetPositionLeft);
        pBackRight.setTargetPosition(newTargetPositionRight);
        pFrontLeft.setTargetPosition(newTargetPositionLeft);
        pFrontRight.setTargetPosition(newTargetPositionRight);
        runtime.reset();

        telemetry.addData("Initial Value", "Running at %7d :%7d",
                pBackLeft.getCurrentPosition(), pBackRight.getCurrentPosition());
        telemetry.addData("Initial Value", "Running at %7d :%7d",
                pFrontLeft.getCurrentPosition(), pFrontRight.getCurrentPosition());
        telemetry.update();

        pFrontRight.setPower((pSpeed));
        pBackRight.setPower((pSpeed));
        pFrontLeft.setPower((pSpeed));
        pBackLeft.setPower((pSpeed));

        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, pSpeed);
        pidDrive.setInputRange(90, 90);
        pidDrive.enable();
        double correction;

        //This while loop will keep the motors running to the target position until color is detected or pStopDist is reached
        while ((pBackLeft.isBusy() && pBackRight.isBusy() && pFrontLeft.isBusy() && pFrontRight.isBusy())) {
            correction = pidDrive.performPID(getAngle(pImu));
            pFrontRight.setPower(pSpeed + correction);
            pBackRight.setPower(pSpeed + correction);
            pFrontLeft.setPower(pSpeed - correction);
            pBackLeft.setPower(pSpeed - correction);

            if(isRed){
                if (pFrontColor.red() > 59) {
                    telemetry.addLine("RED DETECTED STOP");
                    telemetry.update();
                    break;
                }
            }else{
                if (pFrontColor.red() > 30) {
                    telemetry.addLine("WHITE DETECTED STOP");
                    telemetry.update();
                    break;
                }
            }
            if(pFrontLeft.getCurrentPosition() >= pStopDist || pFrontRight.getCurrentPosition() >= pStopDist) {
                telemetry.addLine("BACKUP STOP");
                telemetry.update();
                break;
            }
        }

        //stop motors
        pFrontLeft.setPower(0);
        pFrontRight.setPower(0);
        pBackLeft.setPower(0);
        pBackRight.setPower(0);

        telemetry.addData("Final Position", "Running at %7d :%7d : %7d: %7d",
                pBackLeft.getCurrentPosition(),
                pBackRight.getCurrentPosition(),
                pFrontLeft.getCurrentPosition(),
                pFrontRight.getCurrentPosition());
        telemetry.update();
    }

    private double getAngle(BNO055IMU pImu)
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = pImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    public void untilRed(DcMotor pBackLeft, DcMotor pBackRight, DcMotor pFrontRight, DcMotor pFrontLeft, Telemetry telemetry, BNO055IMU pImu, ColorSensor pFrontColor) {

        ElapsedTime runtime = new ElapsedTime();

        while ((pBackLeft.isBusy() && pBackRight.isBusy() && pFrontLeft.isBusy() && pFrontRight.isBusy())) {

            runtime.reset();

            pFrontRight.setPower(0.3);
            pBackRight.setPower(0.3);
            pFrontLeft.setPower(0.3);
            pBackLeft.setPower(0.3);

            if (pFrontColor.red() > 59) {
                telemetry.addLine("RED DETECTED STOP");
                telemetry.update();

                break;

            } else if (runtime.seconds()>5) {
                pFrontRight.setPower(0);
                pBackRight.setPower(0);
                pFrontLeft.setPower(0);
                pBackLeft.setPower(0);

                telemetry.addLine("FAILED. END.");
                telemetry.update();
                break;

            }
        }

    }


    }





