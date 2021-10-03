package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class GoForwardEasy {

    @Autonomous(name="Go Forward", group="auto")

    public class GoForward extends LinearOpMode {

        DcMotor frontLeft;
        DcMotor frontRight;
        DcMotor backLeft;
        DcMotor backRight;

        NavigationHelper navigate = new NavigationHelper();
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class,"imu");

        @Override
        public void runOpMode() throws InterruptedException {

            initialize();
            waitForStart();
            navigate.forwardDrive(10,0.5,backLeft,backRight,frontRight,frontLeft,telemetry, imu,true);
            frontRight.setPower(0.35);
            backLeft.setPower(0.35);
            backRight.setPower(0.35);

            try{
                Thread.sleep(3000);
            }
            catch(Exception e){

            }

            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            try{

                Thread.sleep(5000);
            }
            catch(Exception e){

            }

            frontLeft.setPower(-0.35);
            frontRight.setPower(-0.35);
            backLeft.setPower(-0.35);
            backRight.setPower(-0.35);

            try{
                Thread.sleep(1500);
            }
            catch(Exception e){

            }

            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

        }

        public void initialize(){
            frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
            frontRight = hardwareMap.get(DcMotor.class, "frontRight");
            backLeft = hardwareMap.get(DcMotor.class, "backLeft");
            backRight = hardwareMap.get(DcMotor.class, "backRight");

            frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
            backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        }

    }
}
