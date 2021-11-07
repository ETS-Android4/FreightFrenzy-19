package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(name="AUTO RED NEAR", group="auto")

public class AutoRedNear extends LinearOpMode {

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    NavigationHelper navigate = new NavigationHelper();
    BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        waitForStart();

        telemetry.addData("entering loop", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        telemetry.update();

        navigate.forwardDrive(-20,0.5,backLeft,backRight,frontRight,frontLeft,telemetry, imu,true);
        try {
            Thread.sleep(500);
        } catch(InterruptedException E){

        }
        navigate.navigate(15, Constants2020.Direction.RIGHT,0,0.3,backLeft,backRight,frontRight,frontLeft,imu,telemetry,false);
        try {
            Thread.sleep(500);
        } catch(InterruptedException E){

        }
        navigate.forwardDrive(10,0.5,backLeft,backRight,frontRight,frontLeft,telemetry, imu,true);
        try {
            Thread.sleep(500);
        } catch(InterruptedException E){

        }
        navigate.navigate(0, Constants2020.Direction.TURN,90,0.5,backLeft,backRight,frontRight,frontLeft,imu,telemetry,true);
        try {
            Thread.sleep(500);
        } catch(InterruptedException E){

        }
        navigate.forwardDrive(-40,0.7,backLeft,backRight,frontRight,frontLeft,telemetry, imu,true);


    }

    public void initialize(){
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        imu = hardwareMap.get(BNO055IMU.class,"imu");
        imu.initialize(parameters);

        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

}










