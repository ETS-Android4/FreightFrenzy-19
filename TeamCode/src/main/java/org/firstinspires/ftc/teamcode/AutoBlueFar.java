package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(name="AUTO BLUE FAR", group="auto")

public class AutoBlueFar extends LinearOpMode {

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    DcMotor intake;
    DcMotor carousel;
    Servo dumperServo;


    final double dumperDump = 0.35;
    final double dumperGoingUp = 0.65;
    final double dumperFirstLevel = 0.70;
    final double dumperIntaking = 0.85;


    NavigationHelper navigate = new NavigationHelper();
    BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        waitForStart();

        telemetry.addData("entering loop", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        telemetry.update();

        float header = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                AngleUnit.DEGREES).firstAngle;

        if(opModeIsActive()){
            try {
                Thread.sleep(7000);
            } catch (InterruptedException E) {

            }
            navigate.forwardDrive(16, 0.5, backLeft, backRight, frontRight, frontLeft, telemetry, imu, true);
            try {
                Thread.sleep(500);
            } catch (InterruptedException E) {

            }
            navigate.navigate(25, Constants2020.Direction.LEFT, 0, 0.3, backLeft, backRight, frontRight, frontLeft, imu, telemetry, header, false);
            try {
                Thread.sleep(500);
            } catch (InterruptedException E) {

            }

            dumperServo.setPosition(dumperFirstLevel);
            try {
                Thread.sleep(500);
            } catch(InterruptedException E){

            }
            intake.setPower(-0.7);
            try {
                Thread.sleep(2000);
            } catch(InterruptedException E){

            }
            intake.setPower(0);

            navigate.navigate(60, Constants2020.Direction.RIGHT, 0, 0.5, backLeft, backRight, frontRight, frontLeft, imu, telemetry, header, true);
            try {
                Thread.sleep(500);
            } catch (InterruptedException E) {

            }
            navigate.forwardDrive(13, 0.5, backLeft, backRight, frontRight, frontLeft, telemetry, imu, true);
            try {
                Thread.sleep(500);
            } catch (InterruptedException E) {

            }
        }



    }

    public void initialize(){
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");


        intake = hardwareMap.get(DcMotor.class, "intake");
        carousel = hardwareMap.get(DcMotor.class, "carousel");
        dumperServo = hardwareMap.get(Servo.class,"dumperServo");
        dumperServo.setPosition(dumperGoingUp);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        imu = hardwareMap.get(BNO055IMU.class,"imu");
        imu.initialize(parameters);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
    }

}










