package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name="Test Cap", group="teleop")

public class TestCap extends LinearOpMode {


    Servo baseCap;
    Servo topCap;
    BNO055IMU imu;
    ElapsedTime a_time = new ElapsedTime();
    ElapsedTime b_time = new ElapsedTime();
    ElapsedTime x_time = new ElapsedTime();
    ElapsedTime y_time = new ElapsedTime();

    public void initialize(){

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        imu = hardwareMap.get(BNO055IMU.class,"imu");
        imu.initialize(parameters);

        baseCap = hardwareMap.get(Servo.class, "baseCap");
        topCap = hardwareMap.get(Servo.class, "topCap");

        baseCap.setPosition(0.5);
        topCap.setPosition(0.5);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        double baseCurrent = 0.5;
        double topCurrent = 0.5;
        while(opModeIsActive()) {
            if(gamepad1.a&&a_time.seconds()>=0.25){
                a_time.reset();
                baseCurrent+=0.1;
                baseCap.setPosition(baseCurrent);
            }
            if(gamepad1.b&&b_time.seconds()>=0.25){
                b_time.reset();
                baseCurrent-=0.1;
                baseCap.setPosition(baseCurrent);
            }
            if(gamepad1.x&&x_time.seconds()>=0.25){
                x_time.reset();
                topCurrent+=0.1;
                topCap.setPosition(topCurrent);
            }
            if(gamepad1.y&&y_time.seconds()>=0.25){
                y_time.reset();
                topCurrent-=0.1;
                topCap.setPosition(topCurrent);
            }

        }

    }
}
