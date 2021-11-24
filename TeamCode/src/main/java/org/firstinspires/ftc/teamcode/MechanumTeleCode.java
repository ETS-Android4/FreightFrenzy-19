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

@TeleOp(name="drive", group="teleop")

public class MechanumTeleCode extends LinearOpMode {

    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    DcMotor frontLeftMotor;
    DcMotor dumperLift;
    DcMotor intake;
    DcMotor carousel;
    Servo dumperServo;
    BNO055IMU imu;
    final double dumperDump = 0.35;
    final double dumperGoingUp = 0.65;
    final double dumperIntaking = 0.84;
    ElapsedTime a_time = new ElapsedTime();
    ElapsedTime b2_time = new ElapsedTime();
    ElapsedTime x2_time = new ElapsedTime();
    ElapsedTime b_time = new ElapsedTime();
    ElapsedTime x_time = new ElapsedTime();
    ElapsedTime y_time = new ElapsedTime();
    ElapsedTime dpadup_time = new ElapsedTime();
    ElapsedTime dpaddown_time = new ElapsedTime();
    ElapsedTime dpadup2_time = new ElapsedTime();
    ElapsedTime dpaddown2_time = new ElapsedTime();
    boolean intakeOn = false;
    boolean extakeOn = false;
    boolean carouselOn = false;
    boolean slowMode = false;

    public void initialize(){

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        imu = hardwareMap.get(BNO055IMU.class,"imu");
        imu.initialize(parameters);

        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeft");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRight");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeft");

        dumperLift = hardwareMap.get(DcMotor.class, "dumperLift");
        intake = hardwareMap.get(DcMotor.class, "intake");
        carousel = hardwareMap.get(DcMotor.class, "carousel");

        dumperServo = hardwareMap.get(Servo.class,"dumperServo");
        dumperServo.setPosition(dumperIntaking);

        //FORWARD,FORWAD, REVERSE, REVERSE (FORWARD/BACK WAS GOOD AND TURNS/STRAFES WERE FLIPPED)
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    private void mecanumDrive(double scale){

        double radius= Math.hypot(gamepad2.left_stick_x,gamepad2.left_stick_y);
        double angle = (Math.atan2(-(gamepad2.left_stick_y),(gamepad2.left_stick_x)))-(Math.PI/4);
        double rotation = gamepad2.right_stick_x * 0.5;
        double fLPower = 0;
        double bLPower = 0;
        double fRPower = 0;
        double bRPower = 0;

        if( (angle > 5*(Math.PI/12))&& (angle < 7*(Math.PI/12)) ){
            fLPower = radius * Math.cos(Math.PI/4) - rotation;
            bLPower = radius * Math.sin(Math.PI/4) - rotation;
            fRPower = radius * Math.sin(Math.PI/4) + rotation;
            bRPower = radius * Math.cos(Math.PI/4) + rotation;
        }
        else if( (angle < -5*(Math.PI/12))&& (angle > -7*(Math.PI/12)) ){
            fLPower = radius * Math.cos(-3*Math.PI/4) - rotation;
            bLPower = radius * Math.sin(-3*Math.PI/4) - rotation;
            fRPower = radius * Math.sin(-3*Math.PI/4) + rotation;
            bRPower = radius * Math.cos(-3*Math.PI/4) + rotation;
        }
        else {
            fLPower = radius * Math.cos(angle) + rotation;
            bLPower = radius * Math.sin(angle) + rotation;
            fRPower = radius * Math.sin(angle) - rotation;
            bRPower = radius * Math.cos(angle) - rotation;

        }
        frontLeftMotor.setPower((fLPower) * scale);
        backLeftMotor.setPower((bLPower) * scale);
        frontRightMotor.setPower((fRPower) * scale);
        backRightMotor.setPower((bRPower) * scale);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        while(opModeIsActive()){

            if(gamepad2.dpad_up && dpadup2_time.seconds()>0.25){
                dpadup2_time.reset();
                slowMode=false;
            }
            if(gamepad2.dpad_down && dpaddown2_time.seconds()>0.25){
                dpaddown2_time.reset();
                slowMode=true;
            }

            if(gamepad2.x && x2_time.seconds()>0.25){
                x2_time.reset();
                if(!carouselOn){
                    carousel.setPower(0.25);
                    carouselOn = true;
                }
                else{
                    carousel.setPower(-0.2);
                    try {
                        Thread.sleep(100);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    carousel.setPower(0);
                    carouselOn = false;
                }
            }

            if(gamepad2.b && b2_time.seconds()>0.25){
                b2_time.reset();
                if(!carouselOn){
                    carousel.setPower(-0.25);
                    carouselOn = true;
                }
                else{
                    carousel.setPower(0.2);
                    try {
                        Thread.sleep(100);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    carousel.setPower(0);
                    carouselOn = false;
                }
            }

            if(gamepad2.right_bumper && dpadup_time.seconds()>0.25){
                dpadup_time.reset();
                if(!intakeOn){
                    intake.setPower(0.5);
                    intakeOn=true;
                    extakeOn=false;
                }
                else{
                    intake.setPower(0);
                    intakeOn=false;
                    extakeOn=false;
                }
            }
            if(gamepad1.left_bumper && dpaddown_time.seconds()>0.25){
                dpaddown_time.reset();
                if(!extakeOn){
                    intake.setPower(-0.75);
                    extakeOn=true;
                    intakeOn=false;
                }
                else{
                    intake.setPower(0);
                    extakeOn=false;
                    intakeOn=false;
                }
            }

            if(gamepad1.b&&b_time.seconds()>0.25){
                b_time.reset();
                dumperServo.setPosition(dumperDump);
            }
            if(gamepad1.a&&a_time.seconds()>0.25){
                a_time.reset();
                dumperServo.setPosition(dumperGoingUp);
            }
            if(gamepad1.x&&x_time.seconds()>0.25){
                x_time.reset();
                dumperServo.setPosition(dumperIntaking);
            }




            if(gamepad1.left_stick_y>0.05){
                dumperLift.setPower(0.5);
            }
            else if(gamepad1.left_stick_y<-0.05){
                dumperLift.setPower(-0.5);
            }
            else{
                dumperLift.setPower(0);
            }


            if(slowMode){
                telemetry.addData("speed", 0.35);
                telemetry.update();
                mecanumDrive(0.35);
            }
            else{
                telemetry.addData("speed", 1);
                telemetry.update();
                mecanumDrive(1);
            }
            float header = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                    AngleUnit.DEGREES).firstAngle;
            telemetry.addData("IMU:", header);
            telemetry.update();
            idle();
        }
    }
}
