package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.helper.Constants2022;
import org.firstinspires.ftc.teamcode.helper.DetectionHelper;
import org.firstinspires.ftc.teamcode.helper.NavigationHelper;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="AUTO ALL UPDATED", group="auto")

public class AutoAllUpdated extends LinearOpMode {

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    ElapsedTime dpad_up = new ElapsedTime();
    ElapsedTime dpad_down = new ElapsedTime();
    ElapsedTime b = new ElapsedTime();
    ElapsedTime x = new ElapsedTime();
    Servo dumperServo;
    DcMotor intake;
    DcMotor carousel;
    DcMotor pivot;
    Servo arm;
    OpenCvCamera webcam;

    /*final double dumperDump = 0.35;
    final double dumperGoingUp = 0.65;
    final double dumperFirstLevel = 0.8;
    final double dumperIntaking = 0.94;
    final double slidePower = 0.95;
    int centerPos = 1500;
    int rightPos = 2600;

     */

    boolean completeConfig = false;
    boolean red = false;
    boolean blue = false;
    boolean near = false;
    boolean far = false;

    DetectionHelper pipeline;
    NavigationHelper navigate = new NavigationHelper();
    ElapsedTime spinTime = new ElapsedTime();
    BNO055IMU imu;

    DetectionHelper.DuckPosition position = DetectionHelper.DuckPosition.RIGHT;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        Detection();

        waitForStart();

        while(opModeIsActive()){
            Path();

            break;
        }
    }

    public void Detection(){
        while(!opModeIsActive()) {
            telemetry.addData("Left", pipeline.getAnalysis()[2]);
            telemetry.addData("Right:", pipeline.getAnalysis()[3]);
            position = pipeline.getPosition();
            if(position== DetectionHelper.DuckPosition.LEFT){
                telemetry.addData("Position", "LEFT");

            }
            else if(position == DetectionHelper.DuckPosition.CENTER){
                telemetry.addData("Position", "CENTER");
            }
            else if(position == DetectionHelper.DuckPosition.RIGHT){
                telemetry.addData("Position", "RIGHT");
            }
            telemetry.update();
            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
    }

    public void initialize() {

        while (!completeConfig) {
            if (gamepad2.b && b.seconds()>0.25) {
                b.reset();
                red = true;
                telemetry.addData("RED", red);
                telemetry.update();
            }
            if (gamepad2.x && x.seconds()>0.25) {
                x.reset();
                red = false;
                telemetry.addData("RED", red);
                telemetry.update();
            }
            if (gamepad2.dpad_up && dpad_up.seconds()>0.25) {
                dpad_up.reset();
                near = false;
                telemetry.addData("NEAR", near);
                telemetry.update();
            }
            if (gamepad2.dpad_down && dpad_down.seconds()>0.25) {
                dpad_down.reset();
                near = true;
                telemetry.addData("NEAR", near);
                telemetry.update();
            }
            if (gamepad2.y) {
                completeConfig = true;
                telemetry.addLine("Press y to Confirm!");
                telemetry.update();
            }
        }

            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json";
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.mode = BNO055IMU.SensorMode.IMU;
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());


            try {
                webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
                pipeline = new DetectionHelper();
                webcam.setPipeline(pipeline);
                webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {
                        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                    }

                    @Override
                    public void onError(int errorCode) {
                        /*
                         * This will be called if the camera could not be opened
                         */
                    }
                });

            } catch (Exception bad) {
                telemetry.addData("EXCEPTION (from try-catch):", bad.getMessage());
                bad.printStackTrace();
                telemetry.update();
                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            telemetry.addLine("FINISHED INITIALIZING WEBCAM");
            telemetry.update();
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
            frontRight = hardwareMap.get(DcMotor.class, "frontRight");
            backLeft = hardwareMap.get(DcMotor.class, "backLeft");
            backRight = hardwareMap.get(DcMotor.class, "backRight");
            intake = hardwareMap.get(DcMotor.class, "intake");
            carousel = hardwareMap.get(DcMotor.class, "alexCarushow");
            dumperServo = hardwareMap.get(Servo.class, "dumperServo");
            pivot = hardwareMap.get(DcMotor.class, "pivot");
            dumperServo.setPosition(0.7);
            pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
            backRight.setDirection(DcMotorSimple.Direction.REVERSE);
            arm = hardwareMap.get(Servo.class, "arm");



        }

        public void Path(){

        if (red) {

            if (near) {

/*
                frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

 */
                navigate.navigate(28, Constants2022.Direction.LEFT,0, 0.3,backLeft,backRight,frontRight,frontLeft,imu, telemetry, true);
                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }


                navigate.navigate(-10, Constants2022.Direction.STRAIGHT,0, -0.3,backLeft,backRight,frontRight,frontLeft,imu, telemetry, false);



                double distanceF = 19;
                if(position== DetectionHelper.DuckPosition.LEFT){
                    distanceF = 17;
                }
                else if(position == DetectionHelper.DuckPosition.CENTER){
                    distanceF = 14;
                }
                else if(position == DetectionHelper.DuckPosition.RIGHT){
                    distanceF = 14.5;
                }

                navigate.navigate(distanceF, Constants2022.Direction.STRAIGHT,0, 0.5,backLeft,backRight,frontRight,frontLeft,imu, telemetry, true);
                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }


                Placing();

                navigate.navigate(-21, Constants2022.Direction.STRAIGHT,0, -0.5,backLeft,backRight,frontRight,frontLeft,imu, telemetry, false);

                navigate.navigate(0, Constants2022.Direction.TURN, -85, 0.3,backLeft,backRight,frontRight,frontLeft,imu, telemetry, true);

                navigate.navigate(7, Constants2022.Direction.RIGHT,0, 0.3,backLeft,backRight,frontRight,frontLeft,imu, telemetry, true);

                navigate.navigate(70, Constants2022.Direction.STRAIGHT,0, 0.5,backLeft,backRight,frontRight,frontLeft,imu, telemetry, true);

            }

            else {
                navigate.navigate(0, Constants2022.Direction.TURN,-90, 0.3,backLeft,backRight,frontRight,frontLeft,imu, telemetry, true);

                navigate.navigate(2, Constants2022.Direction.RIGHT,0, 0.5,backLeft,backRight,frontRight,frontLeft,imu, telemetry, true);

                navigate.navigate(-18, Constants2022.Direction.STRAIGHT,0, -0.2,backLeft,backRight,frontRight,frontLeft,imu, telemetry, false);

                Carousel();

                navigate.navigate(-10, Constants2022.Direction.STRAIGHT,0, -0.7,backLeft,backRight,frontRight,frontLeft,imu, telemetry, false);


                navigate.navigate(39, Constants2022.Direction.LEFT,0, 0.3,backLeft,backRight,frontRight,frontLeft,imu, telemetry, true);

                navigate.navigate(-21, Constants2022.Direction.STRAIGHT,0, -0.7,backLeft,backRight,frontRight,frontLeft,imu, telemetry, false);

                navigate.navigate(27, Constants2022.Direction.STRAIGHT,0, 0.5,backLeft,backRight,frontRight,frontLeft,imu, telemetry, true);

                Placing();

                navigate.navigate(-29, Constants2022.Direction.STRAIGHT,0, -0.5,backLeft,backRight,frontRight,frontLeft,imu, telemetry, false);

                navigate.navigate(14, Constants2022.Direction.RIGHT,0, 0.3,backLeft,backRight,frontRight,frontLeft,imu, telemetry, true);


            }


        }

        else{

            if(near) {



            }
            else{



            }


        }


    }

    public void Carousel(){

        spinTime.reset();
        while (spinTime.seconds() < 2.8) {
            carousel.setPower(-0.3);

        }
        carousel.setPower(0);

    }

    public void Placing(){
        double positionArm;
        double outtakeSpeed = -0.3;
        if(position== DetectionHelper.DuckPosition.RIGHT){
             positionArm=0.15;
            arm.setPosition(positionArm);
            try {
                sleep(1000);
            } catch (Exception e) {;

            }
        }
        else if(position == DetectionHelper.DuckPosition.CENTER){
             positionArm=0.3;
            arm.setPosition(positionArm);
            try {
                sleep(1000);
            } catch (Exception e) {;

            }
        }
        else{
            outtakeSpeed = -0.3;
            positionArm=0.63;
            arm.setPosition(positionArm);
            try {
                sleep(1000);
            } catch (Exception e) {;

            }
        }

        intake.setPower(outtakeSpeed);
        try {
            sleep(2500);
        } catch (Exception e) {

        }
        intake.setPower(0);

        positionArm=0.7;
        arm.setPosition(positionArm);
        try {
            sleep(1000);
        } catch (Exception e) {;

        }

    }


    }


