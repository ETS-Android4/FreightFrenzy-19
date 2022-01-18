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

@Autonomous(name="AUTO BLUE FAR TOBOR", group="auto")

public class AutoBlueFarUpdatedWithTobor extends LinearOpMode{



        DcMotor frontLeft;
        DcMotor frontRight;
        DcMotor backLeft;
        DcMotor backRight;
        DcMotor slideMotor;
        DcMotor intake;
        DcMotor carousel;
        Servo dumperServo;
        OpenCvCamera webcam;
        float header;

        final double dumperDump = 0.35;
        final double dumperGoingUp = 0.65;
        final double dumperFirstLevel = 0.8;
        final double dumperIntaking = 0.94;
        final double slidePower = 0.95;
        int centerPos = 1700;
        int rightPos = 2600;



        DetectionHelper pipeline;
        //navigationhelper
        NavigationHelper navigate = new NavigationHelper();
        ElapsedTime spinTime = new ElapsedTime();
        BNO055IMU imu;

        @Override
        public void runOpMode() throws InterruptedException {

            initialize();

            DetectionHelper.DuckPosition position = DetectionHelper.DuckPosition.RIGHT;
            while(!opModeIsActive()) {

                telemetry.addData("Analysis", pipeline.getAnalysis());

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

            waitForStart();

            while (opModeIsActive()) {
                telemetry.addData("entering loop", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                telemetry.update();
                header = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                        AngleUnit.DEGREES).firstAngle;

                navigate.navigate(-13, Constants2022.Direction.STRAIGHT,0, -0.5,backLeft,backRight,frontRight,frontLeft,imu, telemetry, header,false);

                dumperServo.setPosition(dumperGoingUp);

                try {
                    Thread.sleep(500);
                } catch(InterruptedException E){

                }

                navigate.navigate(0, Constants2022.Direction.TURN, -90, 0.5, backLeft, backRight, frontRight, frontLeft, imu, telemetry, header, true);

                navigate.navigate(-26, Constants2022.Direction.STRAIGHT,0, -0.5,backLeft,backRight,frontRight,frontLeft,imu, telemetry, header,false);

                navigate.navigate(7.5, Constants2022.Direction.LEFT,0,0.35,backLeft,backRight,frontRight,frontLeft,imu,telemetry,header,false);

                spinTime.reset();
                while (spinTime.seconds() < 2.8) {
                    carousel.setPower(-0.55);

                }
                carousel.setPower(0);


                navigate.navigate(30, Constants2022.Direction.RIGHT, 0, 0.5, backLeft, backRight, frontRight, frontLeft, imu, telemetry,header, true);

                navigate.navigate(-9, Constants2022.Direction.STRAIGHT, 0, -0.7, backLeft, backRight, frontRight, frontLeft, imu, telemetry, header, false);

                if(position== DetectionHelper.DuckPosition.LEFT){
                    navigate.navigate(29, Constants2022.Direction.STRAIGHT,0, 0.3,backLeft,backRight,frontRight,frontLeft,imu, telemetry, header,false);

                    dumperServo.setPosition(dumperGoingUp);
                    try {
                        Thread.sleep(500);
                    } catch(InterruptedException E){

                    }

                }
                else if (position == DetectionHelper.DuckPosition.CENTER){

                    navigate.navigate(24, Constants2022.Direction.STRAIGHT,0, 0.3,backLeft,backRight,frontRight,frontLeft,imu, telemetry, header,false);

                    slideMotor.setTargetPosition(centerPos);
                    slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    while(slideMotor.isBusy()){
                        slideMotor.setPower(slidePower);
                        telemetry.addData("encoder pos", slideMotor.getCurrentPosition());
                        telemetry.update();
                    }
                    slideMotor.setPower(0);

                }
                else{
                    navigate.navigate(21, Constants2022.Direction.STRAIGHT,0, 0.3,backLeft,backRight,frontRight,frontLeft,imu, telemetry, header,false);

                    slideMotor.setTargetPosition(rightPos);
                    slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    while(slideMotor.isBusy()){
                        slideMotor.setPower(slidePower);
                        telemetry.addData("encoder pos", slideMotor.getCurrentPosition());
                        telemetry.update();
                    }
                    slideMotor.setPower(0);


                }

                intake.setPower(-0.7);
                try {
                    Thread.sleep(1500);
                } catch(InterruptedException E){

                }
                intake.setPower(0);

                slideMotor.setTargetPosition(0);
                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while(slideMotor.isBusy()){
                    slideMotor.setPower(-slidePower);
                    telemetry.addData("encoder pos", slideMotor.getCurrentPosition());
                    telemetry.update();
                }
                slideMotor.setPower(0);


                if(position == DetectionHelper.DuckPosition.LEFT){
                    navigate.navigate(-30, Constants2022.Direction.STRAIGHT, 0, -0.7, backLeft, backRight, frontRight, frontLeft, imu, telemetry, header, false);

                }
                else if(position == DetectionHelper.DuckPosition.CENTER){
                    navigate.navigate(-25, Constants2022.Direction.STRAIGHT, 0, -0.7, backLeft, backRight, frontRight, frontLeft, imu, telemetry, header, false);

                }
                else if(position == DetectionHelper.DuckPosition.RIGHT){
                    navigate.navigate(-22, Constants2022.Direction.STRAIGHT, 0, -0.7, backLeft, backRight, frontRight, frontLeft, imu, telemetry, header, false);

                }

                navigate.navigate(10, Constants2022.Direction.LEFT, 0, 0.3, backLeft, backRight, frontRight, frontLeft, imu, telemetry, header, false);






            /*

            navigate.navigate(41, Constants2022.Direction.STRAIGHT, 0, 0.7, backLeft, backRight, frontRight, frontLeft, imu, telemetry, header, true);

            navigate.navigate(0, Constants2022.Direction.TURN, -90, 0.5, backLeft, backRight, frontRight, frontLeft, imu, telemetry, header, true);

            navigate.navigate(-5, Constants2022.Direction.STRAIGHT, 0, -0.7, backLeft, backRight, frontRight, frontLeft, imu, telemetry, header, false);


            navigate.navigate(21, Constants2022.Direction.STRAIGHT,0, 0.5,backLeft,backRight,frontRight,frontLeft,imu, telemetry, header,true);


            if(position== DetectionHelper.DuckPosition.LEFT){

            }
            else if (position == DetectionHelper.DuckPosition.CENTER){
                slideMotor.setTargetPosition(centerPos);
                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while(slideMotor.isBusy()){
                    slideMotor.setPower(slidePower);
                    telemetry.addData("encoder pos", slideMotor.getCurrentPosition());
                    telemetry.update();
                }
                slideMotor.setPower(0);
            }
            else{
                slideMotor.setTargetPosition(rightPos);
                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while(slideMotor.isBusy()){
                    slideMotor.setPower(slidePower);
                    telemetry.addData("encoder pos", slideMotor.getCurrentPosition());
                    telemetry.update();
                }
                slideMotor.setPower(0);
            }
            dumperServo.setPosition(dumperGoingUp);
            try {
                Thread.sleep(250);
            } catch(InterruptedException E){

            }

            intake.setPower(-0.55);
            try {
                Thread.sleep(1200);
            } catch(InterruptedException E){

            }
            intake.setPower(0);

            slideMotor.setTargetPosition(0);
            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while(slideMotor.isBusy()){
                slideMotor.setPower(-slidePower);
                telemetry.addData("encoder pos", slideMotor.getCurrentPosition());
                telemetry.update();
            }
            slideMotor.setPower(0);

            navigate.navigate(-7, Constants2022.Direction.STRAIGHT, 0, -0.99, backLeft, backRight, frontRight, frontLeft, imu, telemetry, header, false);

            navigate.navigate(0, Constants2022.Direction.TURN, -90, 0.75, backLeft, backRight, frontRight, frontLeft, imu, telemetry, header, true);

            navigate.navigate(-75, Constants2022.Direction.STRAIGHT,0, 0.98,backLeft,backRight,frontRight,frontLeft,imu, telemetry, header,true);


             */
                break;
            }
        }

        public void initialize(){

            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json";
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.mode = BNO055IMU.SensorMode.IMU;
            imu = hardwareMap.get(BNO055IMU.class,"imu");
            imu.initialize(parameters);
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());


            try {
                webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
                pipeline = new DetectionHelper();
                webcam.setPipeline(pipeline);
                webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
                {
                    @Override
                    public void onOpened()
                    {
                        webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
                    }
                    @Override
                    public void onError(int errorCode)
                    {
                        /*
                         * This will be called if the camera could not be opened
                         */
                    }
                });

            } catch (Exception bad){
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
            carousel = hardwareMap.get(DcMotor.class, "carousel");
            dumperServo = hardwareMap.get(Servo.class,"dumperServo");
            slideMotor = hardwareMap.get(DcMotor.class,"slideMotor");


            frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
            backRight.setDirection(DcMotorSimple.Direction.FORWARD);
            slideMotor.setDirection(DcMotorSimple.Direction.FORWARD);

            slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            //    spintime.reset();
        }

    }












