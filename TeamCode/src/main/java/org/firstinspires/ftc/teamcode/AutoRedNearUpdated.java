package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="AUTO RED NEAR v2", group="auto")

public class AutoRedNearUpdated extends LinearOpMode {

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    DcMotor intake;
    DcMotor carousel;
    DcMotor slideMotor;
    Servo dumperServo;
    Servo capServo;
    OpenCvCamera webcam;
    float header;

    double currentAngle;
    Orientation lastAngles = new Orientation();

    final double dumperDump = 0.35;
    final double dumperGoingUp = 0.75;
    final double dumperFirstLevel = 0.8;
    final double dumperIntaking = 0.94;
    final double slidePower = 0.95;
    int centerPos = 1000;
    int rightPos = 2000;

    DetectionHelper pipeline;
    NavigationHelper navigate = new NavigationHelper();
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

        if(opModeIsActive()){
            telemetry.addData("entering loop", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            telemetry.update();
            header = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                    AngleUnit.DEGREES).firstAngle;


            navigate.navigate(-13, Constants2020.Direction.STRAIGHT,0, -0.5,backLeft,backRight,frontRight,frontLeft,imu, telemetry, header,true);
            try {
                Thread.sleep(500);
            } catch(InterruptedException E){

            }

            navigate.navigate(25, Constants2020.Direction.RIGHT,0,0.5,backLeft,backRight,frontRight,frontLeft,imu,telemetry,header,false);

            navigate.navigate(0, Constants2020.Direction.TURN, -180, 0.5, backLeft, backRight, frontRight, frontLeft, imu, telemetry, header, true);

            navigate.navigate(4, Constants2020.Direction.STRAIGHT,0, -0.5,backLeft,backRight,frontRight,frontLeft,imu, telemetry, header,true);

            try {
                Thread.sleep(500);
            } catch(InterruptedException E){

            }

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
                Thread.sleep(500);
            } catch(InterruptedException E){

            }

            intake.setPower(-0.8);
            try {
                Thread.sleep(2000);
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



            navigate.navigate(-10, Constants2020.Direction.STRAIGHT,0,0.5,backLeft,backRight,frontRight,frontLeft,imu,telemetry,header,false);

            try {
                Thread.sleep(250);
            } catch(InterruptedException E){

            }

            navigate.navigate(0, Constants2020.Direction.TURN,-80,0.5,backLeft,backRight,frontRight,frontLeft,imu,telemetry,header,true);

            try {
                Thread.sleep(500);
            } catch(InterruptedException E){

            }

            navigate.navigate(80, Constants2020.Direction.STRAIGHT,0,0.99,backLeft,backRight,frontRight,frontLeft,imu,telemetry, header, true);

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
        capServo = hardwareMap.get(Servo.class, "capServo");

        dumperServo.setPosition(dumperGoingUp);

        slideMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    private double angleConversion()
    {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double changeInAngle = angles.firstAngle - lastAngles.firstAngle;
        currentAngle += changeInAngle;

        /*
        if (currentAngle > 0)
        {
            currentAngle = ((currentAngle + 180) % (360)) - 180;
        }
        else // currentAngle < 0
        {
            currentAngle = ((currentAngle - 180) % (360)) + 180;
        }
         */

        if (currentAngle > 180){
            currentAngle -= 360;
        } else if(currentAngle < -180){
            currentAngle += 360;
        }

        lastAngles = angles;
        return currentAngle;
    }
    private void TurnUntilAngleReached(final int degrees)
    {
        if (degrees < 0)
        {
            while (true)
            {
                if ((angleConversion() <= degrees)) break;
                telemetry.addData("Angle", angleConversion());
            }
        }

        else // degrees >= 0
        {
            while (true)
            {
                if ((angleConversion() >= degrees)) break;
                telemetry.addData("Angle", angleConversion());
            }
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        sleep(100);
        resetAngle();
    }

    private void resetAngle(){
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currentAngle = 0;
    }

    public void turn(final String direction, final int degrees, int motorPower)
    {
        double flPower = 0, frPower = 0, blPower = 0, brPower = 0;
        resetAngle();

        switch (direction){
            case "left":
                flPower = -0.5 * motorPower;
                frPower = 0.5 * motorPower;
                blPower = -0.5 * motorPower;
                brPower = 0.5 * motorPower;
                break;

            case "right":
                flPower = 0.5 * motorPower;
                frPower = -0.5 * motorPower;
                blPower = 0.5 * motorPower;
                brPower = -0.5 * motorPower;
                break;

            default:
                try {
                    throw new IllegalStateException("Invalid turn direction: " + direction);
                } catch (IllegalStateException e) {
                    e.printStackTrace();
                }
        }

        frontLeft.setPower(flPower);
        frontRight.setPower(frPower);
        backLeft.setPower(blPower);
        backRight.setPower(brPower);

        TurnUntilAngleReached(degrees);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

}










