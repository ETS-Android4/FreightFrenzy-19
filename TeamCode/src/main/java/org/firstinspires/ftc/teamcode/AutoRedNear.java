package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="AUTO RED NEAR", group="auto")

public class AutoRedNear extends LinearOpMode {

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    OpenCvCamera webcam;
    float header;

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

        telemetry.addData("entering loop", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        telemetry.update();
        header = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                AngleUnit.DEGREES).firstAngle;
        navigate.forwardDrive(18,0.5,backLeft,backRight,frontRight,frontLeft,telemetry, imu,true);
        try {
            Thread.sleep(500);
        } catch(InterruptedException E){

        }
        navigate.navigate(15, Constants2020.Direction.LEFT,0,0.3,backLeft,backRight,frontRight,frontLeft,imu,telemetry,header,false);
        try {
            Thread.sleep(250);
        } catch(InterruptedException E){

        }
        navigate.forwardDrive(-10,0.5,backLeft,backRight,frontRight,frontLeft,telemetry, imu,true);
        try {
            Thread.sleep(250);
        } catch(InterruptedException E){

        }
        navigate.navigate(0, Constants2020.Direction.TURN,90,0.3,backLeft,backRight,frontRight,frontLeft,imu,telemetry, header, true);
        try {
            Thread.sleep(250);
        } catch(InterruptedException E){

        }
        navigate.forwardDrive(40,0.7,backLeft,backRight,frontRight,frontLeft,telemetry, imu,true);


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


        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
    }

}










