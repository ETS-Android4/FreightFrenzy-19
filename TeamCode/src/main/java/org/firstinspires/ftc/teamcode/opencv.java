package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.HashMap;

public class opencv extends LinearOpMode {

    Boolean isBlue = false;
    Boolean isWall = false;
    final double CLAMP_SERVO_IN = 0.7;
    final double HINGE_SERVO_IN = 0.95;
    final double WING_DOWN = 0.6;
    public final double SHOOTER_INTAKE_SERVO_DOWN = 0.62;
    ElapsedTime runtime = new ElapsedTime();
    BNO055IMU imu;

    OpenCvCamera webcam;

    NavigationHelper navigater = new NavigationHelper();

    DetectionHelper pipeline;
    Constants2020.TargetZone box;
    HashMap<String, Object> variableMap = new HashMap<String, Object>();

    public void initialize() {

        isBlue = false;
        isWall = false;

        telemetry.addLine("reached initialization");
        telemetry.update();

        //Initializing the IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        telemetry.addLine("about to initialize webcam");
        telemetry.update();

        try {
            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
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
                public void onError(int errorCode){

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

        createVariableMap();

        telemetry.addLine("about to initialize wobble arm");
        telemetry.update();
        //initialize wobble arm
    }

    private void createVariableMap(){
        variableMap.put(Constants2020.BLUE_FLAG, this.isBlue);
        variableMap.put(Constants2020.WALL_FLAG, this.isWall);

        variableMap.put(Constants2020.TELEMETRY, this.telemetry);
        variableMap.put(Constants2020.IMU, this.imu);

        variableMap.put(Constants2020.WEBCAM, this.webcam);

        variableMap.put(Constants2020.ELAPSEDTIME, this.runtime);

    }

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addLine("RunOpmode Entered");
        telemetry.update();

        try {

            initialize();

            while(!isStopRequested() && !imu.isGyroCalibrated()){
                sleep(50);
                idle();
            }

            telemetry.addData("imu calib status: ", imu.getCalibrationStatus().toString());
            telemetry.update();

            createVariableMap();

            DetectionHelper.RingPosition position = null;

            while(!opModeIsActive()) {

                telemetry.addData("Analysis", pipeline.getAnalysis());
                telemetry.addData("Position", pipeline.getPosition());
                position = pipeline.getPosition();
                telemetry.update();
                // Don't burn CPU cycles busy-looping in this sample
                sleep(50);
            }

            if (position.equals(DetectionHelper.RingPosition.NONE)){
                box = Constants2020.TargetZone.ALPHA;
            }
            if (position.equals(DetectionHelper.RingPosition.ONE)){
                box = Constants2020.TargetZone.BETA;
            }
            if (position.equals(DetectionHelper.RingPosition.FOUR)){
                box = Constants2020.TargetZone.CHARLIE;
            }

            variableMap.put(Constants2020.POSITION, this.box);
            waitForStart();

            if (opModeIsActive()) {
                runtime.reset();
                //int num = detectionLoop();
                telemetry.addData("Number of Rings", position);
                telemetry.update();
                sleep(500);

                //shootingRings.newPowerShoot(variableMap);
                //shootingRings.ringShoot(variableMap);
                //shootingRings.powerShoot(variableMap);
            }

            //reset imu
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.mode = BNO055IMU.SensorMode.IMU;
            imu.initialize(parameters);



            telemetry.addLine("PROGRAM END");
            telemetry.update();


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
    }
    public int detectionLoop(){
        double begin = System.currentTimeMillis()/1000;
        double end = System.currentTimeMillis()/1000;
        DetectionHelper.RingPosition position = null;
        while(end-begin<1.5) {

            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.getPosition());
            position = pipeline.getPosition();
            telemetry.update();
            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
            end = System.currentTimeMillis()/1000;
        }
        if(position.equals(DetectionHelper.RingPosition.FOUR)){
            return 4;
        }
        else if(position.equals(DetectionHelper.RingPosition.ONE)){
            return 1;
        }
        else if(position.equals(DetectionHelper.RingPosition.NONE)){
            return 0;
        }
        return 3;
    }


}
