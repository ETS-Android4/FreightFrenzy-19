package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.helper.DetectionHelper;
import org.firstinspires.ftc.teamcode.helper.NavigationHelper;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="AUTO ALL", group="auto")
public class AutoAll extends LinearOpMode {

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    DcMotor intake;
    DcMotor carousel;
    Servo dumperServo;
    OpenCvCamera webcam;
    float header;

    final double dumperDump = 0.35;
    final double dumperGoingUp = 0.65;
    final double dumperFirstLevel = 0.70;
    final double dumperIntaking = 0.85;

    DetectionHelper pipeline;
    NavigationHelper navigate = new NavigationHelper();
    ElapsedTime spinTime = new ElapsedTime();
    BNO055IMU imu;

    boolean completeConfig = false;
    boolean red = false;
    boolean blue = false;
    boolean near = false;
    boolean far = false;

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
    }

    public void initialize(){

        while(!completeConfig){
            if(gamepad1.x){
                if(!red){
                    red = true;
                }
                else if(red){
                    red = false;
                }
                telemetry.addData("RED", red);
                telemetry.update();
            }
            if(gamepad1.b){
                if(!blue){
                    blue = true;
                }
                else if(blue){
                    blue = false;
                }
                telemetry.addData("BLUE", blue);
                telemetry.update();
            }
            if(gamepad1.dpad_up){
                if(!near){
                    near = true;
                }
                else if(near){
                    near = false;
                }
                telemetry.addData("NEAR", near);
                telemetry.update();
            }
            if(gamepad1.dpad_down){
                if(!far){
                    far = true;
                }
                else if(far){
                    far = false;
                }
                telemetry.addData("FAR", red);
                telemetry.update();
            }
            if(gamepad1.y){
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

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        intake = hardwareMap.get(DcMotor.class, "intake");
        carousel = hardwareMap.get(DcMotor.class, "carousel");
        dumperServo = hardwareMap.get(Servo.class,"dumperServo");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        dumperServo.setPosition(dumperGoingUp);

    }

    public void Path(){

        if (blue) {

            if (near) {


            }


        }

        else if (red) {


        }

    }

    public void Carousel(){

    }

    public void Placing(){

    }

}
