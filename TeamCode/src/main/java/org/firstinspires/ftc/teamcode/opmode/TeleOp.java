package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="drive", group="teleop")

public class TeleOp extends LinearOpMode {

    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    DcMotor frontLeftMotor;
    DcMotor wijeratne;
    DcMotor menaka;
    Servo arm;
    BNO055IMU imu;
    double position;
    boolean slowMode;
    boolean intakeOn;
    boolean extakeOn;
    ElapsedTime left_bumper = new ElapsedTime();
    ElapsedTime right_bumper = new ElapsedTime();
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

        wijeratne = hardwareMap.get(DcMotor.class, "wijeratne");

        menaka = hardwareMap.get(DcMotor.class, "menaka");
        menaka.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        menaka.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        menaka.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm = hardwareMap.get(Servo.class, "arm");
        arm.setPosition(0);

        //FORWARD,FORWAD, REVERSE, REVERSE (FORWARD/BACK WAS GOOD AND TURNS/STRAFES WERE FLIPPED)
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeOn = false;
        extakeOn = false;

    }

    private class AttachmentsThread extends Thread {

        public void AttachmentsThread() {
            this.setName("Attachments Thread");
        }

        @Override
        public void run() {
            try {
                while (!isInterrupted()) {

                    if (gamepad1.dpad_up) {
                        position=0.1;
                        arm.setPosition(position);
                        try {
                            sleep(1000);
                        } catch (Exception e) {

                        }
                    }

                    if(gamepad1.dpad_right) {
                        position = 0.98;
                        arm.setPosition(position);
                        try{
                            sleep(1000);

                        } catch (Exception e) {

                        }
                    }

                    if (gamepad1.dpad_down) {
                        position=0.9;
                        arm.setPosition(position);
                        try {
                            sleep(1000);
                        } catch (Exception e) {

                        }
                    }
                    if(gamepad1.dpad_left){
                        position = 0.45;
                        arm.setPosition(position);
                        try{
                            sleep(1000);
                        } catch (Exception e){

                        }
                    }

                    telemetry.addData("arm pos: ", position);
                    telemetry.update();

                    if(gamepad1.b){
                        menaka.setTargetPosition(625);
                        menaka.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        int movement = Math.abs(menaka.getCurrentPosition()-625);
                        while(menaka.isBusy()){
                            int fraction = Math.abs((menaka.getCurrentPosition()-625)/movement);
                            menaka.setPower(fraction*0.3+0.6);
                        }
                        menaka.setPower(0);
                    }
                    if(gamepad1.y){
                        menaka.setTargetPosition(1400);
                        menaka.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        while(menaka.isBusy()){
                            menaka.setPower(0.7);

                        }
                        menaka.setPower(0);
                    }

                    if(gamepad1.x){
                        menaka.setTargetPosition(-625);
                        menaka.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        int movement = Math.abs(menaka.getCurrentPosition()+625);
                        while(menaka.isBusy()){
                            int fraction = Math.abs((menaka.getCurrentPosition()+625)/movement);
                            menaka.setPower(fraction*-0.3+0.6);
                        }
                        menaka.setPower(0);
                    }

                    if(gamepad1.a){
                        int turn = menaka.getCurrentPosition();
                        menaka.setTargetPosition(0);
                        menaka.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        while(menaka.isBusy()){
                            if(turn>0){
                                menaka.setPower(-0.7);
                            }
                            else{
                                menaka.setPower(0.7);
                            }

                        }
                        menaka.setPower(0);
                    }

                    if(gamepad2.right_bumper && right_bumper.seconds()>0.25){
                        right_bumper.reset();
                        if(!intakeOn){
                            wijeratne.setPower(0.7);
                            intakeOn=true;
                            extakeOn=false;
                        }
                        else{
                            wijeratne.setPower(0);
                            intakeOn=false;
                            extakeOn=false;
                        }
                    }

                    if(gamepad1.left_bumper && left_bumper.seconds()>0.25){
                        left_bumper.reset();
                        if(!extakeOn){
                            wijeratne.setPower(-0.5);
                            extakeOn=true;
                            intakeOn=false;
                        }
                        else{
                            wijeratne.setPower(0);
                            extakeOn=false;
                            intakeOn=false;
                        }
                    }
                }
                    idle();
            } catch (Exception e) {

            }
        }
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
                double ratioCos=1;
                double rationSin=1;
                fLPower = radius * ratioCos - rotation;
                bLPower = radius * rationSin - rotation;
                fRPower = radius * ratioCos + rotation;
                bRPower = radius * ratioCos + rotation;
            }
            else if( (angle < -5*(Math.PI/12))&& (angle > -7*(Math.PI/12)) ){
                double ratioCos=1;
                double rationSin=1;
                fLPower = radius * ratioCos - rotation;
                bLPower = radius * rationSin - rotation;
                fRPower = radius * rationSin + rotation;
                bRPower = radius * ratioCos + rotation;
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



    double positionArm = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        Thread attachments = new TeleOp.AttachmentsThread();
        waitForStart();
        attachments.start();
        while(opModeIsActive()) {

            if(gamepad2.dpad_down){
                slowMode=true;
            }

            if (slowMode) {
                    //telemetry.addData("speed", 0.35);
                    //telemetry.update();
                    mecanumDrive(0.35);
                } else {
                    //telemetry.addData("speed", 1);
                    //telemetry.update();
                    mecanumDrive(1);
                }
                idle();

            }
        attachments.interrupt();
        }

    public void turnTest(double turn, double speed){

        double error = speed*10*3 - 7;
        double currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                AngleUnit.DEGREES).firstAngle;

        if (turn < 0) {

            double math = currentAngle + turn;

            if (math<-180) {
                math = 180 - Math.abs(-180-math);
            }
            telemetry.addData("goalAngle:", math);
            telemetry.update();
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            while (Math.abs(currentAngle - math) > error) {
                frontLeftMotor.setPower(speed);
                backLeftMotor.setPower(speed);
                frontRightMotor.setPower(-speed);
                backRightMotor.setPower(-speed);

                currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                        AngleUnit.DEGREES).firstAngle;

            }

            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backRightMotor.setPower(0);

        }
        else{
            double math = currentAngle + turn;
            if (math>180) {
                math = -180 + Math.abs(180-math);
            }
            telemetry.addData("goalAngle:", math);
            telemetry.update();
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            while (Math.abs(currentAngle - math) > error) {
                frontLeftMotor.setPower(-speed);
                backLeftMotor.setPower(-speed);
                frontRightMotor.setPower(speed);
                backRightMotor.setPower(speed);

                currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                        AngleUnit.DEGREES).firstAngle;

            }
            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backRightMotor.setPower(0);

        }
    }

}

