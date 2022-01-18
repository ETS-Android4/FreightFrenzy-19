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
    DcMotor slideMotor;
    DcMotor menaka;
    Servo dumperServo;
  //  Servo capServo;
    Servo arm;
    BNO055IMU imu;
    final double dumperDump = 0.6;
    final double dumperGoingUp = 0.67;
    //0.8
    final double dumperIntaking = 0.83;
    final double slidePower = 0.95;
    final double capDown = 0.75;
    final double capUp = 0.45;
    double position;
    int currPos = 0;
    int targetPos = 1300;
    ElapsedTime a_time = new ElapsedTime();
    ElapsedTime b2_time = new ElapsedTime();
    ElapsedTime x2_time = new ElapsedTime();
    ElapsedTime b_time = new ElapsedTime();
    ElapsedTime x_time = new ElapsedTime();
    ElapsedTime y_time = new ElapsedTime();
    ElapsedTime rb_time = new ElapsedTime();
    ElapsedTime dpadup_time = new ElapsedTime();
    ElapsedTime dpaddown_time = new ElapsedTime();
    ElapsedTime dpadup2_time = new ElapsedTime();
    ElapsedTime dpaddown2_time = new ElapsedTime();
    boolean intakeOn = false;
    boolean extakeOn = false;
    boolean carouselOn = false;
    boolean slowMode = false;
    boolean endGame = false;
    boolean capUpp = true;

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

        menaka = hardwareMap.get(DcMotor.class, "menaka");
        menaka.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        menaka.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        menaka.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        arm = hardwareMap.get(Servo.class, "arm");
        arm.setPosition(0);


/*
        intake = hardwareMap.get(DcMotor.class, "intake");
        carousel = hardwareMap.get(DcMotor.class, "carousel");
        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");

        dumperServo = hardwareMap.get(Servo.class,"dumperServo");
        dumperServo.setPosition(dumperIntaking);
       // capServo = hardwareMap.get(Servo.class, "capServo");
     //   capServo.setPosition(0.3);
        position = 0.3;

 */

        //FORWARD,FORWAD, REVERSE, REVERSE (FORWARD/BACK WAS GOOD AND TURNS/STRAFES WERE FLIPPED)
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*

        slideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

         */


    }

    private class AttachmentsThread extends Thread {

        public void AttachmentsThread() {
            this.setName("Attachments Thread");
        }

        @Override
        public void run() {
            try {
                while (!isInterrupted()) {
                    if(gamepad1.dpad_up && dpadup_time.seconds() >= 0.25){
                        dpadup_time.reset();
                        currPos += targetPos;
                        slideMotor.setTargetPosition(currPos);
                        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        while(slideMotor.isBusy()){
                            slideMotor.setPower(slidePower);
                            telemetry.addData("encoder pos:", slideMotor.getCurrentPosition());
                            telemetry.update();
                        }
                        slideMotor.setPower(0);
                    }
                    if(gamepad1.dpad_down && dpaddown_time.seconds() >= 0.25){
                        dpaddown_time.reset();
                        if(endGame){
                            currPos-=500;
                        }
                        else{
                            currPos = 0;
                        }
                        slideMotor.setTargetPosition(currPos);
                        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        while(slideMotor.isBusy()){
                            slideMotor.setPower(-slidePower);
                            telemetry.addData("encoder pos", slideMotor.getCurrentPosition());
                            telemetry.update();
                        }
                        slideMotor.setPower(0);
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

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        //Thread attachments = new TeleOp.AttachmentsThread();
        waitForStart();
        //attachments.start();
        while(opModeIsActive()) {



            /*
            if(gamepad1.right_bumper && rb_time.seconds() >= 0.25){
                telemetry.addLine("hit right bumper lol");
                telemetry.update();
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                rb_time.reset();
                if(capUpp){
                    capServo.setPosition(capDown);
                    capUpp = false;
                }
                if(!capUpp){
                    capServo.setPosition(capUp);
                    capUpp = true;
                }
            }

             */

            /*if(gamepad1.y && y_time.seconds() >= 0.25){
                y_time.reset();
                if(!endGame){
                    endGame = true;
                    targetPos = 500;
                    telemetry.addLine("endgame");
                    telemetry.update();

                }else{
                    endGame = false;
                    targetPos = 1200;
                    telemetry.addLine("no endgame");
                    telemetry.update();

                }

            }



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
                    carousel.setPower(-0.65);
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

            if(gamepad2.b && b2_time.seconds()>0.25){
                b2_time.reset();
                if(!carouselOn){
                    carousel.setPower(0.65);
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

            if(gamepad2.right_bumper && dpadup_time.seconds()>0.25){
                dpadup_time.reset();
                if(!intakeOn){
                    intake.setPower(0.8);
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
                    intake.setPower(-0.8);
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


            if (gamepad2.y && y_time.seconds() > 0.25) {
                y_time.reset();



                turnTest(90, 0.5);
            }

             */

            if (gamepad1.dpad_up) {

                arm.setPosition(0.99);
                try {
                    sleep(1000);
                } catch (Exception e) {

                }

            }

            if (gamepad1.dpad_down) {


                arm.setPosition(0);
                try {
                    sleep(500);
                } catch (Exception e) {

                }
            }

            menaka.setPower(gamepad1.left_stick_x);

            if(gamepad1.b){
                menaka.setTargetPosition(1000);
                menaka.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while(menaka.isBusy()){
                    menaka.setPower(0.3);

                }
                menaka.setPower(0);
            }

            if(gamepad1.x){
                menaka.setTargetPosition(-1000);
                menaka.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while(menaka.isBusy()){
                    menaka.setPower(-0.3);
                }
                menaka.setPower(0);
            }

            if(gamepad1.a){
                int turn = menaka.getCurrentPosition();
                menaka.setTargetPosition(0);
                menaka.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while(menaka.isBusy()){
                    if(turn>0){
                        menaka.setPower(-0.3);
                    }
                    else{
                        menaka.setPower(0.3);
                    }

                }
                menaka.setPower(0);
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

                telemetry.addData("imu:", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                        AngleUnit.DEGREES).firstAngle);
                telemetry.update();
            }
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

