package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
    DcMotor alexCarushow;
    DcMotor pivot;
    DcMotor intake;
    Servo dumperServo;
    Servo arm;
    Servo cap;

    BNO055IMU imu;
    double position;
    int rotatePosition = 0;
    boolean slowMode;
    boolean intakeOn;
    boolean extakeOn;
    boolean boxUp;
    boolean dumpPressed;
    boolean capUp;
    boolean carouselTurn = false;
    ElapsedTime left_bumper = new ElapsedTime();
    ElapsedTime left_bumper_2 = new ElapsedTime();
    ElapsedTime right_bumper_time = new ElapsedTime();
    ElapsedTime right_bumper_time2 = new ElapsedTime();
    ElapsedTime b_time = new ElapsedTime();
    ElapsedTime x_time = new ElapsedTime();
    ElapsedTime y_time = new ElapsedTime();
    ElapsedTime back_time = new ElapsedTime();

    int i = 1;

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

        alexCarushow = hardwareMap.get(DcMotor.class, "alexCarushow");
        intake = hardwareMap.get(DcMotor.class, "intake");
        dumperServo = hardwareMap.get(Servo.class,"dumperServo");
        pivot = hardwareMap.get(DcMotor.class, "pivot");
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        cap = hardwareMap.get(Servo.class, "cap");
        arm = hardwareMap.get(Servo.class, "arm");

        arm.setPosition(1);
        cap.setPosition(0.5);

        dumperServo.setPosition(0.7);




        //FORWARD,FORWAD, REVERSE, REVERSE (FORWARD/BACK WAS GOOD AND TURNS/STRAFExcS WERE FLIPPED)


        //frontright
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        //backleft
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        //backright
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //frontleft
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeOn = false;
        extakeOn = false;
        dumpPressed = false;
        boxUp = false;
        capUp = false;

    }

    private class AttachmentsThread extends Thread {

        public void AttachmentsThread() {
            this.setName("Attachments Thread");
        }

        @Override
        public void run() {
            try {
                while (!isInterrupted()) {

                    //carousel
                    if(gamepad2.b && b_time.seconds()>0.25){
                        b_time.reset();
                        if(!carouselTurn){
                            carouselTurn = true;
                            alexCarushow.setPower(0.5);
                        }
                        else{
                            carouselTurn = false;
                            alexCarushow.setPower(0);
                        }

                    }

                    //carousel other direction
                    if(gamepad2.x && x_time.seconds()>0.25){
                        x_time.reset();
                        if(!carouselTurn){
                            carouselTurn=true;
                            alexCarushow.setPower(-0.5);
                        }
                        else{
                            carouselTurn = false;
                            alexCarushow.setPower(0);
                        }
                    }

                    //up position arm
                    if (gamepad1.dpad_up) {
                        position=0.1;
                        arm.setPosition(position);
                        try {
                            sleep(1000);
                        } catch (Exception e) {

                        }
                    }

                    //intake position arm
                    if(gamepad1.dpad_right) {
                        position = 1;
                        arm.setPosition(position);
                        try{
                            sleep(1000);

                        } catch (Exception e) {

                        }
                    }

                    //low position arm
                    if (gamepad1.dpad_down) {
                        position=0.8;
                        arm.setPosition(position);
                        try {
                            sleep(1000);
                        } catch (Exception e) {

                        }
                    }

                    //middle position arm
                    if(gamepad1.dpad_left){
                        position = 0.3;
                        arm.setPosition(position);
                        try{
                            sleep(1000);
                        } catch (Exception e){

                        }
                    }

                    //intake
                    if (gamepad2.right_bumper && right_bumper_time2.seconds()>0.25) {
                        right_bumper_time2.reset();
                        intake.setPower(0.6);
                        if(intakeOn==false){
                            intake.setPower(0.6);
                            intakeOn=true;
                            extakeOn=false;
                        }
                        else{
                            dumperServo.setPosition(0.7);
                            try {
                                sleep(200);
                            } catch (Exception e) {

                            }
                            intake.setPower(0);
                            extakeOn=false;
                            intakeOn=false;
                        }
                    }

                    if(gamepad2.y&&y_time.seconds()>0.25){
                        y_time.reset();
                        if(capUp==false){
                            cap.setPosition(0.3);
                            capUp = true;
                        }
                        else{
                            cap.setPosition(0.7);
                            capUp=false;
                        }
                    }

                    //box up and down
                    if(gamepad1.right_bumper && right_bumper_time.seconds()>0.25){
                        right_bumper_time.reset();
                        if (boxUp==false){
                            dumperServo.setPosition(0.7);
                            boxUp = true;
                        }
                        else{
                            dumperServo.setPosition(0.6);
                            boxUp = false;
                        }
                    }

                    //outtake
                    if(gamepad1.left_bumper && left_bumper.seconds()>0.25){
                        left_bumper.reset();
                        if(!extakeOn){
                            intake.setPower(-0.4);
                            extakeOn=true;
                            intakeOn=false;
                        }
                        else{
                            intake.setPower(0);
                            extakeOn=false;
                            intakeOn=false;
                        }
                    }

                    //right position pivot
                    if(gamepad1.b){
                        pivot.setTargetPosition(625);
                        rotatePosition = 625;
                        pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        //int movement = Math.abs(pivot.getCurrentPosition()-625);
                        while(pivot.isBusy()){
                            //int fraction = Math.abs((pivot.getCurrentPosition()-625)/movement);
                            pivot.setPower(0.5);
                        }
                        pivot.setPower(0);
                    }

                    //back position pivot
                    if(gamepad1.y){
                        pivot.setTargetPosition(1400);
                        rotatePosition = 1400;
                        pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        while(pivot.isBusy()){
                            pivot.setPower(0.5);

                        }
                        pivot.setPower(0);
                    }

                    //left position pivot
                    if(gamepad1.x){
                        pivot.setTargetPosition(-625);
                        rotatePosition = -625;
                        pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        //int movement = Math.abs(pivot.getCurrentPosition()+625);
                        while(pivot.isBusy()){
                            //int fraction = Math.abs((pivot.getCurrentPosition()+625)/movement);
                            pivot.setPower(-0.5);
                        }
                        pivot.setPower(0);
                    }

                    //neutral position pivot
                    if(gamepad1.a){
                        int turn = pivot.getCurrentPosition();
                        pivot.setTargetPosition(0);
                        rotatePosition = 0;
                        pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        while(pivot.isBusy()){
                            if(turn>0){
                                pivot.setPower(-0.5);
                            }
                            else{
                                pivot.setPower(0.5);
                            }

                        }
                        pivot.setPower(0);
                    }

                    if(gamepad1.left_trigger>0.2){
                        if(i==1){
                            position = 0.3;
                            arm.setPosition(position);
                            try{
                                sleep(1000);
                            } catch (Exception e){

                            }
                            pivot.setTargetPosition(625);
                            rotatePosition = 625;
                            pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            int movement = Math.abs(pivot.getCurrentPosition()-625);
                            while(pivot.isBusy()){
                                int fraction = Math.abs((pivot.getCurrentPosition()-625)/movement);
                                pivot.setPower(0.3);
                            }
                            pivot.setPower(0);
                            position=0.6;
                            arm.setPosition(position);
                            try {
                                sleep(1000);
                            } catch (Exception e) {

                            }
                            i=0;
                        }
                        if(i==2){
                            position = 0.3;
                            arm.setPosition(position);
                            try{
                                sleep(1000);
                            } catch (Exception e){

                            }
                            pivot.setTargetPosition(0);
                            rotatePosition = 0;
                            pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            int movement = Math.abs(pivot.getCurrentPosition()-0);
                            while(pivot.isBusy()){
                                int fraction = Math.abs((pivot.getCurrentPosition()-0)/movement);
                                pivot.setPower(0.3);
                            }
                            pivot.setPower(0);
                            dumperServo.setPosition(0.6);
                            boxUp = false;
                            try{
                                sleep(500);

                            } catch (Exception e) {

                            }
                            position = 1;
                            arm.setPosition(position);
                            try{
                                sleep(1000);

                            } catch (Exception e) {

                            }
                            i = 1;

                        }
                    }

                    if(gamepad1.right_trigger>0.2){
                        if(i==1){
                            position = 0.3;
                            arm.setPosition(position);
                            try{
                                sleep(1000);
                            } catch (Exception e){

                            }
                            pivot.setTargetPosition(-625);
                            rotatePosition = -625;
                            pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            int movement = Math.abs(pivot.getCurrentPosition()+625);
                            while(pivot.isBusy()){
                                int fraction = Math.abs((pivot.getCurrentPosition()+625)/movement);
                                pivot.setPower(-0.3);
                            }
                            pivot.setPower(0);
                            position=0.6;
                            arm.setPosition(position);
                            try {
                                sleep(1000);
                            } catch (Exception e) {

                            }
                            i=2;
                        }
                        else if(i==0){
                            position = 0.3;
                            arm.setPosition(position);
                            try{
                                sleep(1000);
                            } catch (Exception e){

                            }

                            pivot.setTargetPosition(0);
                            rotatePosition = 0;
                            pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            int movement = Math.abs(pivot.getCurrentPosition());
                            while(pivot.isBusy()){
                                int fraction = Math.abs((pivot.getCurrentPosition())/movement);
                                pivot.setPower(-0.3);
                            }
                            pivot.setPower(0);
                            dumperServo.setPosition(0.6);
                            boxUp = false;
                            try{
                                sleep(500);

                            } catch (Exception e) {

                            }
                            position = 1;
                            arm.setPosition(position);
                            try{
                                sleep(1000);

                            } catch (Exception e) {

                            }
                            i = 1;
                        }
                    }

                    if(gamepad1.back && back_time.seconds()>0.25){
                        back_time.reset();
                        dumperServo.setPosition(0.2
                        );
                    }

                    pivot.setTargetPosition(rotatePosition);
                    int diff = rotatePosition-pivot.getCurrentPosition();
                    if(diff>20){
                        pivot.setPower(0.1);
                    }
                    else if(diff < -20){
                        pivot.setPower(-0.1);
                    }
                    else{
                        pivot.setPower(0);
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
            double rotation = gamepad2.right_stick_x * 0.4;
            double fLPower = 0;
            double bLPower = 0;
            double fRPower = 0;
            double bRPower = 0;

            /*
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

             */
            fLPower = radius * Math.cos(angle) + rotation;
            bLPower = radius * Math.sin(angle) + rotation;
            fRPower = radius * Math.sin(angle) - rotation;
            bRPower = radius * Math.cos(angle) - rotation;

            frontLeftMotor.setPower((fLPower) * scale);
            backLeftMotor.setPower((bLPower) * scale);
            frontRightMotor.setPower((fRPower) * scale);
            backRightMotor.setPower((bRPower) * scale);
        }




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
            if(gamepad2.dpad_up){
                slowMode=false;

            }

            if (slowMode) {
                    mecanumDrive(0.35);
                } else {
                    mecanumDrive(1);
                }


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

