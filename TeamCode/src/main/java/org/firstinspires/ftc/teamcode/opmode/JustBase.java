package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="just base", group = "auto")
public class JustBase extends LinearOpMode {
    DcMotor frontRight;
    DcMotor backRight;
    DcMotor frontLeft;
    DcMotor backLeft;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        if(opModeIsActive()){
            frontRight.setTargetPosition(2000);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while(frontRight.isBusy()){
                frontRight.setPower(0.5);
                telemetry.addData("front right motor:" , frontRight.getCurrentPosition());
                telemetry.update();

            }
            frontRight.setPower(0);

            backRight.setTargetPosition(2000);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while(backRight.isBusy()){
                backRight.setPower(0.5);
                telemetry.addData("back right motor:" , backRight.getCurrentPosition());
                telemetry.update();
            }
            backRight.setPower(0);

            frontLeft.setTargetPosition(2000);
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while(frontLeft.isBusy()){
                frontLeft.setPower(0.5);

                telemetry.addData("front left motor:" , frontLeft.getCurrentPosition());
                telemetry.update();
            }
            frontLeft.setPower(0);

            backLeft.setTargetPosition(2000);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while(backLeft.isBusy()){
                backLeft.setPower(0.5);

                telemetry.addData("back left motor:" , backLeft.getCurrentPosition());
                telemetry.update();
            }
            backLeft.setPower(0);

        }

    }

    public void initialize(){
         frontRight = hardwareMap.get(DcMotor.class, "frontRight");
         backRight = hardwareMap.get(DcMotor.class, "backRight");
         frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
         backLeft = hardwareMap.get(DcMotor.class, "backLeft");

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }
}
