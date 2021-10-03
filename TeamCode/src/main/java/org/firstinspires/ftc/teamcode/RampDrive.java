package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="Ramp Drive", group="auto")

public class RampDrive extends LinearOpMode {

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    private double regSpeed = 0.5;
    private double startSpeed = 0.1;
    private int time = 5000;


    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        waitForStart();
        frontLeft.setPower(startSpeed);
        frontRight.setPower(startSpeed);
        backLeft.setPower(startSpeed);
        backRight.setPower(startSpeed);
        int count = 0;
        while (startSpeed < regSpeed) {
            startSpeed += 0.1;
            try{
                Thread.sleep(200);
            }
            catch(Exception e){

            }
            frontLeft.setPower(startSpeed);
            frontRight.setPower(startSpeed);
            backLeft.setPower(startSpeed);
            backRight.setPower(startSpeed);


        }
        try{
            Thread.sleep(4000);
        }
        catch(Exception e){

        }
        while (startSpeed > 0) {
            startSpeed -= 0.1;
            try{
                Thread.sleep(200);
            }
            catch(Exception e){

            }
            frontLeft.setPower(startSpeed);
            frontRight.setPower(startSpeed);
            backLeft.setPower(startSpeed);
            backRight.setPower(startSpeed);


        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

    }

    public void initialize(){
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

}










