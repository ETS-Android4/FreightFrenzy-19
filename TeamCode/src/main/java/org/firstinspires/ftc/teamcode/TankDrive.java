package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.motors.GoBILDA5202Series;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Tank Drive", group = "Control")
public class TankDrive extends LinearOpMode {

    DcMotor frontLeft, frontRight, backLeft, backRight;
    Servo intakeServo;
    DcMotor carousel;
    double  powBackLeft, powBackRight, powFrontLeft, powFrontRight, powCarousel;
    double  left_trigger, right_trigger;
    final double intakeOut = 0.45;
    final double intakeIn = 0.85;
    ElapsedTime a_time = new ElapsedTime();
    ElapsedTime b_time = new ElapsedTime();
    ElapsedTime x_time = new ElapsedTime();


    @Override
    public void runOpMode() {

        powBackLeft = powBackRight = powFrontLeft = powFrontRight = powCarousel = 0;
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        carousel = hardwareMap.get(DcMotor.class,"carousel");
       // intakeServo = hardwareMap.get(Servo.class,"intakeServo");

        frontRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        carousel.setDirection(DcMotorSimple.Direction.FORWARD);


        waitForStart();

        while (opModeIsActive()) {
            // Reverse the y-axis to make joystick going up return 1.0
            powBackRight = 0.0 - this.gamepad1.right_stick_y;
            powFrontRight = 0.0 - this.gamepad1.right_stick_y;
            powFrontLeft = 0.0 - this.gamepad1.left_stick_y;
            powBackLeft = 0.0 - this.gamepad1.left_stick_y;
            powCarousel = 0.25;

            frontRight.setPower(powFrontRight);
            frontLeft.setPower(powFrontLeft);
            backRight.setPower(powBackRight);
            backLeft.setPower(powBackLeft);

            if(gamepad2.a&&a_time.seconds()>0.25){
                a_time.reset();
                carousel.setPower(powCarousel);
            }
            if(gamepad2.b&&b_time.seconds()>0.25){
                b_time.reset();
                carousel.setPower(0);
            }
        }

        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);

        telemetry.addData(">", "Done driving; all motors stopped");
        telemetry.update();

    }
}