package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Tank Drive", group = "Control")
public class TankDrive extends LinearOpMode {

    DcMotor frontLeft, frontRight, backLeft, backRight;
    double  powBackLeft, powBackRight, powFrontLeft, powFrontRight;
    double  left_trigger, right_trigger;

    @Override
    public void runOpMode() {

        powBackLeft = powBackRight = powFrontLeft = powFrontRight = 0;
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);


        waitForStart();

        while (opModeIsActive()) {
            // Reverse the y-axis to make joystick going up return 1.0
            powBackRight = 0.0 - this.gamepad1.right_stick_y;
            powFrontRight = 0.0 - this.gamepad1.right_stick_y;
            powFrontLeft = 0.0 - this.gamepad1.left_stick_y;
            powBackLeft = 0.0 - this.gamepad1.left_stick_y;

            // Combine the output of both triggers to get single value for center motor


            telemetry.update();

            frontRight.setPower(powFrontRight);
            frontLeft.setPower(powFrontLeft);
            backRight.setPower(powBackRight);
            backLeft.setPower(powBackLeft);

        }

        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);

        telemetry.addData(">", "Done driving; all motors stopped");
        telemetry.update();

    }
}