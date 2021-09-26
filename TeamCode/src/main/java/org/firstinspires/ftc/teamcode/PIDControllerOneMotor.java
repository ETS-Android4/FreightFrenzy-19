package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class PIDControllerOneMotor extends LinearOpMode {

    private DcMotorEx motor1;
    static double speed = 1200;

    public static PIDCoefficients pidCoeffs = new PIDCoefficients(0,0,0);
    public PIDCoefficients pidGains = new PIDCoefficients(0,0,0);

    ElapsedTime PIDTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


    @Override
    public void runOpMode() {
        motor1 = hardwareMap.get(DcMotorEx.class,"motor1");

        motor1.setDirection(DcMotorSimple.Direction.REVERSE);

        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        if (opModeIsActive()){
            while (opModeIsActive()) {

                PID(speed);

                telemetry.update();
            }
        }
    }

    double integral = 0;
    double lastError = 0;

    public void PID(double targetVelocity) {

        PIDTimer.reset();

        double currentVelocity = motor1.getVelocity();

        double error = targetVelocity - currentVelocity;

        integral += error * PIDTimer.time();

        double deltaError = error - lastError;
        double derivative = deltaError/(PIDTimer.time());

        pidGains.p = pidCoeffs.p * error;
        pidGains.i = pidCoeffs.i * integral;
        pidGains.d = pidCoeffs.d * derivative;

        motor1.setVelocity(pidGains.p+ pidGains.i + pidGains.d + targetVelocity);

        lastError = error;
    }


}
