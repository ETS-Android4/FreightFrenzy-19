package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "SERVO TEST NEW", group = "teleop")
public class ServoTest extends LinearOpMode {

    Servo dumperServo;
    final double dumperUp = 0.5;
    final double dumperDown = 0.8;
    final double dumperInit = 0.99;
    ElapsedTime a_time = new ElapsedTime();
    ElapsedTime b_time = new ElapsedTime();
    ElapsedTime x_time = new ElapsedTime();


    @Override
    public void runOpMode() {
        initialize();
        waitForStart();

        while (opModeIsActive()) {
            // Reverse the y-axis to make joystick going up return 1.0

            if(gamepad2.a&&a_time.seconds()>0.25){
                a_time.reset();
                dumperServo.setPosition(dumperUp);
            }
            if(gamepad2.b&&b_time.seconds()>0.25){
                b_time.reset();
                dumperServo.setPosition(dumperDown);
            }
            if(gamepad2.x&&x_time.seconds()>0.25){
                x_time.reset();
                dumperServo.setPosition(dumperInit);
            }
        }



    }
    public void initialize(){
        dumperServo = hardwareMap.get(Servo.class,"dumperServo");
        dumperServo.setPosition(dumperInit);
    }

}