package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;




@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="PivotTest", group="teleop")
public class ArmTest extends LinearOpMode {

    ElapsedTime a_time = new ElapsedTime();
    ElapsedTime b_time = new ElapsedTime();
    ElapsedTime y_time = new ElapsedTime();

    Servo pivot;

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        waitForStart();
        while (opModeIsActive()) {

            if(gamepad1.a && a_time.seconds()>=0.25){
                pivot.setPosition(0);
            }
            if(gamepad1.b && b_time.seconds()>=0.25){
                pivot.setPosition(0.5);
            }
            if(gamepad1.y && y_time.seconds()>=0.25){
                pivot.setPosition(1);
            }

        }

    }

    public void initialize(){

        pivot = hardwareMap.get(Servo.class,"pivot");
        pivot.setPosition(0.05);

    }
}
