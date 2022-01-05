package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
public class ArmTest extends LinearOpMode {

    CRServo arm;

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        waitForStart();
        while (opModeIsActive()) {

            arm.setPower(0.05);

            if (gamepad1.dpad_up) {

                arm.setPower(0.5);
                try {
                    sleep(1000);
                } catch (Exception e) {

                }

            }

            if (gamepad1.dpad_down) {


                arm.setPower(-0.5);
                try {
                    sleep(500);
                } catch (Exception e) {

                }


            }

        }


    }

    public void initialize(){

        arm = hardwareMap.get(CRServo.class, "arm");

    }
}
