package org.firstinspires.ftc.teamcode.TEST;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Test_06_10")
public class Test_06_10 extends LinearOpMode {

    Servo servo;

    @Override
    public void runOpMode() {
        waitForStart();
        if (opModeIsActive()) {
            // Pre-run
            servo = hardwareMap.get(Servo.class,"S");

            servo.setDirection(Servo.Direction.REVERSE);

            telemetry.addData("adf",servo.getPosition());

            while (opModeIsActive()) {
                // OpMode loop

            }
        }
    }
}
