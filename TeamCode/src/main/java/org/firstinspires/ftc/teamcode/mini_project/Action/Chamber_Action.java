package org.firstinspires.ftc.teamcode.mini_project.Action;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.mini_project.hardware.ARM;
import org.firstinspires.ftc.teamcode.mini_project.hardware.GRIP;
import org.firstinspires.ftc.teamcode.mini_project.hardware.PINGER;
import org.firstinspires.ftc.teamcode.mini_project.hardware.Robot;
import org.firstinspires.ftc.teamcode.mini_project.hardware.WRIST;

@Config
@TeleOp(name = "Chamber_Action")
public class Chamber_Action extends LinearOpMode {


    Robot robot = new Robot();

    public static boolean hang = false;
    public static boolean pick = false;
    public static boolean collect = false;


    public void initialize() {

        robot.HRS.setTargetPosition(0);
        robot.HLS.setTargetPosition(0);
        robot.RS.setTargetPosition(0);

        robot.ARM_L.setPosition(ARM.RESET.value);
        robot.ARM_R.setPosition(ARM.RESET.value);
        robot.WRIST.setPosition(WRIST.PICK.value);
        robot.PINGER.setPosition(PINGER.HANG.value);
        robot.GRIP.setPosition(GRIP.RELEASE.value);

        robot.delay(0.1);
    }

    public void Grip(){

        robot.HRS.setTargetPosition(0);
        robot.HLS.setTargetPosition(0);
        robot.RS.setTargetPosition(0);


        robot.WRIST.setPosition(WRIST.PICK.value);
        robot.PINGER.setPosition(PINGER.PICK.value);

        robot.delay(0.1);
        robot.ARM_L.setPosition(ARM.PICK.value);
        robot.ARM_R.setPosition(ARM.PICK.value);

        robot.delay(0.2);
        robot.GRIP.setPosition(GRIP.CATCH.value);
        robot.delay(0.1);

        robot.ARM_L.setPosition(ARM.HANG.value);
        robot.ARM_R.setPosition(ARM.HANG.value);
        robot.WRIST.setPosition(WRIST.HANG.value);
        robot.PINGER.setPosition(PINGER.HANG.value);
    }

    public void Collect(){

        robot.HRS.setTargetPosition(0);
        robot.HLS.setTargetPosition(0);
        robot.RS.setTargetPosition(0);


        robot.WRIST.setPosition(WRIST.COLLECT.value);
        robot.PINGER.setPosition(PINGER.COLLECT.value);

        robot.delay(0.1);
        robot.ARM_L.setPosition(ARM.COLLECT.value);
        robot.ARM_R.setPosition(ARM.COLLECT.value);

        robot.delay(0.2);
        robot.GRIP.setPosition(GRIP.CATCH.value);
        robot.delay(0.1);

        robot.ARM_L.setPosition(ARM.HANG.value);
        robot.ARM_R.setPosition(ARM.HANG.value);
        robot.WRIST.setPosition(WRIST.HANG.value);
        robot.PINGER.setPosition(PINGER.HANG.value);
    }

    public void Hanging() {
        robot.RS.setTargetPosition(0);
        robot.HRS.setTargetPosition(0);
        robot.HLS.setTargetPosition(0);

    }


    @Override
    public void runOpMode() {
        waitForStart();
        robot.initialize();
        if(opModeIsActive()){

            if(hang) Hanging();
            if(collect) Collect();
            if(pick) Grip();
            while (opModeIsActive()){

            }
        }
    }
}
