package org.firstinspires.ftc.teamcode.mini_project.Action;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.mini_project.hardware.Robot;

public class Chamber_Action extends LinearOpMode {




    Robot robot = new Robot(hardwareMap);
    public void initialize() {

        robot.HRS.setTargetPosition(0);
        robot.HLS.setTargetPosition(0);
        robot.RS.setTargetPosition(0);

        robot.ARM_L.setPosition(robot.ArmPos);
        robot.ARM_R.setPosition(robot.ArmPos);
        robot.WRIST.setPosition(robot.WristPos);
        robot.PINGER.setPosition(robot.PingerPos);
        robot.GRIP.setPosition(robot.GripPos);
    }

    public void Grip(){

        robot.HRS.setTargetPosition(0);
        robot.HLS.setTargetPosition(0);
        robot.RS.setTargetPosition(0);

        robot.ARM_L.setPosition(robot.ArmPos);
        robot.ARM_R.setPosition(robot.ArmPos);
        robot.WRIST.setPosition(robot.WristPos);
        robot.PINGER.setPosition(robot.PingerPos);
        robot.GRIP.setPosition(robot.GripPos);
    }

    public void Hanging() {

        robot.RS.setTargetPosition(0);
        robot.HRS.setTargetPosition(0);
        robot.HLS.setTargetPosition(0);

        robot.ARM_L.setPosition(robot.ArmPos);
        robot.ARM_R.setPosition(robot.ArmPos);
        robot.WRIST.setPosition(robot.WristPos);
        robot.PINGER.setPosition(robot.PingerPos);
        robot.GRIP.setPosition(robot.GripPos);

    }
    public void runOpMode() throws InterruptedException {

    }
}









