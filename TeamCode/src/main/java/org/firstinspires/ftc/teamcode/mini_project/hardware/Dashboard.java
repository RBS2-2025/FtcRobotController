package org.firstinspires.ftc.teamcode.mini_project.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "PosTest")
public class Dashboard extends LinearOpMode {
    Servo ARM_L,ARM_R,GRIP,PINGER,WRIST;
    FtcDashboard dashboard;

    public static double ArmPos = 0;
    public static double GripPos = 0;
    public static double PingerPos = 0;
    public static double WristPos = 0;

    @Override
    public void runOpMode() {
        dashboard = FtcDashboard.getInstance();

        waitForStart();

        ARM_L = hardwareMap.get(Servo.class,"ARM_L");
        ARM_R = hardwareMap.get(Servo.class,"ARM_R");
        GRIP = hardwareMap.get(Servo.class,"GRIP");
        PINGER = hardwareMap.get(Servo.class,"PINGER");
        WRIST = hardwareMap.get(Servo.class,"WRIST");

        if (opModeIsActive()) {
            // Pre-run
            GRIP.setPosition(GripPos);
            PINGER.setPosition(PingerPos);
            WRIST.setPosition(WristPos);

            ARM_L.setPosition(ArmPos);
            ARM_R.setPosition(ArmPos);
            while (opModeIsActive()) {
                // OpMode loop
            }
        }
    }
}
