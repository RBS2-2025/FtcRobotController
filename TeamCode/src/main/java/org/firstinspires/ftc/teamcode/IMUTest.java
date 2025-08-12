package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp(name = "IMUTest")
public class IMUTest extends LinearOpMode {



    IMU imu;

    void setUpIMU(){
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));
        telemetry.addData("IMU","initialized!");
    }



    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        waitForStart();
        if (opModeIsActive()) {
            // Pre-run
            telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
            while (opModeIsActive()) {
                // OpMode loop
            }
        }
    }

}
