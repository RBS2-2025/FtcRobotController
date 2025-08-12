package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

@Config
@TeleOp(name = "DashboardTest")
public class DashboardTest extends LinearOpMode {

    FtcDashboard dashboard;
    public static double TARGET_POS = 100;
    public static PIDCoefficients testPID = new PIDCoefficients(0,0,0);
    @Override
    public void runOpMode() {
        dashboard = FtcDashboard.getInstance();
        waitForStart();
        if (opModeIsActive()) {
            // Pre-run
            while (opModeIsActive()) {
                // OpMode loop
            }
        }
    }
}
