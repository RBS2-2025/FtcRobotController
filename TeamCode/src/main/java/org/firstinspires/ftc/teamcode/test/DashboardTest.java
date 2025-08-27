package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mini_project.Action.Chamber;


@Config
@TeleOp(name = "DashboardTest")
public class DashboardTest extends LinearOpMode {

    public static double arm,wrist,pinger,grip,rotateSpd = 0;

    public static boolean servo, slider, rotation= false;
    Servo ARML,ARMR, WRIST, PINGER, GRIP;
    DcMotor hsl,hsr,rsl,rsr;

    FtcDashboard dashboard;

    void delay(double t){
        ElapsedTime time = new ElapsedTime();
        time.reset();
        while (time.seconds() <= t){

        }
        return;
    }



    @Override
    public void runOpMode() {
        dashboard = FtcDashboard.getInstance();
        ARML = hardwareMap.servo.get("ARM_L");
        ARMR = hardwareMap.servo.get("ARM_R");
        WRIST = hardwareMap.servo.get("WRIST");
        PINGER = hardwareMap.servo.get("PINGER");
        GRIP = hardwareMap.servo.get("GRIP");
        ARML.setDirection(Servo.Direction.REVERSE);

        hsl = hardwareMap.dcMotor.get("HSL");
        hsr = hardwareMap.dcMotor.get("HSR");
        rsl = hardwareMap.dcMotor.get("RSL");
        rsr = hardwareMap.dcMotor.get("RSR");

        hsl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hsr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rsl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rsr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        hsl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hsr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rsl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rsr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rsl.setDirection(DcMotorSimple.Direction.REVERSE);
        hsl.setDirection(DcMotorSimple.Direction.REVERSE);

        Chamber chamber = new Chamber(rsr,rsl,hsl,hsr,GRIP,WRIST,PINGER,ARML,ARMR);
        waitForStart();
        if (opModeIsActive()) {
            // Pre-run

            while (opModeIsActive()) {
                // OpMode loop

                if(servo){
                    ARML.setPosition(arm);
                    ARMR.setPosition(arm);
                    WRIST.setPosition(wrist);
                    PINGER.setPosition(pinger);
                    GRIP.setPosition(grip);
                }
                if(slider){
                    chamber.moveSlider(gamepad2.left_stick_y,rotateSpd);
                }
                if(rotation){
                    chamber.RotateSlider(gamepad2.right_stick_y,opModeIsActive());
                }
                //////////////////////////////////////////////////////////////
                telemetry.addData("hsl: ",hsl.getCurrentPosition());
                telemetry.addData("hsr: ",hsr.getCurrentPosition());
                telemetry.addData("rsl: ",rsl.getCurrentPosition());
                telemetry.addData("rsr: ",rsr.getCurrentPosition());
                telemetry.update();
            }
        }
    }
}
