package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mini_project.hardware.pose.ARM;
import org.firstinspires.ftc.teamcode.mini_project.hardware.pose.GRIP;
import org.firstinspires.ftc.teamcode.mini_project.hardware.pose.PINGER;
import org.firstinspires.ftc.teamcode.mini_project.hardware.pose.WRIST;

@Config
@TeleOp(name = "DashboardTest")
public class DashboardTest extends LinearOpMode {

    public static double arm,wrist,pinger,grip = 0;
    public static boolean doGrip, doHang, doCollect, fl,fr,rl,rr = false;
    Servo ARML,ARMR, WRIST, PINGER, GRIP;
    DcMotor FL,FR,RL,RR;

    FtcDashboard dashboard;

    void delay(double t){
        ElapsedTime time = new ElapsedTime();
        time.reset();
        while (time.seconds() <= t){

        }
        return;
    }

    public void grip(){
        WRIST.setPosition(org.firstinspires.ftc.teamcode.mini_project.hardware.pose.WRIST.PICK.value);
        PINGER.setPosition(org.firstinspires.ftc.teamcode.mini_project.hardware.pose.PINGER.PICK.value);
        GRIP.setPosition(org.firstinspires.ftc.teamcode.mini_project.hardware.pose.GRIP.RELEASE.value);

        delay(0.5);
        ARMR.setPosition(ARM.PICK.value);
        ARML.setPosition(ARM.PICK.value);
        delay(0.5);
        GRIP.setPosition(org.firstinspires.ftc.teamcode.mini_project.hardware.pose.GRIP.CATCH.value);
        delay(0.3);

        ARMR.setPosition(ARM.RESET.value);
        ARML.setPosition(ARM.RESET.value);
    }

    public void collect(){
        WRIST.setPosition(org.firstinspires.ftc.teamcode.mini_project.hardware.pose.WRIST.COLLECT.value);
        PINGER.setPosition(org.firstinspires.ftc.teamcode.mini_project.hardware.pose.PINGER.COLLECT.value);
        GRIP.setPosition(org.firstinspires.ftc.teamcode.mini_project.hardware.pose.GRIP.RELEASE.value);

        delay(0.5);
        ARMR.setPosition(ARM.COLLECT.value);
        ARML.setPosition(ARM.COLLECT.value);
        delay(0.5);
        GRIP.setPosition(org.firstinspires.ftc.teamcode.mini_project.hardware.pose.GRIP.CATCH.value);
        delay(0.3);

        ARMR.setPosition(ARM.RESET.value);
        ARML.setPosition(ARM.RESET.value);
    }

    public void hang(){
        ARML.setPosition(ARM.HANG.value);
        ARMR.setPosition(ARM.HANG.value);
        GRIP.setPosition(org.firstinspires.ftc.teamcode.mini_project.hardware.pose.GRIP.CATCH.value);
        WRIST.setPosition(org.firstinspires.ftc.teamcode.mini_project.hardware.pose.WRIST.HANG.value);
        PINGER.setPosition(org.firstinspires.ftc.teamcode.mini_project.hardware.pose.PINGER.HANG.value);
        delay(0.3);
        ARML.setPosition(ARM.HANG2.value);
        ARMR.setPosition(ARM.HANG2.value);
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

        FL = hardwareMap.dcMotor.get("FL");
        FR = hardwareMap.dcMotor.get("FR"); // FR: RR
        RL = hardwareMap.dcMotor.get("RL");
        RR = hardwareMap.dcMotor.get("RR");
        waitForStart();
        if (opModeIsActive()) {
            // Pre-run
            if(doGrip){
                grip();
            }
            else if(doHang){
                hang();
            }
            else if(doCollect){
                collect();
            }
            else {
                ARML.setPosition(arm);
                ARMR.setPosition(arm);
                WRIST.setPosition(wrist);
                PINGER.setPosition(pinger);
                GRIP.setPosition(grip);
            }
            while (opModeIsActive()) {
                // OpMode loop
                if(fl){
                    FL.setPower(1);
                    FR.setPower(0);
                    RL.setPower(0);
                    RR.setPower(0);
                }
                if(fr){
                    FL.setPower(0);
                    FR.setPower(1);
                    RL.setPower(0);
                    RR.setPower(0);
                }
                if(rl){
                    FL.setPower(0);
                    FR.setPower(0);
                    RL.setPower(1);
                    RR.setPower(0);
                }
                if(rr){
                    FL.setPower(0);
                    FR.setPower(0);
                    RL.setPower(0);
                    RR.setPower(1);
                }
            }
        }
    }
}
