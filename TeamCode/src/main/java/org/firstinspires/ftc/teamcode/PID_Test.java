package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mini_project.hardware.ARM;
import org.firstinspires.ftc.teamcode.mini_project.hardware.GRIP;
import org.firstinspires.ftc.teamcode.mini_project.hardware.PINGER;
import org.firstinspires.ftc.teamcode.mini_project.hardware.WRIST;

import java.util.Dictionary;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name = "PID_Test")
public class PID_Test extends LinearOpMode {

    public static boolean hang = false;
    public static boolean pick = false;
    public static boolean collect = false;

    public static double grip = 0;
    public static double arm = 0;
    public static double wrist = 0;
    public static double pinger = 0;

    ElapsedTime time = new ElapsedTime();
    Servo gripS,wristS,pingerS,armL,armR;

    public void delay(double seconds){
        ElapsedTime time = new ElapsedTime();
        time.reset();
        while (time.time(TimeUnit.SECONDS) <= seconds){

        }
        return;
    }

    void initialize(){
        gripS = hardwareMap.servo.get("GRIP");
        wristS = hardwareMap.servo.get("WRIST");
        pingerS =  hardwareMap.servo.get("PINGER");
        armL =  hardwareMap.servo.get("ARM_L");
        armR =  hardwareMap.servo.get("ARM_R");
        armL.setDirection(Servo.Direction.REVERSE);
    }

    public void Grip(){



        wristS.setPosition(WRIST.PICK.value);
        pingerS.setPosition(PINGER.PICK.value);
        gripS.setPosition(GRIP.RELEASE.value);

        delay(0.1);
        armL.setPosition(ARM.PICK.value);
        armR.setPosition(ARM.PICK.value);

        delay(0.2);
        gripS.setPosition(GRIP.CATCH.value);
        delay(0.1);

        armL.setPosition(ARM.RESET.value);
        armR.setPosition(ARM.RESET.value);
        wristS.setPosition(WRIST.HANG.value);
        pingerS.setPosition(PINGER.HANG.value);
    }

    public void Collect(){




        wristS.setPosition(WRIST.COLLECT.value);
        pingerS.setPosition(PINGER.COLLECT.value);

        delay(0.1);
        armL.setPosition(ARM.COLLECT.value);
        armR.setPosition(ARM.COLLECT.value);

        delay(0.2);
        gripS.setPosition(GRIP.CATCH.value);
        delay(0.1);

        armL.setPosition(ARM.HANG.value);
        armR.setPosition(ARM.HANG.value);
        delay(0.1);
        wristS.setPosition(WRIST.HANG.value);
        pingerS.setPosition(PINGER.HANG.value);
    }

    public void Hanging() {

        armL.setPosition(ARM.RESET.value);
        armR.setPosition(ARM.RESET.value);
        wristS.setPosition(WRIST.HANG.value);
        pingerS.setPosition(PINGER.HANG.value);
    }

    @Override
    public void runOpMode() {
        waitForStart();
        initialize();
        if (opModeIsActive()) {
            // Pre-run
            if(hang) Hanging();
            else if(collect) Collect();
            else if(pick) Grip();
            else{
                armL.setPosition(ARM.PICK.value);
                armR.setPosition(ARM.PICK.value);

                delay(0.5);

                armL.setPosition(ARM.RESET.value);
                armR.setPosition(ARM.RESET.value);
            }
            while (opModeIsActive()) {
                // OpMode loop
            }
        }
    }
}
