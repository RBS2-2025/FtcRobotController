package org.firstinspires.ftc.teamcode.mini_project.hardware;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

@TeleOp()
public class Robot extends LinearOpMode {

    //GRIP: 0.24 (catch)
    //WRIST: 0 (hang) 0.5(pick) 0.4 (collection)
    //PINGER: 0 (hang) 0.65 (pick)
    //ARM: 0.65 (hang) 0.65(pick) 0.75 (collection) 1.1(reset)

    public DcMotor FR, FL, RR, RL;
    public DcMotor HRS, HLS, RS;
    public Servo GRIP, WRIST, PINGER, ARM_L, ARM_R;



    public void initialize() {
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        RR = hardwareMap.get(DcMotor.class, "RR");
        RL = hardwareMap.get(DcMotor.class, "RL");

        HRS = hardwareMap.get(DcMotor.class, "HSR");
        HLS = hardwareMap.get(DcMotor.class, "HSL");

        GRIP    = hardwareMap.get(Servo.class, "GRIP");
        WRIST  = hardwareMap.get(Servo.class, "WRIST");
        PINGER = hardwareMap.get(Servo.class, "PINGER");
        ARM_L   = hardwareMap.get(Servo.class, "ARM_L");
        ARM_R   = hardwareMap.get(Servo.class, "ARM_R");


        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        RR.setDirection(DcMotorSimple.Direction.REVERSE);
        HRS.setDirection(DcMotorSimple.Direction.REVERSE);
        ARM_L.setDirection(Servo.Direction.REVERSE);

        for (DcMotor m : new DcMotor[]{FR, FL, RR, RL, HRS,HLS, RS}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void delay(double seconds){
        ElapsedTime time = new ElapsedTime();
        time.reset();
        while (time.time(TimeUnit.SECONDS) <= seconds){

        }
        return;
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
