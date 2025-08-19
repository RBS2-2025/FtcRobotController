package org.firstinspires.ftc.teamcode.mini_project.hardware;


import com.qualcomm.robotcore.hardware.*;

public class Robot {
    public DcMotor FR, FL, RR, RL;
    public DcMotor HRS, HLS, RS;
    public Servo GRIP, WRIST, PINGER, ARM_L, ARM_R;
    HardwareMap hw;
    public double ArmPos = 0;
    public double GripPos = 0;
    public double PingerPos = 0;
    public double WristPos = 0;

    public Robot(HardwareMap hardwareMap){
        this.hw = hardwareMap;

        HRS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        HLS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        init();

    }

    public void init() {
        FR = hw.get(DcMotor.class, "FR");
        FL = hw.get(DcMotor.class, "FL");
        RR = hw.get(DcMotor.class, "RR");
        RL = hw.get(DcMotor.class, "RL");

        HRS = hw.get(DcMotor.class, "LRS");
        HLS = hw.get(DcMotor.class, "LLS");

        GRIP    = hw.get(Servo.class, "L_arm");
        WRIST  = hw.get(Servo.class, "L_wrist");
        PINGER = hw.get(Servo.class, "L_pinger");
        ARM_L   = hw.get(Servo.class, "L_claw");
        ARM_R   = hw.get(Servo.class, "L_claw");


        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        RR.setDirection(DcMotorSimple.Direction.REVERSE);
        HRS.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotor m : new DcMotor[]{FR, FL, RR, RL, HRS,HLS, RS}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
}
