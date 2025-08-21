package org.firstinspires.ftc.teamcode.mini_project.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mini_project.Action.IMU_Driving;
import org.firstinspires.ftc.teamcode.mini_project.Action.Chamber_Action;
import org.firstinspires.ftc.teamcode.mini_project.hardware.ARM;
import org.firstinspires.ftc.teamcode.mini_project.hardware.GRIP;
import org.firstinspires.ftc.teamcode.mini_project.hardware.PINGER;
import org.firstinspires.ftc.teamcode.mini_project.hardware.Robot;
import org.firstinspires.ftc.teamcode.mini_project.hardware.WRIST;
@TeleOp
public class Teleop_total_version extends LinearOpMode{

    IMU_Driving imu_driving = new IMU_Driving();
    Chamber_Action chamber_action = new Chamber_Action();
    public DcMotor FR, FL, RR, RL;
    public DcMotor HRS, HLS, RS;
    public Servo GRIP, WRIST, PINGER, ARM_L, ARM_R;
    public IMU imu_IMU;


    @Override
    public void runOpMode() {

        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        RR = hardwareMap.get(DcMotor.class, "RR");
        RL = hardwareMap.get(DcMotor.class, "RL");

        HRS = hardwareMap.get(DcMotor.class, "HSR");
        HLS = hardwareMap.get(DcMotor.class, "HSL");
        RS = hardwareMap.get(DcMotor.class, "RS");

        GRIP    = hardwareMap.get(Servo.class, "GRIP");
        WRIST  = hardwareMap.get(Servo.class, "WRIST");
        PINGER = hardwareMap.get(Servo.class, "PINGER");
        ARM_L   = hardwareMap.get(Servo.class, "ARM_L");
        ARM_R   = hardwareMap.get(Servo.class, "ARM_R");


        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        RR.setDirection(DcMotorSimple.Direction.REVERSE);
        HRS.setDirection(DcMotorSimple.Direction.REVERSE);
        ARM_L.setDirection(Servo.Direction.REVERSE);

        imu_IMU = hardwareMap.get(IMU.class, "imu");
        imu_driving.IMU_INIT();


        for (DcMotor m : new DcMotor[]{FR, FL, RR, RL, HRS,HLS, RS}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        waitForStart();


        while (opModeIsActive()) {
            // Put loop blocks here.
            imu_driving.getYaw();
            imu_driving.mecanumDriveStickView();

            if(gamepad1.a) {

                chamber_action.Hanging();

            }




            if(gamepad1.right_bumper) {

                imu_driving.speed = 0.5;
            }


        }


    }










    }