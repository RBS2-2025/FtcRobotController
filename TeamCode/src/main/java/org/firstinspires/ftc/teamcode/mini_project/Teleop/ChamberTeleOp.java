package org.firstinspires.ftc.teamcode.mini_project.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mini_project.Action.Chamber;
import org.firstinspires.ftc.teamcode.mini_project.Action.IMU_Driving;

@TeleOp(name = "ChamberTeleOp")
public class ChamberTeleOp extends LinearOpMode {

    DcMotor rs, hsl, hsr, FL, FR, RL, RR;
    Servo grip, wrist, pinger, arm_L, arm_R;

    void initialize(){
        rs = hardwareMap.dcMotor.get("RS");
        hsl = hardwareMap.dcMotor.get("HSL");
        hsr = hardwareMap.dcMotor.get("HSR");

        FL = hardwareMap.dcMotor.get("FL");
        FR = hardwareMap.dcMotor.get("FR");
        RL = hardwareMap.dcMotor.get("RL");
        RR = hardwareMap.dcMotor.get("RR");

        grip = hardwareMap.servo.get("GRIP");
        wrist = hardwareMap.servo.get("WRIST");
        pinger = hardwareMap.servo.get("PINGER");
        arm_L = hardwareMap.servo.get("ARML");
        arm_R = hardwareMap.servo.get("ARMR");

        arm_L.setDirection(Servo.Direction.REVERSE);
        hsl.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void runOpMode() {
        initialize();
        Chamber chamber = new Chamber(rs,hsl,hsr,grip,wrist,pinger,arm_L,arm_R);
        IMU_Driving driving = new IMU_Driving(FL,FR,RL,RR);
        driving.IMU_INIT();
        waitForStart();
        if (opModeIsActive()) {
            // Pre-run
            while (opModeIsActive()) {
                // OpMode loop
                if(gamepad2.a) chamber.grip();
                if(gamepad2.b) chamber.hang();
                if(gamepad2.y) chamber.collect();
                chamber.moveSlider(gamepad2.left_stick_y);
                driving.mecanumDriveStickView();
            }
        }
    }
}
