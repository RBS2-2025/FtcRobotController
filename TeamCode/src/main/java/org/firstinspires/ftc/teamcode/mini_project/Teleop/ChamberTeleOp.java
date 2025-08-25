package org.firstinspires.ftc.teamcode.mini_project.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mini_project.Action.BasciMovement;
import org.firstinspires.ftc.teamcode.mini_project.Action.Chamber;

@Config
@TeleOp(name = "ChamberTeleOp")
public class ChamberTeleOp extends LinearOpMode {

    DcMotor FL, FR, RL, RR;
    DcMotorEx rsr, rsl, hsl, hsr;
    Servo grip, wrist, pinger, arm_L, arm_R;
    FtcDashboard dashboard;

    public static PIDCoefficients pid = new PIDCoefficients(1,1,1);

    void initialize(){
        rsr = hardwareMap.get(DcMotorEx.class,"RSR");
        rsl = hardwareMap.get(DcMotorEx.class,"RSL");
        hsl = hardwareMap.get(DcMotorEx.class,"HSL");
        hsr = hardwareMap.get(DcMotorEx.class,"HSR");

        FL = hardwareMap.dcMotor.get("FL");
        FR = hardwareMap.dcMotor.get("FR");
        RL = hardwareMap.dcMotor.get("RL");
        RR = hardwareMap.dcMotor.get("RR");

        grip = hardwareMap.servo.get("GRIP");
        wrist = hardwareMap.servo.get("WRIST");
        pinger = hardwareMap.servo.get("PINGER");
        arm_L = hardwareMap.servo.get("ARM_L");
        arm_R = hardwareMap.servo.get("ARM_R");

        for(DcMotor motor: new DcMotor[]{FL,FR,RL,RR,hsl,hsr,rsl,rsr}){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }



            arm_L.setDirection(Servo.Direction.REVERSE);
        hsl.setDirection(DcMotorSimple.Direction.REVERSE);
        rsl.setDirection(DcMotorSimple.Direction.REVERSE);

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        RL.setDirection(DcMotorSimple.Direction.REVERSE);



    }

    @Override
    public void runOpMode() {


        dashboard = FtcDashboard.getInstance();
        dashboard.getTelemetry();

        initialize();
        Chamber chamber = new Chamber(rsr,rsl,hsl,hsr,grip,wrist,pinger,arm_L,arm_R,pid);
        BasciMovement driving = new BasciMovement(FL,FR,RL,RR);
        waitForStart();
        if (opModeIsActive()) {
            // Pre-run
//            chamber.moveSliderTo(400,1);
            while (opModeIsActive()) {
                // OpMode loop
                if(gamepad2.a) chamber.grip();
                if(gamepad2.b) chamber.hang();
                if(gamepad2.y) chamber.collect();
                chamber.moveSlider(gamepad2.left_stick_y,1);
                chamber.rotateSlider(gamepad2.right_stick_y);
                driving.move(gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x);
                telemetry.addData("l: ", hsl.getCurrentPosition());
                telemetry.addData("r: ",hsr.getCurrentPosition());
                telemetry.update();
            }
        }
    }
}
