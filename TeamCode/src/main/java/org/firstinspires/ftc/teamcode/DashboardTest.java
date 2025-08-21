package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mini_project.hardware.ARM;
import org.firstinspires.ftc.teamcode.mini_project.hardware.GRIP;
import org.firstinspires.ftc.teamcode.mini_project.hardware.PINGER;
import org.firstinspires.ftc.teamcode.mini_project.hardware.WRIST;

import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name = "DashboardTest")
public class DashboardTest extends LinearOpMode {

    Servo _GRIP, _WRIST, _PINGER,ARM_L,ARM_R;
    DcMotorEx RS, HSL, HSR;
    FtcDashboard dashboard;
    public static double arm = 0;
    public static double pinger = 0;
    public static double wrist = 0;
    public static double grip = 0;
    public static boolean _DO_HANG, _DO_GRIP = false;
    public static PIDCoefficients pid = new PIDCoefficients(1,1,1);


    void delay(double d){
        ElapsedTime t = new ElapsedTime();
        t.reset();
        while (t.time(TimeUnit.SECONDS) <= d){

        }
        return;
    }

    void grip(){
        _WRIST.setPosition(WRIST.PICK.value);
        _PINGER.setPosition(PINGER.PICK.value);
        _GRIP.setPosition(GRIP.RELEASE.value);

        delay(0.05);
        ARM_R.setPosition(ARM.PICK.value);
        ARM_L.setPosition(ARM.PICK.value);
        delay(0.05);
        _GRIP.setPosition(GRIP.CATCH.value);
        delay(0.02);

        ARM_R.setPosition(ARM.RESET.value);
        ARM_L.setPosition(ARM.RESET.value);
    }
    void test(){
        ARM_L.setPosition(arm);
        ARM_R.setPosition(arm);
        _GRIP.setPosition(grip);
        _WRIST.setPosition(wrist);
        _PINGER.setPosition(pinger);

    }

    void hang(){
        ARM_L.setPosition(ARM.HANG.value);
        ARM_R.setPosition(ARM.HANG.value);
        _GRIP.setPosition(GRIP.CATCH.value);
        _WRIST.setPosition(WRIST.HANG.value);
        _PINGER.setPosition(PINGER.HANG.value);
        delay(0.5);
        ARM_L.setPosition(ARM.HANG2.value);
        ARM_R.setPosition(ARM.HANG2.value);
        delay(0.1);
//        _GRIP.setPosition(GRIP.RELEASE.value);

    }

    void dc(){
        double y = gamepad1.left_stick_y;
        RS.setPower(-y);
        telemetry.addData("RS: ", RS.getCurrentPosition());

        telemetry.update();

    }

    double integralSum = 0;
    ElapsedTime timerP = new ElapsedTime();
    double lastError = 0;
    double PIDControl(double ref, double state){
        double error = ref - state;
        integralSum += error * timerP.seconds();

        double derivative = (error - lastError) / timerP.seconds();
        lastError = error;

        timerP.reset();

        double output = (error * pid.p) + (derivative * pid.d) + (integralSum * pid.i) + (ref * 5 /*K_f*/);
        return  output;
    }


    @Override
    public void runOpMode() {
        dashboard = FtcDashboard.getInstance();
        dashboard.getTelemetry();
        _GRIP = hardwareMap.get(Servo .class, "GRIP");
        _WRIST = hardwareMap.get(Servo.class, "WRIST");
        _PINGER = hardwareMap.get(Servo.class, "PINGER");
        ARM_L   = hardwareMap.get(Servo.class, "ARM_L");
        ARM_R   = hardwareMap.get(Servo.class, "ARM_R");
        ARM_L.setDirection(Servo.Direction.REVERSE);
        RS = hardwareMap.get(DcMotorEx.class,"RS");
        RS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        HSL = hardwareMap.get(DcMotorEx.class, "HSL");
        HSR = hardwareMap.get(DcMotorEx.class, "HSR");
        HSL.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        if (opModeIsActive()) {
            // Pre-run

            if(_DO_HANG){
                hang();
            }
            else  if(_DO_GRIP){
                grip();
            }
            else {
//                test();
            }
            while (opModeIsActive()) {
                // OpMode loop
                HSL.setPower(gamepad1.left_stick_y);
                HSR.setPower(gamepad1.left_stick_y);
                if(gamepad1.a){
                    hang();
                }
                if(gamepad1.b){
                    grip();
                }
                if(gamepad1.x){
                    _GRIP.setPosition(GRIP.RELEASE.value);
                }

            }
        }
    }
}
