package org.firstinspires.ftc.teamcode.mini_project.Action;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mini_project.hardware.pose.*;

public class Chamber {
    public Telemetry telemetry;


    DcMotor rsr, rsl, hsl, hsr;
    Servo grip, wrist, pinger, arm_L, arm_R;


    int sliderTargetPos = 0;


    int MIN_HIGH_SLIDER = 0; //TODO
    int MAX_HIGH_SLIDER = 1500; //TODO
    int MIN_ROTATE_SLIDER = 0; //TODO
    int INIT_ROTATE_SLIDER = 75; //TODO
    int MAX_ROTATE_SLIDER = 430; //TODO



    public Chamber(DcMotor rsr, DcMotor rsl, DcMotor hsl, DcMotor hsr, Servo grip, Servo wrist, Servo pinger, Servo arm_L, Servo arm_R){
        this.rsr = rsr;
        this.rsl = rsl;
        this.hsl = hsl;
        this.hsr = hsr;
        this.grip = grip;
        this.wrist = wrist;
        this.pinger = pinger;
        this.arm_L = arm_L;
        this.arm_R = arm_R;
    }


    void delay(double t) {
        ElapsedTime time = new ElapsedTime();
        time.reset();
        while (time.seconds() <= t) {

        }
        return;
    }

    public void grip(){
        RotateToPosition(MIN_ROTATE_SLIDER,true);

        delay(0.1);
        wrist.setPosition(WRIST.PICK.value);
        pinger.setPosition(PINGER.PICK.value);
        grip.setPosition(GRIP.RELEASE.value);

        delay(0.5);
        arm_R.setPosition(ARM.PICK.value);
        arm_L.setPosition(ARM.PICK.value);
        delay(0.5);
        grip.setPosition(GRIP.CATCH.value);
        delay(0.3);

        wrist.setPosition(WRIST.HANG.value);

        ResetSliderRotation();
    }

    public void collect(){
        RotateToPosition(MIN_ROTATE_SLIDER,true);


        wrist.setPosition(WRIST.COLLECT.value);
        pinger.setPosition(PINGER.COLLECT.value);
        grip.setPosition(GRIP.RELEASE.value);
        delay(0.5);
        arm_R.setPosition(ARM.COLLECT.value);
        arm_L.setPosition(ARM.COLLECT.value);
        delay(0.5);
        grip.setPosition(GRIP.CATCH.value);
        delay(0.3);
        wrist.setPosition(WRIST.HANG.value);


        ResetSliderRotation();
    }
    
    public void hang(){
        arm_L.setPosition(ARM.HANG.value);
        arm_R.setPosition(ARM.HANG.value);
        grip.setPosition(GRIP.CATCH.value);
        wrist.setPosition(WRIST.HANG.value);
        pinger.setPosition(PINGER.HANG.value);
        delay(0.1);
        SlideToPosition(-MAX_HIGH_SLIDER,true);
        delay(0.2);
        arm_L.setPosition(ARM.HANG2.value);
        arm_R.setPosition(ARM.HANG2.value);
        delay(0.05);
        grip.setPosition(GRIP.RELEASE.value);
        SlideToPosition(-MIN_HIGH_SLIDER,true);

    }

    public void RotateSlider(double i,boolean isOpMode){
        if(Math.abs(i) <= 0.1){
            return;
        }

        if(i < 0){
            RotateToPosition(MAX_ROTATE_SLIDER,isOpMode);
        }
        if(i > 0){
            ResetSliderRotation();
        }
    }

    public void manualHighSliderControl(double i) {
        double power = i * 0.7;

        // deadzone
        if (Math.abs(power) > 0.1) {
            hsl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hsr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hsr.setPower(0.7);
            hsl.setPower(0.7);
        }
        hsl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hsr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hsr.setPower(0);
        hsl.setPower(0);
    }

    public void moveSlider(double i, double speed){
        if(Math.abs(i) < 0.1 || (Math.abs(hsl.getCurrentPosition()) >= MAX_HIGH_SLIDER && i < 0)) {
            hsl.setPower(0);
            hsr.setPower(0);
            return;
        }
        hsl.setPower(i * 0.7);
        hsr.setPower(i * 0.7);
    }

    public void manualHIghSliderRotate(double i) {
        double power = i;

        // deadzone
        if (Math.abs(power) > 0.1) {
            rsr.setPower(power);
            rsl.setPower(power);

        }
        rsr.setPower(0);
        rsl.setPower(0);
    }

    public void SlideToPosition(int targetPosition, boolean opModeIsActive) {
        hsl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hsr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double kP = 0.0035; // 슬라이드 세팅에 따라 값 조정 가능
        double power;

        while (opModeIsActive&&
                (Math.abs(hsl.getCurrentPosition() - targetPosition) > 10 ||
                        Math.abs(hsr.getCurrentPosition() - targetPosition) > 10)) {

            int errorL = targetPosition - hsl.getCurrentPosition();
            int errorR = targetPosition - hsr.getCurrentPosition();

            power = errorL * kP;
            power = Math.max(-1, Math.min(1, power));
            hsl.setPower(power);

            power = errorR * kP;
            power = Math.max(-1, Math.min(1, power));
            hsr.setPower(power);
        }

        hsl.setPower(0);
        hsr.setPower(0);
    }




    public void RotateToPosition(int targetPosition, boolean opModeIsActive) {
        rsl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rsr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double kP = 0.0035; // 슬라이드 세팅에 따라 값 조정 가능
        double power;

        while (opModeIsActive&&
                (Math.abs(rsl.getCurrentPosition() - targetPosition) > 10 ||
                        Math.abs(rsr.getCurrentPosition() - targetPosition) > 10)) {

            int errorL = targetPosition - rsl.getCurrentPosition();
            int errorR = targetPosition - rsr.getCurrentPosition();

            power = errorL * kP;
            power = Math.max(-1, Math.min(1, power));
            rsl.setPower(power);

            power = errorR * kP;
            power = Math.max(-1, Math.min(1, power));
            rsr.setPower(power);
        }

        rsl.setPower(0);
        rsr.setPower(0);
    }
    public void ResetSliderRotation(){
        rsl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rsr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rsl.setTargetPosition(INIT_ROTATE_SLIDER);
        rsr.setTargetPosition(INIT_ROTATE_SLIDER);

        rsl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rsr.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        rsl.setPower(1);
        rsr.setPower(1);
    }

    public void RotateToPositionToDown(int targetPosition, boolean opModeIsActive) {
        rsl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rsr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double kP = 0.0035; // 슬라이드 세팅에 따라 값 조정 가능
        double power;

        while (opModeIsActive&&
                (Math.abs(rsl.getCurrentPosition() - targetPosition) > 10 ||
                        Math.abs(rsr.getCurrentPosition() - targetPosition) > 10)) {

            int errorL = targetPosition - rsl.getCurrentPosition();
            int errorR = targetPosition - rsr.getCurrentPosition();

            power = errorL * kP;
            power = Math.max(-1, Math.min(1, power));
            rsl.setPower(power * 0.5);

            power = errorR * kP;
            power = Math.max(-1, Math.min(1, power));
            rsr.setPower(power * 0.5);
        }

        rsl.setPower(0);
        rsr.setPower(0);
    }
}
