package org.firstinspires.ftc.teamcode.mini_project.Action;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mini_project.hardware.pose.*;

public class Chamber {
    public Telemetry telemetry;


    DcMotorEx rsr, rsl, hsl, hsr;
    Servo grip, wrist, pinger, arm_L, arm_R;
    PIDCoefficients pidCoe;

    int sliderTargetPos = 0;

    int maxSliderPos = 3000;

    public Chamber(DcMotorEx rsr, DcMotorEx rsl, DcMotorEx hsl, DcMotorEx hsr, Servo grip, Servo wrist, Servo pinger, Servo arm_L, Servo arm_R,PIDCoefficients pid){
        this.rsr = rsr;
        this.rsl = rsl;
        this.hsl = hsl;
        this.hsr = hsr;
        this.grip = grip;
        this.wrist = wrist;
        this.pinger = pinger;
        this.arm_L = arm_L;
        this.arm_R = arm_R;
        this.pidCoe = pid;
    }


    void delay(double t) {
        ElapsedTime time = new ElapsedTime();
        time.reset();
        while (time.seconds() <= t) {

        }
        return;
    }

    public void grip(){
        wrist.setPosition(WRIST.PICK.value);
        pinger.setPosition(PINGER.PICK.value);
        grip.setPosition(GRIP.RELEASE.value);

        delay(0.5);
        arm_R.setPosition(ARM.PICK.value);
        arm_L.setPosition(ARM.PICK.value);
        delay(0.5);
        grip.setPosition(GRIP.CATCH.value);
        delay(0.3);

        arm_R.setPosition(ARM.RESET.value);
        arm_L.setPosition(ARM.RESET.value);
    }

    public void collect(){
        wrist.setPosition(WRIST.COLLECT.value);
        pinger.setPosition(PINGER.COLLECT.value);
        grip.setPosition(GRIP.RELEASE.value);

        delay(0.5);
        arm_R.setPosition(ARM.COLLECT.value);
        arm_L.setPosition(ARM.COLLECT.value);
        delay(0.5);
        grip.setPosition(GRIP.CATCH.value);
        delay(0.3);

        arm_R.setPosition(ARM.RESET.value);
        arm_L.setPosition(ARM.RESET.value);
    }
    
    public void hang(){
        arm_L.setPosition(ARM.HANG.value);
        arm_R.setPosition(ARM.HANG.value);
        grip.setPosition(GRIP.CATCH.value);
        wrist.setPosition(WRIST.HANG.value);
        pinger.setPosition(PINGER.HANG.value);
        delay(0.3);
        arm_L.setPosition(ARM.HANG2.value);
        arm_R.setPosition(ARM.HANG2.value);
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

        double output = (error * pidCoe.p) + (derivative * pidCoe.d) + (integralSum * pidCoe.i) + (ref * 5 /*K_f*/);
        return  output;
    }

//    public void moveSliderTo(int targetPos){
//
//        for(DcMotorEx slider: new DcMotorEx[]{hsl,hsr}){
//            slider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            slider.setPower(PIDControl(targetPos,slider.getCurrentPosition()));
//        }
//    }
//    public void moveSlider(double i,double speed){
//        for(DcMotorEx slider: new DcMotorEx[]{hsl,hsr}){
//            int targetPos = slider.getTargetPosition() + (int)Math.floor(i*speed * 10);
//            slider.setPower(PIDControl(targetPos,slider.getCurrentPosition()));
//        }
//    }

    public void moveSliderTo(int targetPos,double speed){

        for(DcMotorEx slider: new DcMotorEx[]{hsl,hsr}){
            slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slider.setTargetPosition(targetPos);
            slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            int dir = Integer.signum(targetPos - slider.getCurrentPosition());
            slider.setPower(speed * dir);
        }
    }
    public void moveSlider(double i, double speed){
        if(Math.abs(hsl.getCurrentPosition()) < maxSliderPos && i < 0) hsl.setPower(i * speed);
        if(Math.abs(hsr.getCurrentPosition()) < maxSliderPos && i < 0) hsr.setPower(i * speed);

        if(Math.abs(hsl.getCurrentPosition()) > 0 && i > 0) hsl.setPower(i * speed);
        if(Math.abs(hsr.getCurrentPosition()) > 0 && i > 0) hsr.setPower(i * speed);
    }

    public void rotateSlider(double i){
        rsr.setPower(i);
        rsl.setPower(i);
    }
}
