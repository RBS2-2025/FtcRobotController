package org.firstinspires.ftc.teamcode.mini_project.Action;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mini_project.hardware.ARM;
import org.firstinspires.ftc.teamcode.mini_project.hardware.GRIP;
import org.firstinspires.ftc.teamcode.mini_project.hardware.PINGER;
import org.firstinspires.ftc.teamcode.mini_project.hardware.WRIST;

public class Chamber {
    DcMotor rs, hsl, hsr;
    Servo grip, wrist, pinger, arm_L, arm_R;

    public Chamber(DcMotor rs, DcMotor hsl, DcMotor hsr, Servo grip, Servo wrist, Servo pinger, Servo arm_L, Servo arm_R){
        this.rs = rs;
        this.hsl = hsl;
        this.hsr = hsr;
        this.grip = grip;
        this.wrist = wrist;
        this.pinger = pinger;
        this.arm_L = arm_L;
        this.arm_R = arm_R;
    }

    void delay(double t){
        ElapsedTime time = new ElapsedTime();
        time.reset();
        while (time.seconds() <= t){

        }
        return;
    }

    public void grip(){
        wrist.setPosition(WRIST.PICK.value);
        pinger.setPosition(PINGER.PICK.value);
        grip.setPosition(GRIP.RELEASE.value);

        delay(0.05);
        arm_R.setPosition(ARM.PICK.value);
        arm_L.setPosition(ARM.PICK.value);
        delay(0.05);
        grip.setPosition(GRIP.CATCH.value);
        delay(0.02);

        arm_R.setPosition(ARM.RESET.value);
        arm_L.setPosition(ARM.RESET.value);
    }

    public void collect(){
        wrist.setPosition(WRIST.COLLECT.value);
        pinger.setPosition(PINGER.COLLECT.value);
        grip.setPosition(GRIP.RELEASE.value);

        delay(0.05);
        arm_R.setPosition(ARM.COLLECT.value);
        arm_L.setPosition(ARM.COLLECT.value);
        delay(0.05);
        grip.setPosition(GRIP.CATCH.value);
        delay(0.02);

        arm_R.setPosition(ARM.RESET.value);
        arm_L.setPosition(ARM.RESET.value);
    }
    
    public void hang(){
        arm_L.setPosition(ARM.HANG.value);
        arm_R.setPosition(ARM.HANG.value);
        grip.setPosition(GRIP.CATCH.value);
        wrist.setPosition(WRIST.HANG.value);
        pinger.setPosition(PINGER.HANG.value);
        delay(0.1);
        arm_L.setPosition(ARM.HANG2.value);
        arm_R.setPosition(ARM.HANG2.value);
    }
    public void moveSlider(double i, double speed){
        hsl.setPower(i * speed);
        hsr.setPower(i * speed);
    }
    public void moveSlider(double i){
        moveSlider(i,1);
    }
}
