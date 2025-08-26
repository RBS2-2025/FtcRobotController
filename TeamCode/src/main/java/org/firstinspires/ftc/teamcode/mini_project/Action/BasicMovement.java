package org.firstinspires.ftc.teamcode.mini_project.Action;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

public class BasicMovement {
    DcMotor FL,FR,RL,RR;
    public double speed = 0.7;
    public BasicMovement(DcMotor fl, DcMotor fr, DcMotor rl, DcMotor rr){
        this.FL = fl;
        this.FR = fr;
        this.RL = rl;
        this.RR = rr;
    }

    public void move(double x,double y,double rx){
        if(Math.abs(x) > 0.1 || Math.abs(y) > 0.1 || Math.abs(rx) > 0.1){
            double denominator = JavaUtil.maxOfList(JavaUtil.createListWith(JavaUtil.sumOfList(JavaUtil.createListWith(Math.abs(y), Math.abs(x), Math.abs(rx))), 1));
            FL.setPower((-y+x+rx)/denominator*speed);
            FR.setPower((-y-x-rx)/denominator*speed);
            RL.setPower((-y-x+rx)/denominator*speed);
            RR.setPower((-y+x-rx)/denominator*speed);
        } else{
            FL.setPower(0);
            FR.setPower(0);
            RL.setPower(0);
            RR.setPower(0);
        }
    }
}
