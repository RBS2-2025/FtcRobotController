package org.firstinspires.ftc.teamcode.mini_project.Action;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

public class BasciMovement {
    DcMotor FL,FR,RL,RR;
    public double speed = 1;
    public BasciMovement(DcMotor FL, DcMotor FR, DcMotor RL, DcMotor RR){
        this.FL = FL;
        this.FR = FR;
        this.RL = RL;
        this.RR = RR;
    }

    public void move(double x, double y, double rx){

        double denominator = JavaUtil.maxOfList(JavaUtil.createListWith(JavaUtil.sumOfList(JavaUtil.createListWith(Math.abs(y), Math.abs(x), Math.abs(rx))), 1));


        FL.setPower((y+x+rx)/denominator*speed);
        FR.setPower((y-x-rx)/denominator*speed);
        RL.setPower((y-x+rx)/denominator*speed);
        RR.setPower((y+x-rx)/denominator*speed);
    }
}
