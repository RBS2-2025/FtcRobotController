package org.firstinspires.ftc.teamcode.mini_project.hardware.pose;

public enum ARM{
    HANG(0.35),
    HANG2(0.65),
    PICK(0.41),
    COLLECT(0.42),
    RESET(0.7);
    public double value;
    ARM(double val){
        this.value = val;
    }

}
