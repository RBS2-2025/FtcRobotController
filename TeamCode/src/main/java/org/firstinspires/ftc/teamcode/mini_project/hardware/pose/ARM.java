package org.firstinspires.ftc.teamcode.mini_project.hardware.pose;

public enum ARM{
    HANG(0.25),
    HANG2(0.35),
    PICK(0.41),
    COLLECT(0.42),
    RESET(0.7);
    public double value;
    ARM(double val){
        this.value = val;
    }

}
