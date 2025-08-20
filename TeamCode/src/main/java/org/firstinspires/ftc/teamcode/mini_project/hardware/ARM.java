package org.firstinspires.ftc.teamcode.mini_project.hardware;

public enum ARM{
    HANG(0.065),
    PICK(0.065),
    COLLECT(0.075),
    RESET(0.13);
    public double value;
    ARM(double val){
        this.value = val;
    }

}
