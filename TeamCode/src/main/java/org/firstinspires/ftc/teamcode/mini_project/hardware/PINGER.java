package org.firstinspires.ftc.teamcode.mini_project.hardware;

public enum PINGER{
    HANG(0),
    PICK(0.65),
    COLLECT(0.65);
    public double value;
    PINGER(double val){
        this.value = val;
    }
}
