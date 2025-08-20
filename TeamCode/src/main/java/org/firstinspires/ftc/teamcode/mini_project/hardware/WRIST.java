package org.firstinspires.ftc.teamcode.mini_project.hardware;

public enum WRIST{
    HANG(0),
    PICK(0.5),
    COLLECT(0.4);
    public double value;
    WRIST(double val){
        this.value = val;
    }
}
