package org.firstinspires.ftc.teamcode.TEST;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Test_0609")public class Test_0609 extends LinearOpMode {

    DcMotor lf,rf,lr,rr;
    DcMotor[] motors;

    void forward(int dist, double speed){
        for(int i =0; i<4; i++){
            motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motors[i].setTargetPosition(dist);
            motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motors[i].setPower(speed);
        }
        while (lf.isBusy()){

        }
    }
    void left(int dist, double speed){
        int[] directions = {-1,1,1,-1};
        for(int i =0; i<4; i++){
            motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motors[i].setTargetPosition(dist * directions[i]);
            motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motors[i].setPower(speed * directions[i]);
        }
        while (lf.isBusy()){

        }
    }


    @Override public void runOpMode() {



        waitForStart();
        if (opModeIsActive()) {
            lf = hardwareMap.get(DcMotor.class,"FL");
            rf = hardwareMap.get(DcMotor.class,"FR");
            lr = hardwareMap.get(DcMotor.class,"RL");
            rr = hardwareMap.get(DcMotor.class,"RR");

            lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            lf.setDirection(DcMotorSimple.Direction.REVERSE);
            lr.setDirection(DcMotorSimple.Direction.REVERSE);

            motors = new DcMotor[]{lf,rf,lr,rr};

            forward(1000,0.4);
            left(1000, 0.4);
            while(opModeIsActive()){

            }
        }
    }
}
