package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Test_25_07_08")
public class Test_25_07_08 extends LinearOpMode {

    DcMotor lf, rf, lr, rr;
    Servo servo;


    void initialize(){
        lf = hardwareMap.dcMotor.get("FL");
        rf = hardwareMap.dcMotor.get("FR");
        lr = hardwareMap.dcMotor.get("RL");
        rr = hardwareMap.dcMotor.get("RR");

        rf.setDirection(DcMotorSimple.Direction.FORWARD);
        rr.setDirection(DcMotorSimple.Direction.FORWARD);
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lr.setDirection(DcMotorSimple.Direction.REVERSE);

        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        servo = hardwareMap.servo.get("S");

        servo.setPosition(0);
        lf.setPower(0);
        rf.setPower(0);
        lr.setPower(0);
        rr.setPower(0);
    }

    void move(double x, double y, double r, double spd){
        lf.setPower(spd*(x-y+r)/3);
        rf.setPower(spd*(-x-y-r)/3);
        lr.setPower(spd*(-x-y+r)/3);
        rr.setPower(spd*(x-y-r)/3);
    }

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        if (opModeIsActive()) {
            // Pre-run
            while (opModeIsActive()) {
                // OpMode loop
                move(
                        gamepad1.left_stick_x,
                        gamepad1.left_stick_y,
                        gamepad1.right_stick_x,
                        0.6);

                if(gamepad1.a){
                    servo.setPosition(1);
                } else {
                    servo.setPosition(0);
                }
            }
        }
    }
}
