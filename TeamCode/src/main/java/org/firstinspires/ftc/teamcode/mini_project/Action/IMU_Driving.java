package org.firstinspires.ftc.teamcode.mini_project.Action;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mini_project.hardware.Robot;

public class IMU_Driving {

    Robot robot = new Robot();

    public IMU imu_IMU;


     public double speed = 1.0;  /** 메카넘 주행 속도, 속도 조절 변수로도 사용 */

    double yaw;
    public void IMU_INIT() {
        // Initializes the IMU with non-default settings. To use this block,
        // plug one of the "new IMU.Parameters" blocks into the parameters socket.
        // Creates a Parameters object for use with an IMU in a REV Robotics Control Hub or
        // Expansion Hub, specifying the hub's orientation on the robot via the direction that
        // the REV Robotics logo is facing and the direction that the USB ports are facing.
        imu_IMU.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        // Prompt user to press start button.
        telemetry.addData("IMU Init!", "Press start to continue...");
        telemetry.update();
    }

    /**
     * 로봇 주행 시작 시 로봇이 앞을 보고 있는 기준으로 yaw 값 재조정, 오류 생길 가능성 0 사용도에 따라서 따로 init 버튼 추가...
     */
    public void getYaw() {
        yaw = imu_IMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        telemetry.addData("yaw", yaw);
    }


    public void mecanumDriveStickView() {
        double headingDir;
        double y;
        double x;
        double stopDir;
        double movingDir;
        double diffAngle = 0;
        double rx;
        double denominator;


        if (0.01 < Math.abs(gamepad1.left_stick_x) || 0.01 < Math.abs(gamepad1.left_stick_y)) {
            headingDir = Math.atan2(-gamepad1.left_stick_x, -gamepad1.left_stick_y) / Math.PI * 180;
            telemetry.addData("HeadingDir", headingDir);
            movingDir = headingDir - yaw;
            y = Math.cos(movingDir / 180 * Math.PI) * 1;
            x = -Math.sin(movingDir / 180 * Math.PI) * 1;
        } else {
            y = 0;
            x = 0;
        }
        if (0.01 < Math.abs(gamepad1.right_stick_x) || 0.01 < Math.abs(gamepad1.right_stick_y)) {
            stopDir = Math.atan2(-gamepad1.right_stick_x, -gamepad1.right_stick_y) / Math.PI * 180;
            diffAngle = yaw - stopDir;
            if (diffAngle > 180) {
                diffAngle = -(360 - diffAngle);
            }
            if (diffAngle < -180) {
                diffAngle = 360 - Math.abs(diffAngle);
            }
            if (Math.abs(diffAngle) > 50) {
                rx = (diffAngle / Math.abs(diffAngle)) * 0.7;
            } else {
                rx = diffAngle / 50;
            }
            telemetry.addData("stopDir", stopDir);
        } else {
            if (gamepad1.dpad_left) {
                rx = -0.5;
            } else if (gamepad1.dpad_right) {
                rx = 0.5;
            } else {
                rx = 0;
            }
        }
        telemetry.addData("diffAngle", diffAngle);
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("rx", rx);
        denominator = JavaUtil.maxOfList(JavaUtil.createListWith(JavaUtil.sumOfList(JavaUtil.createListWith(Math.abs(y), Math.abs(x), Math.abs(rx))), 1));
        robot.FR.setPower((y + x + rx) / denominator * speed);
        robot.FL.setPower(((y - x) + rx) / denominator * speed);
        robot.RR.setPower(((y - x) - rx) / denominator * speed);
        robot.RL.setPower(((y + x) - rx) / denominator * speed);
    }

}
