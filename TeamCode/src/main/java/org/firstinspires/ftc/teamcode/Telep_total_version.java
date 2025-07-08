package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class Telep_total_version extends LinearOpMode {


    private ColorSensor colorSensor;

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private IMU imu_IMU;
    private DcMotor HRS;
    private DcMotor HLS;
    private Servo HL_arm;
    private Servo HR_arm;
    private Servo H_wrist;
    private Servo H_claw;

    private Servo LRS;
    private Servo LL_arm;
    private Servo LR_arm;
    private Servo L_wrist;
    private Servo L_pinger;
    private Servo L_claw;
    private ElapsedTime ChamberreadyTimer = new ElapsedTime();
    private ElapsedTime ChamberscoreTimer = new ElapsedTime();

    private ElapsedTime GiverTimer = new ElapsedTime();
    private ElapsedTime BascketTimer = new ElapsedTime();
    private ElapsedTime ChamberTimer = new ElapsedTime();
    private boolean Chamber_ready  = false;
    private boolean Chamber_score = false;
    private boolean Chamber_score2 = false;

    private boolean Giver = false;
    private boolean Bascket_score = false;
    private boolean[] Chamber_stepDone = new boolean[4];
    private boolean[] Chamberscore_stepDone = new boolean[4];
    private boolean[] Giver_stepDone = new boolean[4];
    private boolean[] Bascket_stepDone = new boolean[4];
    private boolean[] Chamber_ready_stepDone = new boolean[4];

    private final double r = (1.0 / 1080.0) * 300;

    private final double LRS_MIN = 0.5, LRS_MAX = 1.0;
    private double LRSPOS = 0.5;
    private final double LRSINCR = 0.01; // 한 번에 바뀌는 양
    double speed = 1.0;
    double yaw;

    private double pingerPos = 0.05;
    private final double INCREMENT = 0.01; // 한 번에 바뀌는 양
    private final double PINGER_MIN = 0.0;
    private final double PINGER_MAX = 1.0;

    private void IMU_INIT() {
        // Initializes the IMU with non-default settings. To use this block,
        // plug one of the "new IMU.Parameters" blocks into the parameters socket.
        // Creates a Parameters object for use with an IMU in a REV Robotics Control Hub or
        // Expansion Hub, specifying the hub's orientation on the robot via the direction that
        // the REV Robotics logo is facing and the direction that the USB ports are facing.
        imu_IMU.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        // Prompt user to press start button.
        telemetry.addData("IMU Init!", "Press start to continue...");
        telemetry.update();
    }

    /**
     * Describe this function...
     */
    private void getYaw() {
        yaw = imu_IMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        telemetry.addData("yaw", yaw);
    }


    private void mecanumDriveStickView() {
        double headingDir;
        double y;
        double x;
        double stopDir;
        double movingDir;
        double diffAngle = 0;
        double rx;
        double denominator;

        if (0.01 < Math.abs(gamepad1.left_stick_x) || 0.01 < Math.abs(gamepad1.left_stick_y)) {//감도: 0.01
            headingDir = Math.toDegrees(Math.atan2(-gamepad1.left_stick_x, -gamepad1.left_stick_y));
            telemetry.addData("HeadingDir", headingDir);
            movingDir = headingDir - yaw;
            y = Math.cos(Math.toRadians(movingDir)) * 1;
            x = -Math.sin(Math.toRadians(movingDir )) * 1;
        } else {
            y = 0;
            x = 0;
        }
        if (0.01 < Math.abs(gamepad1.right_stick_x) || 0.01 < Math.abs(gamepad1.right_stick_y)) {
            stopDir = Math.toDegrees(Math.atan2(-gamepad1.right_stick_x, -gamepad1.right_stick_y));
            diffAngle = yaw - stopDir;
            if (diffAngle > 180) {
                diffAngle = -(360 - diffAngle);
            }
            if (diffAngle < -180) {
                diffAngle = 360 - Math.abs(diffAngle);
            }
            if (Math.abs(diffAngle) > 50) {
                rx = Math.signum(diffAngle) * 0.7;
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
        leftFront.setPower((y + x + rx) / denominator * speed);
        leftBack.setPower(((y - x) + rx) / denominator * speed);
        rightFront.setPower(((y - x) - rx) / denominator * speed);
        rightBack.setPower(((y + x) - rx) / denominator * speed);
    }

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        imu_IMU = hardwareMap.get(IMU.class, "imu");
        IMU_INIT();

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        HRS = hardwareMap.get(DcMotor.class, "HRS");
        HLS = hardwareMap.get(DcMotor.class, "HLS");
        HL_arm = hardwareMap.get(Servo.class, "HL_arm");
        HR_arm = hardwareMap.get(Servo.class, "HR_arm");
        H_wrist = hardwareMap.get(Servo.class, "H_wrist");
        H_claw = hardwareMap.get(Servo.class, "H_claw");

        LRS = hardwareMap.get(Servo.class, "LRS");
        LL_arm = hardwareMap.get(Servo.class, "LL_arm");
        LR_arm = hardwareMap.get(Servo.class, "LR_arm");
        L_wrist = hardwareMap.get(Servo.class, "L_wrist");
        L_pinger = hardwareMap.get(Servo.class, "L_pinger");
        L_claw = hardwareMap.get(Servo.class, "L_claw");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        HRS.setDirection(DcMotorSimple.Direction.FORWARD);
        HLS.setDirection(DcMotorSimple.Direction.REVERSE);
        HRS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        HLS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        HRS.setTargetPosition(0);
        HLS.setTargetPosition(0);
        HLS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        HRS.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LL_arm.setPosition(0.7* r);
        LR_arm.setPosition(r - (0.7* r));
        L_wrist.setPosition(0.5);
        L_pinger.setPosition(0.05);
        L_claw.setPosition(1);

        HL_arm.setPosition(0.02);
        HR_arm.setPosition(0.98);
        H_wrist.setPosition(0.77);
        H_claw.setPosition(0.6);



        waitForStart();


        while (opModeIsActive()) {
            // Put loop blocks here.
            getYaw();
            mecanumDriveStickView();
            manualHighSliderControl();
            if (gamepad2.dpad_right) {
                pingerPos += INCREMENT;
            }

            // 왼쪽 DPAD 누르면 감소
            if (gamepad2.dpad_left) {
                pingerPos -= INCREMENT;
            }

            // 위치를 0.0 ~ 1.0 사이로 제한
            pingerPos = Range.clip(pingerPos, PINGER_MIN, PINGER_MAX);

            // 서보에 적용
            L_pinger.setPosition(pingerPos);
            if(gamepad1.back){
                speed = 0.5;
            }

            if(gamepad1.start){
                speed = 1;
            }
            if(gamepad1.right_stick_button){
                IMU_INIT();
            }

            int red = colorSensor.red();
            int green = colorSensor.green();
            int blue = colorSensor.blue();

            String detectedColor;

            // 노랑 먼저 체크 (빨강 + 녹색 강하고 파랑 약할 때)
            if (red > 100 && green > 100 && blue < 80) {
                detectedColor = "Yellow";
            }
            // 빨강이 가장 강할 때
            else if (red > blue && red > green) {
                detectedColor = "Red";
            }
            // 파랑이 가장 강할 때
            else if (blue > red && blue > green) {
                detectedColor = "Blue";
            }
            // 그 외에는 판별 불가
            else {
                detectedColor = "Unknown";
            }

            double stickY = -gamepad2.left_stick_y; // 위로 밀면 +1, 아래로 내리면 -1

            // deadzone 설정 (작게 움직이면 무시)
            if (stickY > 0.2) {
                LRSPOS += LRSINCR;
            } else if (stickY < -0.2) {
                LRSPOS -= LRSINCR;
            }

            // 위치를 0.0 ~ 1.0 사이로 제한
            LRSPOS = Range.clip(LRSPOS, LRS_MIN, LRS_MAX);

            // 서보에 적용
            LRS.setPosition(LRSPOS);

            if (gamepad2.x) {
                //collect

                Collect();

            }

            if (gamepad2.y) {
                //standby

                Collect_StandBy();

            }






            if (gamepad2.left_bumper) {

                L_claw.setPosition(1);

            } else if (gamepad2.right_bumper) {

                L_claw.setPosition(0.79);

            }

            if(gamepad2.dpad_up){
                LRS.setPosition(1.0);
            }

            if(gamepad2.dpad_down){
                LRS.setPosition(0.5);
            }

            if (gamepad2.a && !Chamber_ready) {
                Chamber_ready = true;
                ChamberreadyTimer.reset();
                Chamber_ready_stepDone = new boolean[4]; // 단계 초기화
            }
            if(Chamber_ready) Chamber_ready();

            if (gamepad2.b && !Chamber_score) {
                Chamber_score = true;
                ChamberTimer.reset();
                Chamber_stepDone = new boolean[4]; // 단계 초기화

            }
            if (Chamber_score) Chamber_score();

            if (gamepad1.x && !Chamber_score2) {
                Chamber_score2 = true;
                ChamberscoreTimer.reset();
                Chamberscore_stepDone = new boolean[4]; // 단계 초기화

            }

            if (Chamber_score2) Chamber_score2();

            // A 버튼을 눌러 Giver 동작 시작
            if (gamepad1.a && !Giver) {
                Giver = true;
                GiverTimer.reset();
                Giver_stepDone = new boolean[4];
            }

            if (Giver) Giver();

            // B 버튼을 눌러 Bascket 동작 시작
            if (gamepad1.b && !Bascket_score) {
                Bascket_score = true;
                BascketTimer.reset();
                Bascket_stepDone = new boolean[4];
            }

            if (Bascket_score) Bascket_score();

            // 수동 제어
            if (gamepad1.left_bumper) {
                H_claw.setPosition(0.6);
            } else if (gamepad1.right_bumper) {
                H_claw.setPosition(1);
            }

        }

        telemetry.update();


    }

    public void Giver() {
        double Giver_elapsed = GiverTimer.seconds();

        if (Giver_elapsed > 0.2 && !Giver_stepDone[0]) {
            LRS.setPosition(0.5);
            LL_arm.setPosition(0.865 * r);
            LR_arm.setPosition(r - (0.865 * r));
            L_wrist.setPosition(0.34);
            L_pinger.setPosition(0.05);
            L_claw.setPosition(0.79);

            H_claw.setPosition(1);
            HL_arm.setPosition(0.84);
            HR_arm.setPosition(0.16);
            HLS.setPower(1);
            HRS.setPower(1);
            HLS.setTargetPosition(0);
            HRS.setTargetPosition(0);
            HRS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            HLS.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            Giver_stepDone[0] = true;
        }

        if (Giver_elapsed > 0.6 && !Giver_stepDone[1]) {

            H_wrist.setPosition(0.85);
            H_claw.setPosition(0.6);


            Giver_stepDone[1] = true;
        }

        if (Giver_elapsed > 1.0 && !Giver_stepDone[2]) {
            H_claw.setPosition(1);
            Giver_stepDone[2] = true;
        }

        if (Giver_elapsed > 1.3 && !Giver_stepDone[3]) {
            L_claw.setPosition(1);
            Giver_stepDone[3] = true;
            Giver = false;
        }
    }

    public void Collect() {
        LL_arm.setPosition(0.98 * r);
        LR_arm.setPosition(r - (0.98 * r));
        L_wrist.setPosition(0.8);
        L_pinger.setPosition(0.05);
    }

    public void Collect_StandBy() {
        LL_arm.setPosition(0.8 * r);
        LR_arm.setPosition(r - (0.8 * r));
        L_wrist.setPosition(0.8);
        L_pinger.setPosition(0.05);
    }

    public void Bascket_score() {
        double Bascket_elapsed = BascketTimer.seconds();

        if (Bascket_elapsed > 0.2 && !Bascket_stepDone[0]) {


            Bascket_stepDone[0] = true;
        }

        if (Bascket_elapsed > 1 && !Bascket_stepDone[1]) {
            H_wrist.setPosition(0.4);
            HL_arm.setPosition(0.5);
            HR_arm.setPosition(0.5);
            H_claw.setPosition(1);

            LL_arm.setPosition(0.8 * r);
            LR_arm.setPosition(r - (0.8 * r));
            L_wrist.setPosition(0.8);
            Bascket_stepDone[1] = true;
        }

        if (Bascket_elapsed > 1.3 && !Bascket_stepDone[2]) {

            Bascket_stepDone[2] = true;
        }

        if (Bascket_elapsed > 1.5 && !Bascket_stepDone[3]) {

            Bascket_stepDone[3] = true;
            Bascket_score = false;
        }
    }

    private void Chamber_ready() {

        double Chamber_ready_elapsed = ChamberreadyTimer.seconds();

        if (Chamber_ready_elapsed > 0.1 && !Chamber_ready_stepDone[0]) {





            Chamber_ready_stepDone[0] = true;
        }

        if (Chamber_ready_elapsed > 0.5 && !Chamber_ready_stepDone[1]) {


            HL_arm.setPosition(0.02);
            HR_arm.setPosition(0.98);
            H_wrist.setPosition(0.77);
            H_claw.setPosition(0.6);


            Chamber_ready_stepDone[1] = true;
        }


        if (Chamber_ready_elapsed > 1 && !Chamber_ready_stepDone[2]) {
            //score



            Chamber_ready_stepDone[2] = true;

        }
        if (Chamber_ready_elapsed > 1.2 && !Chamber_ready_stepDone[3]) {
            //score


            Chamber_ready_stepDone[3] = true;
            Chamber_ready = false; // 완료 후 자동 종료
        }

    }

    private void Chamber_score() {
        double Chamber_elapsed = ChamberTimer.seconds();

        if (Chamber_elapsed > 0.1 && !Chamber_stepDone[0]) {
            //highslider 수직 이동 및 low 이동

            H_claw.setPosition(1);
            H_wrist.setPosition(0.87);


            Chamber_stepDone[0] = true;
        }

        if (Chamber_elapsed > 0.5 && !Chamber_stepDone[1]) {

            HL_arm.setPosition(0.77);
            HR_arm.setPosition(0.23);
            H_wrist.setPosition(0.535);
            H_claw.setPosition(1);


            Chamber_stepDone[1] = true;
        }

        if (Chamber_elapsed > 1 && !Chamber_stepDone[2]) {
            //score



            Chamber_stepDone[2] = true;

        }
        if (Chamber_elapsed > 1.5 && !Chamber_stepDone[3]) {
            //score

            Chamber_stepDone[3] = true;
            Chamber_score = false; // 완료 후 자동 종료
        }

    }
    public void Chamber_score2(){
        double Chamberscore_elapsed = ChamberscoreTimer.seconds();

        if (Chamberscore_elapsed > 0.1 && !Chamberscore_stepDone[0]) {
            //highslider 수직 이동 및 low 이동

            HLS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            HRS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            HLS.setPower(-0.85);
            HRS.setPower(-0.85);

            while (opModeIsActive() && (HLS.getCurrentPosition() > -650 || HRS.getCurrentPosition() > -650)) {
                telemetry.addData("HLS pos", HLS.getCurrentPosition());
                telemetry.addData("HRS pos", HRS.getCurrentPosition());
                telemetry.update();
            }

            HLS.setPower(0);
            HRS.setPower(0);


            Chamberscore_stepDone[0] = true;
        }

        if (Chamberscore_elapsed > 0.5 && !Chamberscore_stepDone[1]) {


            H_claw.setPosition(0.6);


            Chamberscore_stepDone[1] = true;
        }

        if (Chamberscore_elapsed > 1 && !Chamberscore_stepDone[2]) {
            //score

            HLS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            HRS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            HLS.setPower(0.85);
            HRS.setPower(0.85);

            while (opModeIsActive() && (HLS.getCurrentPosition() < 0 || HRS.getCurrentPosition() < 0)) {
                telemetry.addData("HLS pos", HLS.getCurrentPosition());
                telemetry.addData("HRS pos", HRS.getCurrentPosition());
                telemetry.update();
            }

            HLS.setPower(0);
            HRS.setPower(0);

            Chamberscore_stepDone[2] = true;

        }
        if (Chamberscore_elapsed > 1.5 && !Chamberscore_stepDone[3]) {
            //score

            Chamberscore_stepDone[3] = true;
            Chamber_score2 = false; // 완료 후 자동 종료
        }



    }
    private void manualHighSliderControl() {
        double power = gamepad2.right_stick_y;

        // deadzone
        if (Math.abs(power) > 0.05) {
            HRS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            HLS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            HRS.setPower(power);
            HLS.setPower(power);

        }
    }

    public void SlideToPosition(int targetPosition) {
        HLS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        HRS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double kP = 0.0035; // 슬라이드 세팅에 따라 값 조정 가능
        double power;

        while (opModeIsActive() &&
                (Math.abs(HLS.getCurrentPosition() - targetPosition) > 10 ||
                        Math.abs(HRS.getCurrentPosition() - targetPosition) > 10)) {

            int errorL = targetPosition - HLS.getCurrentPosition();
            int errorR = targetPosition - HRS.getCurrentPosition();

            power = errorL * kP;
            power = Math.max(-1, Math.min(1, power));
            HLS.setPower(power);

            power = errorR * kP;
            power = Math.max(-1, Math.min(1, power));
            HRS.setPower(power);
        }

        HLS.setPower(0);
        HRS.setPower(0);
    }

}




