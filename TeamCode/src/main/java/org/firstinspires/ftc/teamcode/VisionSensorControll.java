package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;



@TeleOp(name = "VisionSensorControll")
public class VisionSensorControll extends LinearOpMode {
    @Override
    public void runOpMode() {
        PredominantColorProcessor colorSensor = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.1,0.1,-0.1,0.1))
                .setSwatches(
                        PredominantColorProcessor.Swatch.RED,
                        PredominantColorProcessor.Swatch.GREEN,
                        PredominantColorProcessor.Swatch.BLUE,
                        PredominantColorProcessor.Swatch.YELLOW,
                        PredominantColorProcessor.Swatch.WHITE,
                        PredominantColorProcessor.Swatch.BLACK
                        )
                .build();

        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorSensor)
                .setCamera(hardwareMap.get(WebcamName.class,"cam"))
                .build();

        waitForStart();

        if (opModeIsActive()) {
            // Pre-run
            while (opModeIsActive()) {
                // OpMode loop
                PredominantColorProcessor.Result result = colorSensor.getAnalysis();

                telemetry.addData("Best Match", result.closestSwatch);
                telemetry.addLine(String.format("RGB   (%3d, %3d, %3d)", Color.red(result.rgb),Color.green(result.rgb),Color.blue(result.rgb)));
                telemetry.update();

                sleep(20);


            }
        }
    }
}
