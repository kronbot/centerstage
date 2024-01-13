package org.firstinspires.ftc.teamcode.kronbot.detection.legacy;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class GameElementDetection {
    OpenCvWebcam webcam;
    DetectionPipeline pipeline;

    public enum Color {
        GREEN(0), RED(1), BLUE(2);

        private int color;

        Color (int color) {
            this.color = color;
        }
    }

    public enum Position {
        LEFT(0), MIDDLE(1), RIGHT(2);

        private int position;

        Position (int position) {
            this.position = position;
        }
    }

    public void init(HardwareMap hardwareMap, Telemetry telemetry, Color color) {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        pipeline = new DetectionPipeline();
        pipeline.init(telemetry, color.color);
        webcam.setPipeline(pipeline);
        telemetry.addData("leftAvg", pipeline.getLeftAvg());
        telemetry.addData("rightAvg", pipeline.getRightAvg());

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {
            }
        });
    }

    public Position detect() {
        return pipeline.position();
    }

    public double getRightAvg() {
        return pipeline.getRightAvg();
    }

    public void close() {
        webcam.stopStreaming();
        webcam.closeCameraDevice();
    }

}
