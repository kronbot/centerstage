package org.firstinspires.ftc.teamcode.kronbot.detection;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class GameElementDetection {
    OpenCvWebcam webcam;
    DetectionPipeline pipeline;

    public enum Color {
        RED(2), GREEN(1), BLUE(0);

        private int color;

        Color (int color) {
            this.color = color;
        }
    }

    public void init(HardwareMap hardwareMap, Telemetry telemetry, Color color) {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        pipeline = new DetectionPipeline();
        pipeline.init(telemetry, color.color);
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }

    public boolean detect() {
        return pipeline.isLeft();
    }

    public void close() {
        webcam.stopStreaming();
        webcam.closeCameraDevice();
    }

    class DetectionPipeline extends OpenCvPipeline {
        Mat YCrCb = new Mat();
        Mat leftElement;
        Mat rightElement;

        double leftAvg, rightAvg;
        Mat output = new Mat();
        Scalar rectColor = new Scalar(0, 255, 0);

        boolean isLeft = false;

        Telemetry telemetry;

        int color = 0;

        public void init(Telemetry telemetry, int color) {
            this.telemetry = telemetry;
            this.color = color;
        }

        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            telemetry.addLine("Pipeline running");

            Rect leftRect = new Rect(1, 1, 639, 719);
            Rect rightRect = new Rect(640, 1, 1279, 719);

            input.copyTo(output);
            Imgproc.rectangle(output, leftRect, rectColor, 2);
            Imgproc.rectangle(output, rightRect, rectColor, 2);

            leftElement = YCrCb.submat(leftRect);
            rightElement = YCrCb.submat(rightRect);

            Core.extractChannel(leftElement, leftElement, color);
            Core.extractChannel(rightElement, rightElement, color);

            Scalar leftMean = Core.mean(leftElement);
            Scalar rightMean = Core.mean(rightElement);

            leftAvg = leftMean.val[0];
            rightAvg = rightMean.val[0];

            if (leftAvg > rightAvg) {
                Imgproc.rectangle(output, leftRect, rectColor, 5);
                telemetry.addData("Element", "Left");
                isLeft = true;
            } else {
                Imgproc.rectangle(output, rightRect, rectColor, 5);
                telemetry.addData("Element", "Right");
                isLeft = false;
            }

            return (output);
        }

        public boolean isLeft() {
            return isLeft;
        }
    }


}
