package org.firstinspires.ftc.teamcode.kronbot.detection.legacy;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

class DetectionPipeline extends OpenCvPipeline {
    Mat YCrCb = new Mat();
    Mat leftElement;
    Mat rightElement;

    double leftAvg, rightAvg;
    Mat output = new Mat();
    Scalar rectColor = new Scalar(0, 255, 0);

    GameElementDetection.Position position;

    Telemetry telemetry;

    int color = 0;

    double trashHold = Constants.CAMERA_TRASH_HOLD;

    public void init(Telemetry telemetry, int color) {
        this.telemetry = telemetry;
        this.color = color;
    }

    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        telemetry.addLine("pipeline running");


        Rect leftRect = new Rect(1, 1, 319, 359);
        Rect rightRect = new Rect(320, 1, 319, 359);

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

        if (leftAvg > rightAvg && leftAvg > trashHold) {
            Imgproc.rectangle(output, leftRect, rectColor, 5);
            telemetry.addData("Element", "Left");
            position = GameElementDetection.Position.MIDDLE;
        } else if (rightAvg > leftAvg && rightAvg > trashHold) {
            Imgproc.rectangle(output, rightRect, rectColor, 5);
            telemetry.addData("Element", "Right");
            position = GameElementDetection.Position.RIGHT;
        } else position = GameElementDetection.Position.LEFT;

        return (output);
    }

    public GameElementDetection.Position position() {
        return position;
    }

    public double getLeftAvg() {
        return leftAvg;
    }

    public double getRightAvg() {
        return rightAvg;
    }


}