package org.firstinspires.ftc.teamcode.kronbot.detection;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

public class TagDetection {
    TfodProcessor tfod;
    AprilTagProcessor aprilTagProcessor;
    VisionPortal visionPortal;

    List<AprilTagDetection> tags;

    public void init(HardwareMap hardwareMap) {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawCubeProjection(true)
                .build();

        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTagProcessor);
        visionPortal.setProcessorEnabled(aprilTagProcessor, true);
    }

    public void close() {
        visionPortal.close();
    }

    public List<AprilTagDetection> getTags() {
        return tags;
    }

    public void detect() {
        tags = aprilTagProcessor.getDetections();
    }

    public void telemetry(Telemetry telemetry) {
        if (tags.size() == 0) {
            telemetry.addLine("No tags detected");
            return;
        }

        for (AprilTagDetection tag : tags) {
            telemetry.addData("Tag ID", tag.metadata.id);
            telemetry.addData("Tag X", tag.ftcPose.x);
            telemetry.addData("Tag Y", tag.ftcPose.y);
            telemetry.addData("Tag Z", tag.ftcPose.z);
            telemetry.addData("Tag Roll", tag.ftcPose.roll);
            telemetry.addData("Tag Pitch", tag.ftcPose.pitch);
            telemetry.addLine(" ");
        }
    }
}
