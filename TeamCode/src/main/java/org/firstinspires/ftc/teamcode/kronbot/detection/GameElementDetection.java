package org.firstinspires.ftc.teamcode.kronbot.detection;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

public class    GameElementDetection {
    private static final String TFOD_MODEL_FILE = "GameElement.tflite";

    private static final String[] LABELS = {
            "Red",
            "Blue"
    };

    TfodProcessor tfod;
    VisionPortal visionPortal;

    private List<Recognition> gameElementRecognition;

    public void init(HardwareMap hardwareMap) {
        tfod = new TfodProcessor.Builder()
            .setModelFileName(TFOD_MODEL_FILE)
            .setModelLabels(LABELS).build();

        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), tfod);
    }

    public void close() {
        visionPortal.close();
    }

    public List<Recognition> getGameElement() {
        return gameElementRecognition;
    }

    public void detect() {
        gameElementRecognition = tfod.getRecognitions();
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("# Objects Detected", gameElementRecognition.size());

        for (Recognition recognition : gameElementRecognition) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }
    }
}
