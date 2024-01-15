package org.firstinspires.ftc.teamcode.kronbot.autonomy;

import static org.firstinspires.ftc.teamcode.kronbot.utils.AutonomousConstants.Parking;
import static org.firstinspires.ftc.teamcode.kronbot.utils.AutonomousConstants.RedPixelLeft;
import static org.firstinspires.ftc.teamcode.kronbot.utils.AutonomousConstants.RedPixelMiddle;
import static org.firstinspires.ftc.teamcode.kronbot.utils.AutonomousConstants.RedPixelRight;
import static org.firstinspires.ftc.teamcode.kronbot.utils.AutonomousConstants.coordinatesConvert;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.kronbot.KronBot;
import org.firstinspires.ftc.teamcode.kronbot.detection.GameElementDetection;
import org.firstinspires.ftc.teamcode.kronbot.utils.AutonomousConstants;
import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name = "Right Red", group = Constants.MAIN_GROUP)
public class LeftRed extends LinearOpMode {
    GameElementDetection detection;
    GameElementDetection.Position position;
    KronBot robot = new KronBot();

    @Override
    public void runOpMode()  throws InterruptedException {
        robot.init(hardwareMap, true);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        detection = new GameElementDetection();
        detection.init(hardwareMap, true);

        FtcDashboard.getInstance().startCameraStream(detection.getCamera(), 30);

        while (!opModeIsActive() && !isStopRequested()) {
            position = detection.getPosition();
            telemetry.addData("Position", detection.getPosition());
            telemetry.update();
        }

        detection.close();

        Pose2d pixelPose = new Pose2d(0, 0, 0);
        if (position == GameElementDetection.Position.RIGHT)
            pixelPose = coordinatesConvert(RedPixelRight);
        else if (position == GameElementDetection.Position.MIDDLE)
            pixelPose = coordinatesConvert(RedPixelMiddle);
        else
            pixelPose = coordinatesConvert(RedPixelLeft);
        Pose2d parkPose = coordinatesConvert(Parking);

        SequentialAction middlePixel = new SequentialAction(
                drive.actionBuilder(drive.pose)
                        .lineToX(pixelPose.position.x)
                        .strafeTo(new Vector2d(pixelPose.position.x, pixelPose.position.y))
                        .build(),
                (drop) -> {
                    robot.servos.pixel(false);
                    sleep(1000);
                    return false;
                }
        );
        SequentialAction rightPixel = new SequentialAction(
                drive.actionBuilder(drive.pose)
                        .strafeTo(new Vector2d(pixelPose.position.x, pixelPose.position.y)).lineToX(pixelPose.position.x).build(),
                (drop) -> {
                    robot.servos.pixel(false);
                    sleep(1000);
                    return false;
                }
        );
        SequentialAction leftPixel = new SequentialAction(
                drive.actionBuilder(drive.pose).strafeTo(new Vector2d(pixelPose.position.x, pixelPose.position.y)).lineToX(pixelPose.position.x).build(),
                (drop) -> {
                    robot.servos.pixel(false);
                    sleep(1000);
                    return false;
                }
        );

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .turnTo(Math.toRadians(180))
                        .build()
        );

        waitForStart();


        if (position == GameElementDetection.Position.MIDDLE)
            Actions.runBlocking(middlePixel);
        else if (position == GameElementDetection.Position.LEFT)
            Actions.runBlocking(leftPixel);
        else if (position == GameElementDetection.Position.RIGHT)
            Actions.runBlocking(rightPixel);

        while (!isStopRequested() && opModeIsActive()) {
            telemetry.update();
        }
    }
}