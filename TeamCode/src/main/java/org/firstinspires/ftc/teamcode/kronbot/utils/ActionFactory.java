package org.firstinspires.ftc.teamcode.kronbot.utils;

import static org.firstinspires.ftc.teamcode.kronbot.utils.AutonomousConstants.Back;
import static org.firstinspires.ftc.teamcode.kronbot.utils.AutonomousConstants.ClosePark;
import static org.firstinspires.ftc.teamcode.kronbot.utils.AutonomousConstants.FarPark;
import static org.firstinspires.ftc.teamcode.kronbot.utils.AutonomousConstants.PixelForward;
import static org.firstinspires.ftc.teamcode.kronbot.utils.AutonomousConstants.PixelLeft;
import static org.firstinspires.ftc.teamcode.kronbot.utils.AutonomousConstants.PixelMiddle;
import static org.firstinspires.ftc.teamcode.kronbot.utils.AutonomousConstants.PixelRight;
import static org.firstinspires.ftc.teamcode.kronbot.utils.AutonomousConstants.coordinatesConvert;

import static java.lang.Thread.sleep;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.kronbot.KronBot;
import org.firstinspires.ftc.teamcode.kronbot.detection.GameElementDetection;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.function.Supplier;

public class ActionFactory {
    public static SequentialAction ActionBuilder(KronBot robot, MecanumDrive drive, boolean isBlue, boolean isClose, GameElementDetection.Position position, Runnable sleep) {
        Pose2d pixelPose = new Pose2d(0, 0, 0);
        if (position == GameElementDetection.Position.RIGHT)
            pixelPose = coordinatesConvert(PixelRight);
        else if (position == GameElementDetection.Position.MIDDLE)
            pixelPose = coordinatesConvert(PixelMiddle);
        else
            pixelPose = coordinatesConvert(PixelLeft);

        Pose2d parkPose = new Pose2d(0, 0, 0);
        Pose2d backPose = coordinatesConvert(Back);
        if (isClose) parkPose = coordinatesConvert(ClosePark);
        else parkPose = coordinatesConvert(FarPark);

        if (isBlue) parkPose = new Pose2d(parkPose.position.x, -parkPose.position.y, -ClosePark.heading);

        SequentialAction action = new SequentialAction(
            drive.actionBuilder(drive.pose)
                .lineToX(PixelForward.x)
                .splineTo(new Vector2d(pixelPose.position.x, pixelPose.position.y), pixelPose.heading)
                .build(),
            (drop) -> {
                sleep.run();
                return false;
            }
//            drive.actionBuilder(drive.pose)
//                    .setReversed(true)
//                    .lineToX(backPose.position.x)
//                    .setReversed(false)
//                    .build()

//            drive.actionBuilder(drive.pose)
//                .strafeToLinearHeading(new Vector2d(backPose.position.x, backPose.position.y), backPose.heading)
////                    .strafeTo(new Vector2d(backPose.position.x, backPose.position.y))
////                    .turnTo(backPose.heading)
//                .strafeTo(new Vector2d(isBlue ? -parkPose.position.x : parkPose.position.x, parkPose.position.y))
//                .build()
//            (drop) -> {
//                sleep.run();
//                // raise slides
//                sleep.run();
//                // raise cupa
//                sleep.run();
//                // start wheel
//                sleep.run();
//                sleep.run();
//                // stop wheel
//                return false;
//            }
        );

        return action;
    }
}
