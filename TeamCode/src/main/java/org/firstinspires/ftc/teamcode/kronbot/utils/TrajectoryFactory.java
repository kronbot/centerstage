package org.firstinspires.ftc.teamcode.kronbot.utils;

import static org.firstinspires.ftc.teamcode.kronbot.utils.AutonomousConstants.PixelForward;
import static org.firstinspires.ftc.teamcode.kronbot.utils.AutonomousConstants.coordinatesConvert;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.kronbot.detection.GameElementDetection;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

public class TrajectoryFactory {
    public static TrajectorySequence createTrajectory(SampleMecanumDrive drive, GameElementDetection.Position position, boolean isBlue, boolean isClose) {
        AutonomousConstants.Coordinates pixelCoordinates;
        Pose2d parkPose = coordinatesConvert(AutonomousConstants.ClosePark);
        Pose2d backPose;
        Pose2d startPose;

        int multiplier = isBlue ? -1 : 1;

        if (isBlue) {
            if (!isClose) startPose = coordinatesConvert(AutonomousConstants.StartPoseRightBlue);
            else startPose = coordinatesConvert(AutonomousConstants.StartPoseLeftBlue);
        } else {
            if (isClose) startPose = coordinatesConvert(AutonomousConstants.StartPoseRightRed);
            else startPose = coordinatesConvert(AutonomousConstants.StartPoseLeftRed);
        }

        if (position == GameElementDetection.Position.RIGHT) {
            pixelCoordinates = AutonomousConstants.PixelRight;
            if (isBlue)
                pixelCoordinates.heading = AutonomousConstants.PixelLeft.heading;
        }
        else if (position == GameElementDetection.Position.MIDDLE)
            pixelCoordinates = AutonomousConstants.PixelMiddle;
        else {
            pixelCoordinates = AutonomousConstants.PixelLeft;
            if (isBlue)
                pixelCoordinates.heading = AutonomousConstants.PixelRight.heading;
        }

        if (isClose) backPose = coordinatesConvert(AutonomousConstants.FarBack);
        else backPose = coordinatesConvert(AutonomousConstants.Back);

        TrajectorySequence trajectory;

        if (isClose) {
            trajectory = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(startPose.getX(), startPose.getY() + PixelForward.y * multiplier))
                .splineTo(new Vector2d(startPose.getX() + pixelCoordinates.x * multiplier, startPose.getY() + pixelCoordinates.y * multiplier), Math.toRadians(pixelCoordinates.heading * multiplier))
                .waitSeconds(1)
                .lineTo(new Vector2d(startPose.getX(), backPose.getY() * multiplier))
                .lineToLinearHeading(new Pose2d(parkPose.getX(), parkPose.getY() * multiplier, parkPose.getHeading()))
                .build();
        } else {
            trajectory = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(startPose.getX(), startPose.getY() + PixelForward.y * multiplier))
                .splineTo(new Vector2d(startPose.getX() + pixelCoordinates.x * multiplier, startPose.getY() + pixelCoordinates.y * multiplier), Math.toRadians(pixelCoordinates.heading * multiplier))
                .waitSeconds(1)
                .splineTo(new Vector2d(startPose.getX(), backPose.getY() * multiplier), Math.toRadians(backPose.getHeading() * multiplier))
                .turn(Math.toRadians(-90 * multiplier))
                .lineToConstantHeading(new Vector2d(parkPose.getX() - 30, backPose.getY() * multiplier))
                .splineTo(new Vector2d(parkPose.getX(), parkPose.getY() * multiplier), 0)
                .build();
        }

        return trajectory;
    }
}
