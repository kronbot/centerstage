package org.firstinspires.ftc.teamcode.kronbot.utils;

import static org.firstinspires.ftc.teamcode.kronbot.utils.AutonomousConstants.PixelForward;
import static org.firstinspires.ftc.teamcode.kronbot.utils.AutonomousConstants.SLIDES_COORDINATES;
import static org.firstinspires.ftc.teamcode.kronbot.utils.AutonomousConstants.coordinatesConvert;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.LIFT_REVERSE_CONSTANT;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.REST_POWER;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.SLIDES_SPEED;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.kronbot.KronBot;
import org.firstinspires.ftc.teamcode.kronbot.detection.GameElementDetection;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;
import java.util.List;

public class TrajectoryFactory {
    public static List<TrajectorySequence> createTrajectory(SampleMecanumDrive drive, GameElementDetection.Position position, KronBot robot, Telemetry telemetry, Runnable sleep, boolean isBlue, boolean isClose) {
        AutonomousConstants.Coordinates pixelCoordinates;
        Pose2d parkPose = coordinatesConvert(AutonomousConstants.Park);
        Pose2d cornerPose;
        Pose2d backboardPose;
        Pose2d backPose;
        Pose2d startPose;

       List<TrajectorySequence> trajectories = new ArrayList<TrajectorySequence>();

        int multiplier = isBlue ? -1 : 1;
        int backBoardOffset;

        if (isBlue) {
            backBoardOffset = 7;
            if (!isClose) startPose = coordinatesConvert(AutonomousConstants.StartPoseRightBlue);
            else startPose = coordinatesConvert(AutonomousConstants.StartPoseLeftBlue);
        } else {
            backBoardOffset = 0;
            if (isClose) startPose = coordinatesConvert(AutonomousConstants.StartPoseRightRed);
            else startPose = coordinatesConvert(AutonomousConstants.StartPoseLeftRed);
        }

        if (position == GameElementDetection.Position.RIGHT) {
            pixelCoordinates = AutonomousConstants.PixelRight;
            backboardPose = coordinatesConvert(AutonomousConstants.BackboardRight);
            if (!isBlue)
                backboardPose = coordinatesConvert(AutonomousConstants.BackboardLeft);
            if (isBlue)
                pixelCoordinates.heading = AutonomousConstants.PixelLeft.heading;
        }
        else if (position == GameElementDetection.Position.MIDDLE) {
            pixelCoordinates = AutonomousConstants.PixelMiddle;
            backboardPose = coordinatesConvert(AutonomousConstants.BackboardMiddle);
        }
        else {
            pixelCoordinates = AutonomousConstants.PixelLeft;
            backboardPose = coordinatesConvert(AutonomousConstants.BackboardLeft);
            if (!isBlue)
                backboardPose = coordinatesConvert(AutonomousConstants.BackboardRight);
            if (isBlue)
                pixelCoordinates.heading = AutonomousConstants.PixelRight.heading;
        }

        if (isClose) {
            backPose = coordinatesConvert(AutonomousConstants.Back);
            cornerPose = coordinatesConvert(AutonomousConstants.CornerPark);
        }
        else {
            backPose = coordinatesConvert(AutonomousConstants.FarBack);
            cornerPose = coordinatesConvert(AutonomousConstants.FarCornerPark);
        }

        drive.setPoseEstimate(startPose);
        if (isClose) {
            trajectories.add(drive.trajectorySequenceBuilder(startPose)
                    .setReversed(true)
                    .lineTo(new Vector2d(startPose.getX(), startPose.getY() + PixelForward.y * multiplier))
                    .splineTo(new Vector2d(startPose.getX() + pixelCoordinates.x * multiplier, startPose.getY() + pixelCoordinates.y * multiplier), Math.toRadians(pixelCoordinates.heading * multiplier))
                    .setReversed(false)
                    .lineTo(new Vector2d(startPose.getX() + backPose.getX()+3, (backPose.getY()-3) * multiplier))
                    .build());

            trajectories.add(drive.trajectorySequenceBuilder(new Pose2d(startPose.getX() + backPose.getX()+3, (backPose.getY()-3) * multiplier, parkPose.getHeading()))
                    .setReversed(false)
                    .lineToLinearHeading(new Pose2d(parkPose.getX(), parkPose.getY() * multiplier, parkPose.getHeading()))
                    .lineTo(new Vector2d(backboardPose.getX(), (backboardPose.getY()-0.5) * multiplier))
                    .build());

            trajectories.add(drive.trajectorySequenceBuilder(new Pose2d(backboardPose.getX(),backboardPose.getY()*multiplier, parkPose.getHeading()))
                    .lineTo(new Vector2d(backboardPose.getX() - 2, cornerPose.getY() * multiplier))
                    .lineTo(new Vector2d(cornerPose.getX(), cornerPose.getY() * multiplier))
                    .build());
        }
        else
        {
            trajectories.add(drive.trajectorySequenceBuilder(startPose)
                    .setReversed(true)
                    .lineTo(new Vector2d(startPose.getX(), startPose.getY() + PixelForward.y * multiplier))
                    .splineTo(new Vector2d(startPose.getX() + pixelCoordinates.x * multiplier, startPose.getY() + pixelCoordinates.y * multiplier), Math.toRadians(pixelCoordinates.heading * multiplier))
                    .setReversed(false)
                    .splineTo(new Vector2d(startPose.getX() + backPose.getX(), backPose.getY() * multiplier), Math.toRadians(backPose.getHeading() * multiplier))
                    .setReversed(true)
                    .turn(Math.toRadians(180 * multiplier))
                    .lineTo(new Vector2d((parkPose.getX()) - 35, (backPose.getY() + 2) * multiplier))
                    .build());

            trajectories.add(drive.trajectorySequenceBuilder(
                    new Pose2d(parkPose.getX() - 35, (backPose.getY() + 2) * multiplier, parkPose.getHeading()))
                    .setReversed(true)
                    .lineTo(new Vector2d(backboardPose.getX() + 0.5, (backboardPose.getY()+2) * multiplier + backBoardOffset))
                    .build());

            trajectories.add(drive.trajectorySequenceBuilder(
                    new Pose2d(backboardPose.getX() + 0.5, backboardPose.getY() * multiplier, parkPose.getHeading()))
                    .setReversed(true)
                    .lineTo(new Vector2d(backboardPose.getX() - 3, backboardPose.getY() * multiplier))
                    .lineTo(new Vector2d(backboardPose.getX(), cornerPose.getY() * +multiplier))
                    .build());
        }

        return trajectories;
    }

    public static void raiseSlidesClose(KronBot robot)
    {
        robot.lift.setTargetPosition(SLIDES_COORDINATES);
        robot.lift.setPower(SLIDES_SPEED);
        robot.lift.runToPosition();
        while (robot.lift.isBusy()) {}
        robot.lift.setPower(REST_POWER);
        robot.servos.arm(true);
    }


    public static void raiseSlidesClose(KronBot robot, int position)
    {
        robot.lift.setTargetPosition(SLIDES_COORDINATES);
        robot.lift.setPower(position);
        robot.lift.runToPosition();
        while (robot.lift.isBusy()) {}
        robot.lift.setPower(REST_POWER);
        robot.servos.arm(true);
    }

    public static void resetSlidesClose(KronBot robot, Runnable sleep)
    {
        robot.servos.intakeSpinDown(true);
        sleep.run();
        robot.servos.intakeSpinDown(false);
        robot.servos.arm(false);
        robot.lift.setTargetPosition(0);
        robot.lift.setPower(SLIDES_SPEED * LIFT_REVERSE_CONSTANT);
        robot.lift.runToPosition();
        while (robot.lift.isBusy()) {}
        robot.lift.setPower(REST_POWER);
    }

    public static void raiseSlides(KronBot robot) {
        robot.lift.setTargetPosition(SLIDES_COORDINATES);
        robot.lift.setPower(SLIDES_SPEED);
        robot.lift.runToPosition();
        while (robot.lift.isBusy()) {}
        robot.lift.setPower(REST_POWER);
        robot.servos.arm(true);
    }

    public static void resetSlides(KronBot robot, Runnable sleep) {
        robot.servos.intakeSpinDown(true);
        sleep.run();
        sleep.run();
        robot.servos.arm(false);
        robot.servos.intakeSpinDown(false);
        sleep.run();
        robot.lift.setTargetPosition(0);
        robot.lift.setPower(SLIDES_SPEED * LIFT_REVERSE_CONSTANT);
        robot.lift.runToPosition();
        while (robot.lift.isBusy()) {}
        robot.lift.setPower(REST_POWER);
    }

    public static void takeOut(KronBot robot, Runnable sleep) {
        robot.servos.intakeSpinDown(true);
        sleep.run();
        sleep.run();
        robot.servos.arm(false);
    }

    public static void onlyReset(KronBot robot, Runnable sleep) {
        robot.servos.intakeSpinDown(false);
        robot.servos.arm(false);
        sleep.run();
        robot.lift.setTargetPosition(0);
        robot.lift.setPower(SLIDES_SPEED * LIFT_REVERSE_CONSTANT);
        robot.lift.runToPosition();
        while (robot.lift.isBusy()) {}
        robot.lift.setPower(REST_POWER);
    }
}
