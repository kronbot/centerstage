package org.firstinspires.ftc.teamcode.kronbot.utils;

import static org.firstinspires.ftc.teamcode.kronbot.utils.AutonomousConstants.PixelForward;
import static org.firstinspires.ftc.teamcode.kronbot.utils.AutonomousConstants.SLIDES_COORDINATES;
import static org.firstinspires.ftc.teamcode.kronbot.utils.AutonomousConstants.coordinatesConvert;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.LIFT_REVERSE_CONSTANT;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.REST_POWER;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.SLIDES_SPEED;

import static java.lang.Thread.sleep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.kronbot.KronBot;
import org.firstinspires.ftc.teamcode.kronbot.detection.GameElementDetection;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

public class TrajectoryFactory {
    public static TrajectorySequence createTrajectory(SampleMecanumDrive drive, GameElementDetection.Position position, KronBot robot, Telemetry telemetry, Runnable sleep, boolean isBlue, boolean isClose) {
        AutonomousConstants.Coordinates pixelCoordinates;
        Pose2d parkPose = coordinatesConvert(AutonomousConstants.Park);
        Pose2d cornerPose = coordinatesConvert(AutonomousConstants.CornerPark);
        Pose2d backboardPose;
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

        if (isClose) backPose = coordinatesConvert(AutonomousConstants.Back);
        else backPose = coordinatesConvert(AutonomousConstants.FarBack);

        TrajectorySequence trajectory;

        drive.setPoseEstimate(startPose);
        if (isClose) {
            trajectory = drive.trajectorySequenceBuilder(startPose)
                    .setReversed(true)
                    .lineTo(new Vector2d(startPose.getX(), startPose.getY() + PixelForward.y * multiplier))
                    .splineTo(new Vector2d(startPose.getX() + pixelCoordinates.x * multiplier, startPose.getY() + pixelCoordinates.y * multiplier), Math.toRadians(pixelCoordinates.heading * multiplier))
                    .setReversed(false)
                    .addDisplacementMarker(() -> {
                        robot.lift.setTargetPosition(SLIDES_COORDINATES);
                        robot.lift.setPower(SLIDES_SPEED);
                        robot.lift.runToPosition();
                        while (robot.lift.isBusy()) { telemetry.update(); }
                        robot.lift.setPower(REST_POWER);
                        robot.servos.arm(true);
                    })
                    .lineTo(new Vector2d(startPose.getX() + backPose.getX(), backPose.getY() * multiplier))
                    .lineToLinearHeading(new Pose2d(parkPose.getX(), parkPose.getY() * multiplier, parkPose.getHeading()))
                    .lineTo(new Vector2d(backboardPose.getX(), backboardPose.getY() * multiplier))
                    .addDisplacementMarker(() -> {
                        robot.servos.intakeSpinDown(true);
                        sleep.run();
                        sleep.run();
                        robot.servos.intakeSpinDown(false);
                        robot.servos.arm(false);
                        sleep.run();
                        robot.lift.setTargetPosition(0);
                        robot.lift.setPower(SLIDES_SPEED * LIFT_REVERSE_CONSTANT);
                        robot.lift.runToPosition();
                        while (robot.lift.isBusy()) {}
                        robot.lift.setPower(REST_POWER);
                    })
                    .lineTo(new Vector2d(backboardPose.getX() - 2, cornerPose.getY() * multiplier))
                    .lineTo(new Vector2d(cornerPose.getX(), cornerPose.getY() * multiplier))
                    .build();
        } else {
            trajectory = drive.trajectorySequenceBuilder(startPose)
                    .setReversed(true)
                    .lineTo(new Vector2d(startPose.getX(), startPose.getY() + PixelForward.y * multiplier))
                    .splineTo(new Vector2d(startPose.getX() + pixelCoordinates.x * multiplier, startPose.getY() + pixelCoordinates.y * multiplier), Math.toRadians(pixelCoordinates.heading * multiplier))
                    .setReversed(false)
                    .splineTo(new Vector2d(startPose.getX(), backPose.getY() * multiplier), Math.toRadians(backPose.getHeading() * multiplier))
                    .turn(Math.toRadians(180 * multiplier))
                    .setReversed(true)
                    .lineToConstantHeading(new Vector2d(parkPose.getX() - 30, backPose.getY() * multiplier))
//                    .addTemporalMarker(SLIDES_TIME, () -> {
//                        robot.lift.setTargetPosition(2000);
//                        robot.lift.setPower(SLIDES_SPEED);
//                        robot.lift.runToPosition();
//                        while (robot.lift.isBusy()) { telemetry.update(); }
//                        robot.lift.setPower(REST_POWER);
//                    })
                    .splineTo(new Vector2d(parkPose.getX(), parkPose.getY() * multiplier), 0)
//                    .addTemporalMarker(SLIDES_TIME + SLIDES_WAIT_TIME, () -> {
//                        robot.lift.setTargetPosition(0);
//                        robot.lift.setPower(SLIDES_SPEED * LIFT_REVERSE_CONSTANT);
//                        robot.lift.runToPosition();
//                        while (robot.lift.isBusy()) {}
//                        robot.lift.setPower(REST_POWER);
//                    })
                    .setReversed(false)
                    .build();
        }

        return trajectory;
    }
}
