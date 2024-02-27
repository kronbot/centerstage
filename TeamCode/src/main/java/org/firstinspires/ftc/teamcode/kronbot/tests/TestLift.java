package org.firstinspires.ftc.teamcode.kronbot.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.kronbot.KronBot;
import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;

/**
 * The main TeleOP program for the driving period of the game.
 *
 * @version 1.0
 */
@Disabled
@TeleOp(name = "Slide Test", group = Constants.TEST_GROUP)
public class TestLift extends LinearOpMode {
    private final KronBot robot = new KronBot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initLift(hardwareMap);

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addLine("Initialization Ready");
            telemetry.update();
        }

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            robot.lift.run(gamepad2.right_trigger - gamepad2.left_trigger);
            robot.lift.showInfo(telemetry);

            telemetry.update();
        }
    }
}
