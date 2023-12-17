package org.firstinspires.ftc.teamcode.kronbot.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.kronbot.KronBot;
import org.firstinspires.ftc.teamcode.kronbot.components.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.kronbot.components.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;
import org.firstinspires.ftc.teamcode.kronbot.utils.drivers.LiftDriver;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.Button;

/**
 * The main TeleOP program for the driving period of the game.
 *
 * @version 1.0
 */
@TeleOp(name = "Slide Test", group = Constants.TEST_GROUP)
public class SliderTestOp extends LinearOpMode {
    private final KronBot robot = new KronBot();

    LiftDriver liftDriver;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        liftDriver = new LiftDriver(hardwareMap, gamepad1);
        liftDriver.init(false, false);

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addLine("Initialization Ready");
            telemetry.update();
        }

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            liftDriver.testSlide();
            liftDriver.showInfo(telemetry);

            telemetry.update();
        }
    }
}
