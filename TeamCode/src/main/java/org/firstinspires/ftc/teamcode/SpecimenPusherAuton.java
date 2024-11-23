package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subSystems.Intakes;
import org.firstinspires.ftc.teamcode.subSystems.Pivot;
import org.firstinspires.ftc.teamcode.subSystems.Slides;

@Autonomous
@Config
public class SpecimenPusherAuton extends LinearOpMode {
    FtcDashboard dash;
    private static double backAndForthDistance = 48;
    final private OTOS robot = new OTOS(this);
    private Constants constants;
    private Intakes intakes;
    private Pivot pivot;
    private Slides slides;

    public void initializeSubSystems() throws InterruptedException {
        intakes = new Intakes(hardwareMap);
        pivot = new Pivot(hardwareMap, constants.pivotP, constants.pivotI, constants.pivotD, constants.pivotF);
        slides = new Slides(hardwareMap);
    }

    @Override
    public void runOpMode() {
        // Initialize the robot hardware & Turn on telemetry
        robot.initialize(true, true);

        FtcDashboard dash = FtcDashboard.getInstance();

        try {
            initializeSubSystems();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        // Wait for driver to press start
        telemetry.addData(">", "Touch Play to run Auto");

        waitForStart();

        robot.resetHeading();  // Reset heading to set a baseline for Auto
        if (opModeIsActive()) {
            intakes.specSetPos(constants.specimenHoldPos);
            robot.drive(-20, 0.60, 0.1);
            intakes.pivotSetPos(constants.intakePivotScorePos);
            sleep(200);
            pivot.pivotSetPos(-60);
            pivot.resetPos();
            pivot.setPow(1);
            sleep(500);
            pivot.pivotSetPos(constants.pivotUpPos);
            intakes.pivotSetPos(constants.intakePivotGrabPos);
            sleep(750);
            slides.slideSetPos(constants.highSpecimenPos);
            pivot.setPow(0);
            sleep(750);
            robot.drive(-8, 0.60, 0.1);
            sleep(750);
            slides.slideSetPos(constants.highSpecScorePos);
            sleep(750);
            intakes.specSetPos(constants.specimenScorePos); //Score specimen
            sleep(500);
            pivot.setPow(.6);
//            telemetry.addData("Pivot power: ", pivot.getPow());
//            telemetry.update();
            slides.slideSetPos(0);
            sleep(300);
            pivot.pivotSetPos(constants.pivotDownPos);
            robot.drive(8, .1, .1);
            pivot.setPow(0);
            sleep(300);
            intakes.pivotSetPos(constants.intakePivotScorePos);
            sleep(300);
            pivot.resetPos();
            robot.strafe(-28, .6, .1);
            robot.drive(-30, .6, .1);
            robot.strafe(-11, .6, .1);
            robot.drive(backAndForthDistance, .75, .1); // First push
            robot.drive(-backAndForthDistance, .75, .1);
            robot.strafe(-10, .6, .1);
            robot.drive(backAndForthDistance, .75, .1); //Second push
            robot.drive(-backAndForthDistance, .75, .1);
            robot.strafe(-8, .6, .1);
            robot.drive(backAndForthDistance + 5, .75, .1); //Last push
        }
    }
}