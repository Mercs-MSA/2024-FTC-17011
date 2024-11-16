/* Created by Phil Malone. 2023.
    This class illustrates my simplified Odometry Strategy.
    It implements basic straight line motions but with heading and drift controls to limit drift.
    See the readme for a link to a video tutorial explaining the operation and limitations of the code.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subSystems.Intakes;
import org.firstinspires.ftc.teamcode.subSystems.Pivot;
import org.firstinspires.ftc.teamcode.subSystems.Slides;

/*
 * This OpMode illustrates an autonomous opmode using simple Odometry
 * All robot functions are performed by an external "Robot" class that manages all hardware interactions.
 * Pure Drive or Strafe motions are maintained using two Odometry Wheels.
 * The IMU gyro is used to stabilize the heading during all motions
 */

@Autonomous
public class HighBasketAuton extends LinearOpMode {
    FtcDashboard dash;
    private static double firstTurnAngle = -70;
    private static double toBasketDistance = 50;
    private static double secondTurnAngle = -185;
    // get an instance of the "Robot" class.
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
    @Override public void runOpMode() {
        FtcDashboard dash = FtcDashboard.getInstance();
        // Initialize the robot hardware & Turn on telemetry
        robot.initialize(true, true);

        try {
            initializeSubSystems();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        // Wait for driver to press start
        telemetry.addData(">", "Touch Play to run Auto");

        waitForStart();

        robot.resetHeading();  // Reset heading to set a baseline for Auto

        // Run Auto if stop was not pressed.
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
            telemetry.addData("Pivot power: ", pivot.getPow());
            telemetry.update();
            slides.slideSetPos(0);
            sleep(300);
            pivot.pivotSetPos(30);
            robot.drive(8, .1, .1);
            pivot.setPow(0);
            sleep(300);
            intakes.pivotSetPos(constants.intakePivotScorePos);
            sleep(300);
            pivot.superReset();
            robot.turnTo(firstTurnAngle, .4, .1); //Going to basket
            robot.drive(toBasketDistance, .6, .1);
            robot.turnTo(secondTurnAngle, .4, .1);
            sleep(300);
            intakes.setIntakePower(constants.intakeCollectPow); //First spike
            while ((constants.extendPos - slides.getLeftPos()) > 10) {
                intakes.pivotSetPos(constants.intakePivotGrabPos);
                slides.slideSetPos(constants.extendPos);
            }
            sleep(700);
            slides.slideSetPos(0);
            pivot.setPow(1);
            sleep(200);
            while (pivot.getPos() < (constants.pivotUpPos - 50)) { //First up
                pivot.pivotSetPos(constants.pivotUpPos);
            }
            while (slides.getLeftPos() < (constants.highBasketPos - 10)) {
                slides.slideSetPos(constants.highBasketPos);
            }
            robot.drive(-2, .25, .1);
            for (int i = 0; i < 50; i++) { //Score first sample
                intakes.pivotSetPos(constants.intakePivotScorePos);
                intakes.setIntakePower(constants.intakeScorePow);
            }
            robot.drive(3, .1, .1);
            sleep(500);
            pivot.pivotSetPos(0);
            slides.slideSetPos(0);
            sleep(500);
            robot.drive(2, .1, .1);
        }
        telemetry.update();
        robot.incrementOpModeCounter();
    }
}