package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subSystems.Intakes;
import org.firstinspires.ftc.teamcode.subSystems.Pivot;
import org.firstinspires.ftc.teamcode.subSystems.Slides;

@Autonomous
@Config
public class SpecimenAuto extends OpMode {
    FtcDashboard dash;
    private static double backAndForthDistance = 41;
    final private OTOS_Iterative robot = new OTOS_Iterative(this);
    private Constants constants;
    private Intakes intakes;
    private Pivot pivot;
    private Slides slides;

    private enum AUTO_DRIVE_STATE {
        START_STATE,
        STAGING_TO_SCORE_STATE,
        PUSH_ALL_STATE,
        GO_TO_STAGING_STATE,
        END_STATE,
        DO_NOTHING_STATE,
    }

    private AUTO_DRIVE_STATE currentDriveState = AUTO_DRIVE_STATE.START_STATE;
    private boolean isDriving = false;
    private enum AUTO_STATE {
        START_STATE,
        SPEC_READY_STATE,
        SPEC_SCORE_STATE,
        WHILE_PUSHING_STATE,
        INTAKE_STATE,
        END_STATE,
        DO_NOTHING_STATE
    }

    private AUTO_STATE currentState = AUTO_STATE.START_STATE;

    @Override
    public void init() {
        robot.initialize(true, true);

        FtcDashboard dash = FtcDashboard.getInstance();

        try {
            initializeSubSystems();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    public void initializeSubSystems() throws InterruptedException {
        intakes = new Intakes(hardwareMap);
        pivot = new Pivot(hardwareMap, constants.pivotP, constants.pivotI, constants.pivotD, constants.pivotF);
        slides = new Slides(hardwareMap);
    }

    public void startDriveProcess() {
        if (!isDriving) {
            robot.drive(-20, .6, .1);
            isDriving = true;
            currentDriveState = AUTO_DRIVE_STATE.DO_NOTHING_STATE;
        }
    }

    public void stagingToSpecProcess() {
        if (!isDriving) {
            robot.drive(-8.3, .5, .1);
            isDriving = true;
        }
        if (robot.isPowerZero()) {
            currentDriveState = AUTO_DRIVE_STATE.DO_NOTHING_STATE;
            currentState = AUTO_STATE.SPEC_SCORE_STATE;
        }
    }

    public void pushAllProcess() { //Pushes all spike samples
        if (pivot.getPos() < 300) {
            intakes.pivotSetPos(constants.intakePivotScorePos);
        }
        if (!isDriving) {
            isDriving = true;
            robot.drive(4, .75, .1);
            robot.strafe(-25, .75, .1);
            robot.drive(-21,.75,.1);
            robot.strafe(-20.287,.6,.1); //Behind the first spike
            robot.drive(backAndForthDistance, .75, .1); // First push
            robot.drive(-backAndForthDistance, .75, .1);
//            robot.strafe(-12,.7,.1);
//            robot.drive(backAndForthDistance, .75, .1); //Second push
//            robot.drive(-backAndForthDistance, .75, .1);
            robot.strafe(-10,.6,.1);
            robot.drive(backAndForthDistance + 4, .75, .1); //Last push
            robot.turnTo(175, .7, .1);
            currentDriveState = AUTO_DRIVE_STATE.GO_TO_STAGING_STATE;
//            currentState = AUTO_STATE.INTAKE_STATE;
        }
    }

    public void goToStagingProcess() {}

    public void startMechanismProcess() {
//        if (Math.abs(pivot.getPos() + 40) > 5) {
        intakes.pivotSetPos(constants.intakePivotMidPos);
//            pivot.pivotSetPos(-60);
        intakes.specSetPos(constants.specimenHoldPos);
//        } else {
//            pivot.resetPos();
        currentState = AUTO_STATE.SPEC_READY_STATE;
        isDriving = false;
//        }
    }

    public void specReadyProcess() { //SCORING AHHHHH
        pivot.pivotSetPos(constants.pivotUpPos);
        if (Math.abs(pivot.getPos() - constants.pivotUpPos) < 110) {
            slides.slideSetPos(constants.highSpecimenPos);
            if (robot.isPowerZero() && Math.abs(slides.getLeftPos() - constants.highSpecimenPos) < 50) {
                currentDriveState = AUTO_DRIVE_STATE.STAGING_TO_SCORE_STATE;
            }
        }
    }
    int i = 0;
    public void specScoreProcess() {
        if (i == 0) {
            slides.slideSetPos(constants.highSpecScorePos);
            i++;
        }
        if (Math.abs(slides.getLeftPos() - constants.highSpecScorePos) < 20) {
            intakes.specSetPos(constants.specimenScorePos);
            slides.slideSetPos(0);
        }
        if (slides.getLeftPos() < 20 && i == 1) {
            currentDriveState = AUTO_DRIVE_STATE.PUSH_ALL_STATE;
            currentState = AUTO_STATE.WHILE_PUSHING_STATE;
            isDriving = false;
            i++;
        }
    }

    public void whilePushingProcess() {
//        if (pivot.getPos() > 0) {
//            pivot.pivotSetPos(-200);
//        }
//        if (pivot.getPos() < 0) {
//            pivot.pivotSetPos(0);
//            pivot.resetPos();
//            pivot.setPow(0);
//        }
    }

    public void intakeProcess() {}

    public void doNothingProcess() {}

    private void processDriveState() {
        switch (currentDriveState) {
            case START_STATE: startDriveProcess(); break;
            case STAGING_TO_SCORE_STATE: stagingToSpecProcess(); break;
            case PUSH_ALL_STATE: pushAllProcess(); break;
            case GO_TO_STAGING_STATE: goToStagingProcess(); break;
            case DO_NOTHING_STATE: doNothingProcess(); break;
        }
    }

    private void processOtherState() {
        switch (currentState) {
            case START_STATE: startMechanismProcess(); break;
            case SPEC_READY_STATE: specReadyProcess(); break;
            case SPEC_SCORE_STATE: specScoreProcess(); break;
            case WHILE_PUSHING_STATE: whilePushingProcess(); break;
            case INTAKE_STATE: intakeProcess(); break;
            case DO_NOTHING_STATE: doNothingProcess(); break;
        }
    }

    @Override
    public void loop() {
        processDriveState();
        processOtherState();
        telemetry.addData("Pivot difference 2: ", Math.abs(pivot.getPos() - constants.pivotUpPos));
        telemetry.addData("Slide difference: ", Math.abs(slides.getLeftPos() - constants.highSpecScorePos));
        telemetry.addData("Current slide pos: ", slides.getLeftPos());
        telemetry.addData("Current pivot pos: ", pivot.getPos());
        telemetry.addData("Drive state: ", currentDriveState);
        telemetry.addData("Other state: ", currentState);
//        telemetry.update();
    }
}
