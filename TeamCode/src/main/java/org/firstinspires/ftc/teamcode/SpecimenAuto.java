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
    private static double backAndForthDistance = 48;
    final private OTOS_Iterative robot = new OTOS_Iterative(this);
    private Constants constants;
    private Intakes intakes;
    private Pivot pivot;
    private Slides slides;
    private enum AUTO_DRIVE_STATE {
        START_STATE,
        STAGING_TO_SCORE_STATE,
        END_STATE,
        DO_NOTHING_STATE,
    }

    private AUTO_DRIVE_STATE currentDriveState = AUTO_DRIVE_STATE.START_STATE;
    private boolean isDriving = false;
    private enum AUTO_STATE {
        START_STATE,
        SPEC_READY_STATE,
        SPEC_SCORE_STATE,
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
            robot.drive(-8, .6, .1);
            isDriving = true;
        }
        if (robot.isPowerZero()) {
            currentDriveState = AUTO_DRIVE_STATE.DO_NOTHING_STATE;
            currentState = AUTO_STATE.SPEC_SCORE_STATE;
        }
    }



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
        if (Math.abs(pivot.getPos() - constants.pivotUpPos) < 100) {
            slides.slideSetPos(constants.highSpecimenPos);
            if (robot.isPowerZero()) {
                currentDriveState = AUTO_DRIVE_STATE.STAGING_TO_SCORE_STATE;
            }
        }
    }

    public void specScoreProcess() {
        slides.slideSetPos(constants.highSpecScorePos);
        if (Math.abs(slides.getLeftPos() - constants.highSpecimenPos) < 5) {
            intakes.specSetPos(constants.specimenScorePos);
            slides.slideSetPos(0);
        }
    }

    public void doNothingProcess() {}

    private void processDriveState() {
        switch (currentDriveState) {
            case START_STATE: startDriveProcess(); break;
            case STAGING_TO_SCORE_STATE: stagingToSpecProcess(); break;
            case DO_NOTHING_STATE: doNothingProcess(); break;
        }
    }

    private void processOtherState() {
        switch (currentState) {
            case START_STATE: startMechanismProcess(); break;
            case SPEC_READY_STATE: specReadyProcess(); break;
            case SPEC_SCORE_STATE: specScoreProcess(); break;
            case DO_NOTHING_STATE: doNothingProcess(); break;
        }
    }

    @Override
    public void loop() {
        processDriveState();
        processOtherState();
        telemetry.addData("Pivot difference 1: ", Math.abs(pivot.getPos() + 40));
        telemetry.addData("Pivot difference 2: ", Math.abs(pivot.getPos() - constants.pivotUpPos));
        telemetry.update();
    }
}
