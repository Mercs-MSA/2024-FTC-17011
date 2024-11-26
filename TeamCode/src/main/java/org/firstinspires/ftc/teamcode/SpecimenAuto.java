package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.OTOS_Iterative.DRIVE_TOLERANCE;
import static org.firstinspires.ftc.teamcode.OTOS_Iterative.STRAFE_TOLERANCE;

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
    private static double backAndForthDistance = 43;
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
        GO_TO_INTAKE_STATE,
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
            robot.drive(-8.3, .4, .1);
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
            robot.turnTo(175, .4, .1);
            robot.strafe(24, 1, .1);
            robot.drive(21,1,.1);
//            robot.strafe(15.5,.6,.1); //Behind the first spike
//            robot.drive(-backAndForthDistance, .75, .1); // First push
//            robot.drive(backAndForthDistance, .75, .1);
//            robot.strafe(-12,.7,.1);
//            robot.drive(backAndForthDistance, .75, .1); //Second push
//            robot.drive(-backAndForthDistance, .75, .1);
            STRAFE_TOLERANCE = 6;
            robot.strafe(28.3,1,.1);
            STRAFE_TOLERANCE = 2.5;
            DRIVE_TOLERANCE = 5.5;
            robot.drive(-backAndForthDistance - 6, .75, .1); //Last push
            DRIVE_TOLERANCE = 3;
            intakes.specSetPos(Constants.specimenHoldPos);
            currentState = AUTO_STATE.INTAKE_STATE;
            slides.slideSetPos(Constants.highSpecimenPos);
        }
    }
    int a = 0;
    public void goToStagingProcess() {
        if (!isDriving) {
            i = 0;
            isDriving = true;
            robot.drive(18, 1, .1);
            if (a == 0) {
                robot.strafe(-57.2, 1, .1);
                a++;
            } else {
                robot.strafe(-60, 1, .1);
            }
            robot.turnTo(0, .4, .1);
            if (robot.isPowerZero()) {
                currentState = AUTO_STATE.SPEC_READY_STATE;
            }
        }
    }

    public void goToIntakeProcess() {
        if (!isDriving) {
            isDriving = true;
            robot.drive(4, .5, .1);
            robot.turnTo(180, .4, .1);
            robot.strafe(57.2, 1, .1);
            robot.drive(-27, 1, .1);
            if (robot.isPowerZero()) {
                intakes.specSetPos(Constants.specimenHoldPos);
                currentState = AUTO_STATE.INTAKE_STATE;
                slides.slideSetPos(Constants.highSpecimenPos);
            }
        }
    }

    public void driveEndProcess() {

    }

    public void startMechanismProcess() {
//        if (Math.abs(pivot.getPos() + 40) > 5) {
        intakes.pivotSetPos(Constants.intakePivotMidPos);
//            pivot.pivotSetPos(-60);
        intakes.specSetPos(Constants.specimenHoldPos);
//        } else {
//            pivot.resetPos();
        currentState = AUTO_STATE.SPEC_READY_STATE;
        isDriving = false;
//        }
    }

    public void specReadyProcess() { //SCORING AHHHHH
        pivot.pivotSetPos(constants.pivotUpPos);
        if (Math.abs(pivot.getPos() - Constants.pivotUpPos) < 325) {
            slides.slideSetPos(Constants.highSpecimenPos);
            if (Math.abs(slides.getLeftPos() - Constants.highSpecimenPos) < 50) {
                currentState = AUTO_STATE.DO_NOTHING_STATE;
                isDriving = false;
                currentDriveState = AUTO_DRIVE_STATE.STAGING_TO_SCORE_STATE;
            }
        }
    }
    int i = 0;
    int e = 0;
    public void specScoreProcess() {
        if (i == 0) {
            slides.slideSetPos(Constants.highSpecScorePos);
            i++;
        }
        if (Math.abs(slides.getLeftPos() - Constants.highSpecScorePos) < 20) {
            intakes.specSetPos(Constants.specimenScorePos);
            slides.slideSetPos(0);
        }
        if (slides.getLeftPos() < 20 && i == 1 && e == 0) {
            currentDriveState = AUTO_DRIVE_STATE.PUSH_ALL_STATE;
            currentState = AUTO_STATE.WHILE_PUSHING_STATE;
            isDriving = false;
            i++;
            e++;
        }
        if (slides.getLeftPos() < 20 && i == 1 && e == 1) {
            if (a == 1) {
                currentDriveState = AUTO_DRIVE_STATE.GO_TO_INTAKE_STATE;
                currentState = AUTO_STATE.DO_NOTHING_STATE;
                isDriving = false;
            } else {
                currentState = AUTO_STATE.END_STATE;
                currentDriveState = AUTO_DRIVE_STATE.END_STATE;
                isDriving = false;
            }
        }
    }

    public void intakeStateProcess() {
        currentDriveState = AUTO_DRIVE_STATE.GO_TO_STAGING_STATE;
        isDriving = false;
        currentState = AUTO_STATE.DO_NOTHING_STATE;
    }

    public void mechanismsEndProcess() {
        pivot.pivotSetPos(0);
    }

    public void doNothingProcess() {}

    private void processDriveState() {
        switch (currentDriveState) {
            case START_STATE: startDriveProcess(); break;
            case STAGING_TO_SCORE_STATE: stagingToSpecProcess(); break;
            case PUSH_ALL_STATE: pushAllProcess(); break;
            case GO_TO_STAGING_STATE: goToStagingProcess(); break;
            case GO_TO_INTAKE_STATE: goToIntakeProcess(); break;
            case END_STATE: driveEndProcess(); break;
            case DO_NOTHING_STATE: doNothingProcess(); break;
        }
    }

    private void processOtherState() {
        switch (currentState) {
            case START_STATE: startMechanismProcess(); break;
            case SPEC_READY_STATE: specReadyProcess(); break;
            case SPEC_SCORE_STATE: specScoreProcess(); break;
            case INTAKE_STATE: intakeStateProcess(); break;
            case END_STATE: mechanismsEndProcess(); break;
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
