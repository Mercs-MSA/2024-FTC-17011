package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.subSystems.Intakes;
import org.firstinspires.ftc.teamcode.subSystems.Pivot;
import org.firstinspires.ftc.teamcode.subSystems.Slides;


import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.startingPoseLeft;

public class HighBasketAuto extends OpMode {
    private Constants constants;
    private Follower follower;
    private double botHeading = startingPoseLeft.getHeading();
    private Intakes intakes;
    private Pivot pivot;
    private Slides slides;

    private enum AUTO_STATE {
        START_STATE,
        PATH_ACTIVE,
        PATH_TO_SPEC_READY,
        PATH_TO_BASKET_READY,
        FIRST_MECHANISMS_READY,
        SECOND_MECHANISMS_READY,
        THIRD_MECHANISMS_READY,
        FIRST_SPIKE_READY,
        FIRST_BASKET_READY,
        SECOND_SPIKE_READY,
        SECOND_BASKET_READY
    }

    private AUTO_STATE currentState = AUTO_STATE.START_STATE;
    private AUTO_STATE primerState = AUTO_STATE.START_STATE;

    private static ElapsedTime timeoutTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private int timeoutPeriod = 0;

    //------------------------------------------------------------------------------------------------------------------------
    public static final Point startPoint = new Point (startingPoseLeft.getX(), startingPoseLeft.getY(), Point.CARTESIAN);
    public static final Point specStagingPoint = new Point (0, 0, Point.CARTESIAN); //Before going to drop
    public static final Point specScorePoint = new Point (0, 0, Point.CARTESIAN); //At submersible to score

    public static final Path startToStaging = new Path(new BezierCurve(startPoint, specStagingPoint)); //Go to staging point
    public static final Path stagingToScore = new Path(new BezierCurve(specStagingPoint, specScorePoint)); //Go to submersible
    // ------------------------------------------------------------------------------------------------------------------------
    public static final Point basketPoint = new Point (0, 0, Point.CARTESIAN); //At basket

    public static final Path scoreToBasket = new Path(new BezierCurve(specScorePoint, basketPoint)); //Go to the basket
    //------------------------------------------------------------------------------------------------------------------------
    public static final double startToSpecHeading = Math.toRadians(0);
    public static final double specToBasketHeading = Math.toRadians(45);
    public static final double spikeOneHeading = Math.toRadians(0);
    public static final double spikeTwoHeading = Math.toRadians(0);

    @Override
    public void init() {
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startingPoseLeft);
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

    private void setupPath(Path pathToFollow, double endHeading) {
        double currentHeading = botHeading;
        follower.followPath(pathToFollow);
        pathToFollow.setLinearHeadingInterpolation(currentHeading, endHeading);
        botHeading = endHeading;
    }


    private void restartTimeout(int timeout) {
        timeoutPeriod = timeout;
        timeoutTimer.reset();
    }

    private boolean pathIsBusy() {
        return follower.isBusy();
    }

    private boolean hasTimedOut() {
        if (timeoutTimer.time() < timeoutPeriod)
            return false;
        else
            return true;
    }

    //Close Specimen Intake
    //Drive "Backwards" close to Submersible
    //Pivot Up >> Slides Up
    //Drive "Backwards" into Scoring Position
    //Slides Down >> Specimen Open
    //All set to zero & Intake Pivot to Grab Pos
    //Drive to Baskets
    //Turn if Needed
    //Extend Lifts & Spin Intake
    //Close Intake
    //Pull Back
    //Pivot Up >> Slides Up
    //Intake Pivot Scoring Pos >> Open >> Grab Pos
    //Repeat Last 6 for Second Yellow
    //All set to zero

    public void processStartState() {
        intakes.specSetPos(constants.specimenHoldPos);
        setupPath(startToStaging, startToSpecHeading);
        currentState = AUTO_STATE.PATH_ACTIVE;
        primerState = AUTO_STATE.FIRST_MECHANISMS_READY;
    }

    public void processPathActive() {
        if (!pathIsBusy()) {
            currentState = primerState;
        }
    }

    //First Call. Prepare for spec score.
    public void processMechanismsReady1() {
        pivot.pivotSetPos(constants.pivotUpPos);
        if ((Math.abs(pivot.getPos() - constants.pivotUpPos) < 5)) {
            slides.slideSetPos(constants.highSpecimenPos);
            currentState = AUTO_STATE.PATH_TO_SPEC_READY;
        }
    }
    public void processMechanismsReady2() {
        slides.slideSetPos(constants.highSpecScorePos);
        if ((Math.abs(slides.getLeftPos() - constants.highSpecScorePos) < 10) && Math.abs(slides.getRightPos() - constants.highSpecScorePos) < 10) {
            intakes.specSetPos(constants.specimenScorePos);
            pivot.pivotSetPos(constants.pivotDownPos);
            slides.slideSetPos(0);
            currentState = AUTO_STATE.PATH_TO_BASKET_READY;
            primerState = AUTO_STATE.THIRD_MECHANISMS_READY;
        }
    }

    public void processPathReady1() {
        intakes.pivotSetPos(constants.intakePivotGrabPos);
        setupPath(stagingToScore, startToSpecHeading);
        currentState = AUTO_STATE.PATH_ACTIVE;
        primerState = AUTO_STATE.SECOND_MECHANISMS_READY;
    }
    public void processPathReady2() {
        intakes.pivotSetPos(constants.intakePivotScorePos);
        setupPath(scoreToBasket, specToBasketHeading);
        currentState = AUTO_STATE.PATH_ACTIVE;
        primerState = AUTO_STATE.FIRST_SPIKE_READY;
    }

    public void processFirstSpike() {
        follower.holdPoint(basketPoint, spikeOneHeading);
        botHeading = spikeOneHeading;
        slides.slideSetPos(constants.firstSpikeSlidePos);
        if ((Math.abs(slides.getLeftPos() - constants.highSpecScorePos) < 150) && Math.abs(slides.getRightPos() - constants.highSpecScorePos) < 150) {
            intakes.pivotSetPos(constants.intakePivotGrabPos);
            intakes.setIntakePower(constants.intakeCollectPow);
            if ((Math.abs(slides.getLeftPos() - constants.highSpecScorePos) < 5) && Math.abs(slides.getRightPos() - constants.highSpecScorePos) < 5) {
                slides.slideSetPos(0);
                currentState = AUTO_STATE.FIRST_BASKET_READY;
                intakes.setIntakePower(0);
            }
        }
    }
    public void processFirstBasket() {
        follower.holdPoint(basketPoint, specToBasketHeading);
        botHeading = specToBasketHeading;
        intakes.pivotSetPos(constants.intakePivotGrabPos);
        pivot.pivotSetPos(constants.pivotUpPos);
        if ((Math.abs(pivot.getPos() - constants.pivotUpPos) < 5)) {
            slides.slideSetPos(constants.highBasketPos);
            intakes.pivotSetPos(constants.intakePivotScorePos);
            if ((Math.abs(slides.getLeftPos() - constants.highSpecScorePos) < 10) && Math.abs(slides.getRightPos() - constants.highSpecScorePos) < 10) {
                intakes.setIntakePower(constants.intakeScorePow);
                intakes.pivotSetPos(constants.intakePivotGrabPos);
                slides.slideSetPos(0);
                pivot.pivotSetPos(constants.pivotDownPos);
                currentState = AUTO_STATE.SECOND_SPIKE_READY;
            }
        }
    }
    public void processSecondSpike() {

    }
    private void processStateMachine() {
        switch(currentState) {
            case START_STATE: processStartState(); break;
            case PATH_ACTIVE: processPathActive(); break;
            case FIRST_MECHANISMS_READY: processMechanismsReady1(); break;
            case PATH_TO_SPEC_READY: processPathReady1(); break;
            case SECOND_MECHANISMS_READY: processMechanismsReady2(); break;
            case PATH_TO_BASKET_READY: processPathReady2(); break;
            case FIRST_SPIKE_READY: processFirstSpike(); break;
            case FIRST_BASKET_READY: processFirstBasket(); break;
            case SECOND_SPIKE_READY: processSecondSpike(); break;
        }
    }

    @Override
    public void loop() {
        processStateMachine();

        follower.update();
    }
}
