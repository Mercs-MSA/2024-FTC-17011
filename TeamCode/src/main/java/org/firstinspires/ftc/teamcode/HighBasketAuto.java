package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.propkey.qual.PropertyKeyBottom;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants;
import org.firstinspires.ftc.teamcode.subSystems.Intakes;
import org.firstinspires.ftc.teamcode.subSystems.Pivot;
import org.firstinspires.ftc.teamcode.subSystems.Slides;

import static org.firstinspires.ftc.teamcode.Constants.extendPosAuto;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.startingPoseLeft;
@Autonomous
public class HighBasketAuto extends OpMode {
    private Telemetry telemetryA;
    private FollowerConstants followerConstants;


    private Constants constants;
    private Follower follower;
    //    private double botHeading = startingPoseLeft.getHeading();
    private Intakes intakes;
    private Pivot pivot;
    private Slides slides;


    private enum AUTO_STATE {
        START_STATE,
        AWAY_PATH_ACTIVE,
        BACK_PATH_ACTIVE,
        DO_NOTHING,
        PATH_TO_BASKET_READY,
        GO_TO_FIRST_SPIKE,
        GO_TO_SECOND_SPIKE,
        GO_TO_THIRD_SPIKE,
        SCORE_BASKET,
        MECHANISMS_RESET,
        PICKUP_SPIKE
    }


    private AUTO_STATE currentState = AUTO_STATE.START_STATE;
    private AUTO_STATE primerState = AUTO_STATE.START_STATE;


    private static ElapsedTime timeoutTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private int timeoutPeriod = 0;
    //------------------------------------------------------------------------------------------------------------------------
    public static final Point startPoint = new Point (startingPoseLeft.getX(), startingPoseLeft.getY(), Point.CARTESIAN); // <-- insert correct points     public static final Point startPoint = new Point (startingPoseLeft.getX(), startingPoseLeft.getY(), Point.CARTESIAN);
//    public static final Pose basketScorePos = pointAndHeadingToPose(-52, -55.4, 49.25);
//    public static final Pose firstSpikePos = pointAndHeadingToPose(-50.6101, -51.53, 91.3263); //This is the rightmost spike, and insert correct points
//    public static final Pose secondSpikePos = pointAndHeadingToPose(-58, -51.53, 93);
//    public static final Pose thirdSpikePos = pointAndHeadingToPose(-59, -49.53, 110.5);




    public static final Pose basketScorePos = pointAndHeadingToPose(-53.0773, -57.212, 46.2469); //y-value = 57.212
    public static final Pose firstSpikePos = pointAndHeadingToPose(-49.3214, -51.336, 91.5); //This is the rightmost spike, and insert correct points
    public static final Pose secondSpikePos = pointAndHeadingToPose(-56.3116, -51.542, 88); //56.2116 for x-value
    public static final Pose thirdSpikePos = pointAndHeadingToPose(-60.566, -49.0326, 98); // -54.761 //119.5






    @Override
    public void init() {
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startingPoseLeft);
        follower.update();
        try {
            initializeSubSystems();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        currentState = AUTO_STATE.START_STATE;
        follower.setMaxPower(.8);
        followerConstants.pathEndTimeoutConstraint = 400; //350?




        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());




        telemetryA.update();
    }


    public void initializeSubSystems() throws InterruptedException {
        intakes = new Intakes(hardwareMap, telemetry);
        pivot = new Pivot(hardwareMap, constants.pivotP, constants.pivotI, constants.pivotD, constants.pivotF);
        slides = new Slides(hardwareMap);
    }


    private void setupPath(Path pathToFollow, double endHeading) {
        double currentHeading = endHeading;
        follower.followPath(pathToFollow);
        pathToFollow.setLinearHeadingInterpolation(follower.getPose().getHeading(), endHeading, .7);
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


    private static Pose pointAndHeadingToPose(double x, double y, double headingInDegrees) {
        return new Pose(x, y, Math.toRadians(headingInDegrees));
    }
    public void makePath(Pose targetPose) {
        Point targetPoint = poseToPoint(targetPose);
        double targetHeading = targetPose.getHeading();
        setupPath(new Path(new BezierCurve(poseToPoint(follower.getPose()), targetPoint)), targetHeading);
    }
    public Point poseToPoint(Pose pose) {
        Point returnPoint = new Point(pose.getX(), pose.getY(), Point.CARTESIAN);
        return returnPoint;
    }

    boolean start = true;

    public void processStartState() {
//        intakes.pivotSetPos(constants.intakePivotGrabPos + 1); - IT NEEDS ALL THE POS VALUE
        intakes.pivotSetPos(constants.intakePivotGrabPos);
//        intakes.pivotSetPow();
        intakes.specSetPos(constants.specimenScorePos);
        makePath(basketScorePos);
        currentState = AUTO_STATE.BACK_PATH_ACTIVE;
        primerState = AUTO_STATE.SCORE_BASKET;
    }


    int counter = 0;
    //    int bonusCount = 23;
    public void processScoreState() {
        start = false;
        intakes.spinSetPos(constants.intakeSpinDefault);
        slideCounter = 7;
//        if (pivot.getLeftPos() < (constants.pivotUpPos - 20)) {
        if (slides.getLeftPos() < (constants.highBasketPos - 200))
            intakes.pivotSetPos(constants.intakePivotGrabPos);
        slides.setPow(1);
//            pivot.setPow(1);
//        }
        pivot.pivotSetPos(constants.pivotUpPos);
        if (pivot.getLeftPos() > (constants.pivotUpPos - 100)) {
            slides.slideSetPos(constants.highBasketPos);
            if (pivot.getLeftPos() > (constants.pivotUpPos - 50))
                pivot.setPow(0);
            if (slides.getLeftPos() > (constants.highBasketPos - 100)) {
                intakes.pivotSetPos(constants.intakePivotScorePos);
                if (counter < 21) {
//                    bonusCount--;
                    if (counter > 5) {
                        intakes.setIntakePower(constants.intakeScorePow);
                    }
                    counter++;
                } else {
                    currentState = AUTO_STATE.MECHANISMS_RESET;
                }
            }
        }
    }


    public void processResetMechanisms() {
        counter = 0;
        counter2 = 0;
//        bonusCount = 22;
        if (pivot.getLeftPos() > 30)
            pivot.setPow(.5);
        intakes.setIntakePower(0);
        intakes.pivotSetPos(constants.intakePivotGrabPos);
//        slides.slideSetPos(0);
//        if (slides.getLeftPos() < 1900) {
//            pivot.pivotSetPos(-60);
//            intakes.spinSetPos(constants.intakeSpinDefault);
//            if (pivot.getPos() < 15) {
//                pivot.setPow(0);
//                pivot.resetPos();
        if (samplesAcquired == 0 && !lastStatePickUp)
            currentState = AUTO_STATE.GO_TO_FIRST_SPIKE;
        else if (samplesAcquired == 1 && !lastStatePickUp)
            currentState = AUTO_STATE.GO_TO_SECOND_SPIKE;
        else if (samplesAcquired == 2 && !lastStatePickUp)
            currentState = AUTO_STATE.GO_TO_THIRD_SPIKE;
        else if (lastStatePickUp) {
            lastStatePickUp = false;
            currentState = AUTO_STATE.PATH_TO_BASKET_READY;
        }
//            }
//        }
    }


    public void processFirstSpike() {
        followerConstants.pathEndTimeoutConstraint = 300;
        makePath(firstSpikePos);
        currentState = AUTO_STATE.AWAY_PATH_ACTIVE;
        primerState = AUTO_STATE.PICKUP_SPIKE;
    }


    public void processSecondSpike() {
        followerConstants.pathEndTimeoutConstraint = 300;
        makePath(secondSpikePos);
        currentState = AUTO_STATE.AWAY_PATH_ACTIVE;
        primerState = AUTO_STATE.PICKUP_SPIKE;
    }


    public void processThirdSpike() {
        followerConstants.pathEndTimeoutConstraint = 400;
        makePath(thirdSpikePos);
        currentState = AUTO_STATE.AWAY_PATH_ACTIVE;
        primerState = AUTO_STATE.PICKUP_SPIKE;
    }


    int counter2 = 0;
    int samplesAcquired = 0;
    boolean lastStatePickUp = false;
    public void processSpikePickUp() {
        slideCounter = 7;
        if ((counter2 > 20) && samplesAcquired == 2)
            intakes.spinSetPos(constants.intakeSpinRight - .17);
        if (pivot.getLeftPow() != 0) {
            if (pivot.getLeftPos() > 200)
                intakes.pivotSetPos(constants.intakePivotScorePos);
            else
                intakes.pivotSetPos(constants.intakePivotGrabPos);
            pivot.pivotSetPos(0);
            intakes.spinSetPos(constants.intakeSpinDefault);
            if (pivot.getLeftPos() < 15) {
                pivot.setPow(0);
                pivot.resetPos();
            }
        } else if (pivot.getLeftPow() == 0) {
            lastStatePickUp = true;
            intakes.setIntakePower(constants.intakeCollectPow);
            intakes.pivotSetPos(constants.intakePivotGrabPos);
            if (Math.abs(slides.getPow() - 1) < .5)
                slides.setPow(.9);
            if ((counter2 < 46) && (samplesAcquired == 2)) {
                slides.slideSetPos(extendPosAuto);
                counter2++;
            } else if ((counter2 < 38) && (samplesAcquired < 2)) {
                slides.slideSetPos(extendPosAuto);
                counter2++;
            } else {
                intakes.pivotSetPos(constants.intakePivotMidPos);
                samplesAcquired++;
                currentState = AUTO_STATE.MECHANISMS_RESET;
            }
        }
    }


    public void processGoToBasket() {
        followerConstants.pathEndTimeoutConstraint = 350;
        makePath(new Pose(basketScorePos.getX() + 1, basketScorePos.getY() + 3, basketScorePos.getHeading()));
        currentState = AUTO_STATE.BACK_PATH_ACTIVE;
        primerState = AUTO_STATE.SCORE_BASKET;
    }


    int slideCounter = 7;
    public void processAwayPathActive() {
//        intakes.pivotSetPos(constants.intakePivotScorePos);
        slideCounter--;
        if (slideCounter <= 0) {
            slides.slideSetPos(0);
            if (pivot.getLeftPos() > 200)
                intakes.pivotSetPos(constants.intakePivotScorePos);
            else
                intakes.pivotSetPos(constants.intakePivotGrabPos);
            pivot.pivotSetPos(0);
        }
        if (!pathIsBusy()) {
            currentState = primerState;
        }
    }


    public void processBackPathActive() {
//        intakes.pivotSetPos(constants.intakePivotScorePos);
//        slideCounter--;
//        if (slideCounter <= 0)
//            slides.slideSetPos(0);
        if (!start)
            pivot.pivotSetPos(constants.pivotUpPos);
        intakes.pivotSetPos(constants.intakePivotGrabPos);
        slides.slideSetPos(0);
        pivot.setPow(1);
        if (!pathIsBusy()) {
            currentState = primerState;
        }
    }


    public void processDoNothing() {}


    private void processStateMachine() {
        switch(currentState) {
            case START_STATE: processStartState(); break;
            case AWAY_PATH_ACTIVE: processAwayPathActive(); break;
            case BACK_PATH_ACTIVE: processBackPathActive(); break;
            case SCORE_BASKET: processScoreState(); break;
            case MECHANISMS_RESET: processResetMechanisms(); break;
            case PICKUP_SPIKE: processSpikePickUp(); break;
            case PATH_TO_BASKET_READY: processGoToBasket(); break;
            case GO_TO_FIRST_SPIKE: processFirstSpike(); break;
            case GO_TO_SECOND_SPIKE: processSecondSpike(); break;
            case GO_TO_THIRD_SPIKE: processThirdSpike(); break;
            case DO_NOTHING: processDoNothing(); break;
        }
    }


    @Override
    public void loop() {
        processStateMachine();


        follower.update();
        telemetry.addData("intake pivot pos: ", intakes.pivotGetPos());
        telemetry.addData("counter: ", counter);
        telemetry.addData("samples acquired: ", samplesAcquired);
        telemetry.addData("pivot pos: ", pivot.getLeftPos());
        telemetry.addData("current state: ", currentState);
        telemetry.addData("left pos: ", Math.abs(slides.getLeftPos()));
        follower.telemetryDebug(telemetryA);
    }
}




