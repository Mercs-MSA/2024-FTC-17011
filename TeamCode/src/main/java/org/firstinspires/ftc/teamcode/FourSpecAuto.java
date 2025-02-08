package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.startingPoseRight;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.subSystems.Intakes;
import org.firstinspires.ftc.teamcode.subSystems.Pivot;
import org.firstinspires.ftc.teamcode.subSystems.Slides;

@Autonomous
public class FourSpecAuto extends OpMode {
    private Telemetry telemetryA;

    private Constants constants;
    private Follower follower;
    //    private double botHeading = startingPoseLeft.getHeading();
    private Intakes intakes;
    private Pivot pivot;
    private Slides slides;

    //    private enum AUTO_DRIVE_STATE {
//        START_STATE,
//        STAGING_TO_SCORE_STATE,
//        PUSH_ALL_STATE,
//        GO_TO_STAGING_STATE,
//        GO_TO_INTAKE_STATE,
//        PARK_STATE,
//        DO_NOTHING_STATE,
//    }
//
//    private AUTO_DRIVE_STATE currentDriveState = AUTO_DRIVE_STATE.START_STATE;
    private enum AUTO_STATE {
        START_STATE,
        PATH_ACTIVE,
        COLLECT_STATE,
        TURN_TO_DROP_OFF,
        DROP_OFF_STATE,
        SPIKE_TWO_STATE,
        PIVOT_UP,
        INTAKE_STATE,
        SPEC_SCORE_STATE,
        BACK_TO_INTAKE,
        PARK_STATE,
        END_STATE,
        DO_NOTHING_STATE
    }

    private AUTO_STATE currentState = AUTO_STATE.START_STATE;
    private AUTO_STATE primerState = AUTO_STATE.START_STATE;

    private static ElapsedTime timeoutTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private int timeoutPeriod = 0;
    //------------------------------------------------------------------------------------------------------------------------
    public static final Pose spikeOnePose = pointAndHeadingToPose(-11.25, 43.05, 90);
    public static final Pose spikeOneTurnPose = pointAndHeadingToPose(-11.25, 43.05, 90);
    public static final Pose spikeTwoPose = pointAndHeadingToPose(-9.75, 36.25, 90);
    public static final Pose spikeTwoTurnPose = pointAndHeadingToPose(-11.25, 43.05, 90);

    public static final Pose toIntake = pointAndHeadingToPose(-35.33, 42, 270);
    public static final Pose finalIntakePoint = pointAndHeadingToPose(-33.5,62.7,270);

    public static final Pose specScoreFirstPose = pointAndHeadingToPose(-3.25, 37.25, 89);
    public static final Pose specScoreSecondPose = pointAndHeadingToPose(-6.25, 37.25, 89);
    public static final Pose specScoreThirdPose = pointAndHeadingToPose(-9.25, 37.25, 89);
    public static final Pose specScoreFourthPose = pointAndHeadingToPose(-12.25, 37.55, 89);

    public static final Pose endPoint = pointAndHeadingToPose(-59,68,270);
    //------------------------------------------------------------------------------------------------------------------------
    @Override
    public void init() {
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startingPoseRight);
        follower.update();
        try {
            initializeSubSystems();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        follower.setMaxPower(.8);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetryA.update();
    }

    public static Pose pointAndHeadingToPose(double x, double y, double headingInDegrees) {
        return new Pose(x, y, Math.toRadians(headingInDegrees));
    }

    public void initializeSubSystems() throws InterruptedException {
        intakes = new Intakes(hardwareMap);
        pivot = new Pivot(hardwareMap, constants.pivotP, constants.pivotI, constants.pivotD, constants.pivotF);
        slides = new Slides(hardwareMap);
    }

    private void setupPath(Path pathToFollow, double endHeading) {
        follower.followPath(pathToFollow, true);
        pathToFollow.setLinearHeadingInterpolation(follower.getPose().getHeading(), endHeading, .7);
    }

    private void setupPathChain(PathChain pathChainToFollow, double endHeading) {
        follower.followPath(pathChainToFollow, true);
    }

    private Point poseToPoint(Pose pose) {
        Point returnPoint = new Point(pose.getX(), pose.getY(), Point.CARTESIAN);
        return returnPoint;
    }

    private void makePath(Pose targetPose) {
        Point targetPoint = poseToPoint(targetPose);
        double targetHeading = targetPose.getHeading();
        setupPath(new Path(new BezierCurve(poseToPoint(follower.getPose()), targetPoint)), targetHeading);
    }

    int busyCount = 0;
    private boolean pathIsBusy() {
        busyCount++;
        if (busyCount > 10) {
            if (robotStalled) {
                busyCount = 0;
                return false;
            } else {
                if (!follower.isBusy())
                    busyCount = 0;
                return follower.isBusy();
            }
        } else {
            return true;
        }
    }
    private double[] deltaTracking = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
    private int stallIndex = 0;
    private double deltaTotal = 10;
    private double lastX = 0;
    private double lastHeading = 0;
    private double lastY = 0;
    private double minDelta = 1000;

    private boolean robotStalled = false;
    private void checkIfStalled()
    {
        double distanceMoved = 0;
        if (follower.isBusy()) {
            double currentX = follower.getPose().getX();
            double distanceMovedX = currentX - lastX;
            lastX = currentX;
            double currentY = follower.getPose().getY();
            double distanceMovedY = currentY - lastY;
            lastY = currentY;
            double currentHeading = follower.getPose().getHeading();
            double distanceMovedHeading = currentHeading - lastHeading;
            lastHeading = currentHeading;
            distanceMoved = Math.abs(distanceMovedX) + Math.abs(distanceMovedY) + Math.abs(distanceMovedHeading * 5);
            deltaTotal = deltaTotal - deltaTracking[stallIndex] + distanceMoved;
            if (deltaTotal < minDelta)
                minDelta = deltaTotal;
            deltaTracking[stallIndex] = distanceMoved;
            if (stallIndex < 9)
                stallIndex = stallIndex + 1;
            else
                stallIndex = 0;

            if (deltaTotal < .01)
                robotStalled = true;
            else
                robotStalled = false;

        }
        telemetryA.addData("Distance Moved", distanceMoved);
        telemetryA.addData("deltaTotal", deltaTotal);
        /*telemetryA.addData("minDelta", minDelta);
        //robotStalled = false;*/
        telemetryA.addData("robotStalled", robotStalled);
    }




    private void processStartState() {
        makePath(spikeOnePose);
        intakes.pivotSetPos(Constants.intakePivotMidPos);
        intakes.specSetPos(Constants.specimenScorePos);
        currentState = AUTO_STATE.PATH_ACTIVE;
        primerState = AUTO_STATE.DO_NOTHING_STATE;
    }

    private void processPathActive() {
        if (!pathIsBusy())
            currentState = primerState;
    }

    int collected = 0;
    int collectCounter = 0;
    private void processCollect() {
        intakes.pivotSetPos(constants.intakePivotGrabPos);
        intakes.setIntakePower(constants.intakeCollectPow);
        if (collected == 0 && collectCounter < 45) {
            slides.slideSetPos(constants.extendPos);
            collectCounter++;
        } else if (collected == 1 && collectCounter < 12) {
            slides.slideSetPos(constants.extendPos);
            collectCounter++;
        } else {
            collected++;
            currentState = AUTO_STATE.TURN_TO_DROP_OFF;
        }
    }

    private void processTurnToDropOff() {
        collectCounter = 0;
        if (collected == 0) {
            follower.holdPoint(spikeOneTurnPose);
        } else {
            follower.holdPoint(spikeTwoTurnPose);
        }
        currentState = AUTO_STATE.PATH_ACTIVE;
        primerState = AUTO_STATE.DROP_OFF_STATE;
    }

    int dropCounter = 0;
    private void processDropOff() {
        if (dropCounter < 5) {
            intakes.setIntakePower(constants.intakeScorePow);
        } else if (collected == 1) {
            currentState = AUTO_STATE.SPIKE_TWO_STATE;
        } else {
            currentState = AUTO_STATE.PIVOT_UP;
        }
    }

    private void processSpikeTwo() {
        slides.slideSetPos(constants.extendPos - 150);
        makePath(spikeTwoPose);
        currentState = AUTO_STATE.PATH_ACTIVE;
        primerState = AUTO_STATE.COLLECT_STATE;
    }

    private void processPivotUp() {
        slides.slideSetPos(0);
        pivot.pivotSetPos(constants.pivotUpPos);
        currentState = AUTO_STATE.BACK_TO_INTAKE;
    }

    private void processBackToIntake() {
        follower.setMaxPower(.6);
        if (specCount == 1)
            follower.setMaxPower(.5);
        Path segmentEight = new Path(new BezierCurve(poseToPoint(follower.getPose()), poseToPoint(toIntake)));
        segmentEight.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(270), 1);
        Path segmentNine = new Path(new BezierCurve(poseToPoint(toIntake), poseToPoint(finalIntakePoint)));
        segmentNine.setLinearHeadingInterpolation(Math.toRadians(268), Math.toRadians(268), 1);
        PathChain secondCurve = new PathChain(segmentEight, segmentNine);
        follower.followPath(secondCurve, true);
        currentState = AUTO_STATE.PATH_ACTIVE;
        primerState = AUTO_STATE.INTAKE_STATE;
    }

    int specCount = 0;
    int counter = 0;
    int counter2 = 0;
    private void processIntake() {
        if (counter2 == 0) {
            counter = 0;
            counter2++;
        }
        if (counter < 30) {
            follower.setMaxPower(.8);
            intakes.specSetPos(constants.specimenHoldPos);
            counter++;
        }
        if (counter >= 29) {
            specCount++;
            counter2 = 0;
            slides.slideSetPos(constants.highSpecimenPos);
            if (specCount == 1) {
                makePath(specScoreFirstPose);
                currentState = AUTO_STATE.PATH_ACTIVE;
                primerState = AUTO_STATE.SPEC_SCORE_STATE;
            } else if (specCount == 2 ) {
                makePath(specScoreSecondPose);
                currentState = AUTO_STATE.PATH_ACTIVE;
                primerState = AUTO_STATE.SPEC_SCORE_STATE;
            } else if (specCount == 3) {
                makePath(specScoreThirdPose);
                currentState = AUTO_STATE.PATH_ACTIVE;
                primerState = AUTO_STATE.SPEC_SCORE_STATE;
            } else if (specCount == 4) {
                makePath(specScoreFourthPose);
                currentState = AUTO_STATE.PATH_ACTIVE;
                primerState = AUTO_STATE.SPEC_SCORE_STATE;
            }
        }
    }

    private void processSpecScore() {
        if (intakes.specCheckPos() != constants.specimenScorePos)
            slides.slideSetPos(constants.highSpecScorePos);
        if (pivot.getLeftPow() == 1)
            pivot.setPow(0);
        if (Math.abs(slides.getLeftPos() - constants.highSpecScorePos) < 15) {
            intakes.specSetPos(constants.specimenScorePos);
            slides.slideSetPos(0);
            if (specCount == 1) {
                currentState = AUTO_STATE.BACK_TO_INTAKE;
            } else if (specCount == 2) {
                currentState = AUTO_STATE.BACK_TO_INTAKE;
            } else if (specCount == 3) {
                currentState = AUTO_STATE.BACK_TO_INTAKE;
            } else if (specCount == 4) {
                currentState = AUTO_STATE.PATH_ACTIVE;
            }
        }
    }

    private void processPark() {
        makePath(endPoint);
        currentState = AUTO_STATE.PATH_ACTIVE;
        primerState = AUTO_STATE.END_STATE;
    }

    int bruh = 0;

    private void processDoNothing() {}

    private void processStateMachine() {
        switch(currentState) {
            case START_STATE: processStartState(); break;
            case PATH_ACTIVE: processPathActive(); break;
            case COLLECT_STATE: processCollect(); break;
            case TURN_TO_DROP_OFF: processTurnToDropOff(); break;
            case DROP_OFF_STATE: processDropOff(); break;
            case SPIKE_TWO_STATE: processSpikeTwo(); break;
            case PIVOT_UP: processPivotUp(); break;
            case INTAKE_STATE: processIntake(); break;
            case SPEC_SCORE_STATE: processSpecScore(); break;
            case BACK_TO_INTAKE: processBackToIntake(); break;
            case PARK_STATE: processPark(); break;
            case DO_NOTHING_STATE: processDoNothing(); break;
        }
    }

    @Override
    public void loop() {
        follower.update();
        checkIfStalled();

        processStateMachine();
        if (currentState == AUTO_STATE.PARK_STATE) {
            intakes.pivotSetPos(constants.intakePivotMidPos);
            if (bruh == 0) {
                pivot.normalMode();
                bruh++;
            }
            pivot.setPow(-1);
        }

        telemetry.addData("follower busy?: ", follower.isBusy());
        telemetry.addData("pivot pos: ", pivot.getPos());
        telemetry.addData("current state: ", currentState);
        telemetry.addData("left pos: ", Math.abs(slides.getLeftPos()));
        follower.telemetryDebug(telemetryA);
    }
}
