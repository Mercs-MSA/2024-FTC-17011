package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.startingPoseLeft;
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
public class PedroSpecimenAuto extends OpMode {
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
        STAGING_TO_SCORE_STATE,
        SPEC_SCORE_STATE,
        WHILE_PUSHING_STATE,
        INTAKE_STATE,
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
    public static final Pose specStagingPose = pointAndHeadingToPose(-11.25, 43.05, 90);
    public static final Pose specScorePose = pointAndHeadingToPose(-9.75, 36.25, 90);

    public static final Pose firstStaging = pointAndHeadingToPose(-11.25, 40, 90);
    public static final Pose secondStaging = pointAndHeadingToPose(-35.14,42.23,90);
    public static final Pose thirdStaging = pointAndHeadingToPose(-37,16.58,90);
    public static final Pose firstPush = pointAndHeadingToPose(-53,16.58,90);
    public static final Pose toObs = pointAndHeadingToPose(-50.5,53,90);
    public static final Pose secondPush = pointAndHeadingToPose(-65,17.33,90);
    public static final Pose backToObs = pointAndHeadingToPose(-64,59.52,90);
    public static final Pose toIntake = pointAndHeadingToPose(-35.33, 42, 270);
    public static final Pose finalIntakePoint = pointAndHeadingToPose(-33.5,63.25,270);
    
    public static final Pose specScoreSecondPose = pointAndHeadingToPose(-3.25, 37.75, 89);
    public static final Pose specScoreThirdPose = pointAndHeadingToPose(-8.25, 37.55, 89);

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

    private void setPathToPushSamples() {

//        PathChain pushPath = new PathChain(segmentOne, segmentTwo, segmentThree, segmentFour, segmentFive, segmentSix, segmentSeven, segmentEight, segmentNine);
//        follower.followPath(pushPath, true);
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
        makePath(specStagingPose);
        intakes.pivotSetPos(Constants.intakePivotMidPos);
        intakes.specSetPos(Constants.specimenHoldPos);
        currentState = AUTO_STATE.PATH_ACTIVE;
        primerState = AUTO_STATE.STAGING_TO_SCORE_STATE;
    }

    private void processPathActive() {
        if (!pathIsBusy())
            currentState = primerState;
    }

    private void processStaging() {
        pivot.pivotSetPos(constants.pivotUpPos);
        if (Math.abs(pivot.getPos() - Constants.pivotUpPos) < 65) {
            slides.slideSetPos(Constants.highSpecimenPos);
            if (Math.abs(slides.getLeftPos() - Constants.highSpecimenPos) < 50) {
                makePath(specScorePose);
                pivot.setPow(0);
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
            if (specCount == 0) {
                currentState = AUTO_STATE.WHILE_PUSHING_STATE;
            } else if (specCount == 1) {
                currentState = AUTO_STATE.BACK_TO_INTAKE;
            } else if (specCount >= 2) {
                currentState = AUTO_STATE.PARK_STATE;
            }
        }
    }
    int pushingCount = 0;
    private void processPushingSamples() {
        if (pushingCount == 0) {
//            Path segmentOne = new Path(new BezierCurve(poseToPoint(follower.getPose()), poseToPoint(firstStaging)));
//            segmentOne.setLinearHeadingInterpolation(follower.getPose().getHeading(), Math.toRadians(90), 1);
//            Path segmentTwo = new Path(new BezierCurve(poseToPoint(firstStaging), poseToPoint(secondStaging)));
//            segmentTwo.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90), 1);
//            PathChain firstCurve = new PathChain(segmentOne, segmentTwo);
//            follower.followPath(firstCurve, true);
            makePath(firstStaging);
            pushingCount++;
            currentState = AUTO_STATE.PATH_ACTIVE;
            primerState = AUTO_STATE.WHILE_PUSHING_STATE;
        } else if (pushingCount == 1) {
            makePath(secondStaging);
            pushingCount++;
            currentState = AUTO_STATE.PATH_ACTIVE;
            primerState = AUTO_STATE.WHILE_PUSHING_STATE;
        } else if (pushingCount == 2) {
            makePath(thirdStaging);
            pushingCount++;
            currentState = AUTO_STATE.PATH_ACTIVE;
            primerState = AUTO_STATE.WHILE_PUSHING_STATE;
        } else if (pushingCount == 3) {
            makePath(firstPush);
            pushingCount++;
            currentState = AUTO_STATE.PATH_ACTIVE;
            primerState = AUTO_STATE.WHILE_PUSHING_STATE;
        } else if (pushingCount == 4) {
            makePath(toObs);
            pushingCount++;
            currentState = AUTO_STATE.PATH_ACTIVE;
            primerState = AUTO_STATE.WHILE_PUSHING_STATE;
//        } else if (pushingCount == 4) {
//            makePath(secondPush);
//            pushingCount++;
//            currentState = AUTO_STATE.PATH_ACTIVE;
//            primerState = AUTO_STATE.WHILE_PUSHING_STATE;
//        } else if (pushingCount == 5) {
//            makePath(backToObs);
//            pushingCount++;
//            currentState = AUTO_STATE.PATH_ACTIVE;
//            primerState = AUTO_STATE.WHILE_PUSHING_STATE;
        } else if (pushingCount == 5) {
            Path segmentEight = new Path(new BezierCurve(poseToPoint(toObs), poseToPoint(toIntake)));
            segmentEight.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(270), 1);
            Path segmentNine = new Path(new BezierCurve(poseToPoint(toIntake), poseToPoint(finalIntakePoint)));
            segmentNine.setLinearHeadingInterpolation(Math.toRadians(268), Math.toRadians(268), 1);
            PathChain secondCurve = new PathChain(segmentEight, segmentNine);
            follower.followPath(secondCurve, true);
            currentState = AUTO_STATE.PATH_ACTIVE;
            primerState = AUTO_STATE.INTAKE_STATE;
        }
    }
    
    int specCount = 0;
    int counter = 0;
    int counter2 = 0;
    private void processIntake() {
        if (counter2 == 0) {
            counter = 0;
            counter2++;
        }
        if (counter < 40) {
            follower.setMaxPower(.8);
            intakes.specSetPos(constants.specimenHoldPos);
            counter++;
        }
        if (counter >= 39) {
            specCount++;
            counter2 = 0;
            slides.slideSetPos(constants.highSpecimenPos);
            if (specCount == 1) {
                makePath(specScoreSecondPose);
                currentState = AUTO_STATE.PATH_ACTIVE;
                primerState = AUTO_STATE.SPEC_SCORE_STATE;
            } else if (specCount > 2 ) {
                makePath(specScoreThirdPose);
                currentState = AUTO_STATE.PATH_ACTIVE;
                primerState = AUTO_STATE.SPEC_SCORE_STATE;
            }
        }
    }
    
    public void processBackToIntake() {
        follower.setMaxPower(.6);
        if (specCount == 1)
            follower.setMaxPower(.5);
        Path segmentEight = new Path(new BezierCurve(poseToPoint(follower.getPose()), poseToPoint(toIntake)));
        segmentEight.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(268), 1);
        Path segmentNine = new Path(new BezierCurve(poseToPoint(toIntake), poseToPoint(finalIntakePoint)));
        segmentNine.setLinearHeadingInterpolation(Math.toRadians(268), Math.toRadians(270), 1);
        PathChain secondCurve = new PathChain(segmentEight, segmentNine);
        follower.followPath(secondCurve, true);
        currentState = AUTO_STATE.PATH_ACTIVE;
        primerState = AUTO_STATE.INTAKE_STATE;
    }

    public void processParkAndFinish() {
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
            case STAGING_TO_SCORE_STATE: processStaging(); break;
            case SPEC_SCORE_STATE: processSpecScore(); break;
            case WHILE_PUSHING_STATE: processPushingSamples(); break;
            case INTAKE_STATE: processIntake(); break;
            case BACK_TO_INTAKE: processBackToIntake(); break;
            case PARK_STATE: processParkAndFinish(); break;
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

        telemetry.addData("pushing integer: ", pushingCount);
        telemetry.addData("follower busy?: ", follower.isBusy());
        telemetry.addData("pivot pos: ", pivot.getPos());
        telemetry.addData("current state: ", currentState);
        telemetry.addData("left pos: ", Math.abs(slides.getLeftPos()));
        follower.telemetryDebug(telemetryA);
    }
}
