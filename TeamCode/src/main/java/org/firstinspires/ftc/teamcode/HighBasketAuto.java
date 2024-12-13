package org.firstinspires.ftc.teamcode;


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
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.subSystems.Intakes;
import org.firstinspires.ftc.teamcode.subSystems.Pivot;
import org.firstinspires.ftc.teamcode.subSystems.Slides;


import static org.firstinspires.ftc.teamcode.Constants.extendPosAuto;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.startingPoseLeft;
@Autonomous
public class HighBasketAuto extends OpMode {
    private Telemetry telemetryA;

    private Constants constants;
    private Follower follower;
    //    private double botHeading = startingPoseLeft.getHeading();
    private Intakes intakes;
    private Pivot pivot;
    private Slides slides;

    private enum AUTO_STATE {
        START_STATE,
        PATH_ACTIVE,
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
    public static final Pose basketScorePos = pointAndHeadingToPose(-56.49, -55.757, 45.25);
    public static final Pose firstSpikePos = pointAndHeadingToPose(-50.6101, -52.03, 90.3263); //This is the rightmost spike, and insert correct points
    public static final Pose secondSpikePos = pointAndHeadingToPose(0, -53.03, 90);
    public static final Pose thirdSpikePos = pointAndHeadingToPose(0, -53.03, 90);



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
        follower.setMaxPower(.6);


        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());


        telemetryA.update();
    }

    public void initializeSubSystems() throws InterruptedException {
        intakes = new Intakes(hardwareMap);
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
    public void processStartState() {
        makePath(basketScorePos);
        currentState = AUTO_STATE.PATH_ACTIVE;
        primerState = AUTO_STATE.SCORE_BASKET;
    }

    int counter = 0;
    public void processScoreState() {
        pivot.pivotSetPos(constants.pivotUpPos);
        if (pivot.getPos() > (constants.pivotUpPos - 250)) {
            slides.slideSetPos(constants.highBasketPos);
            if (slides.getLeftPos() > (constants.highBasketPos - 50)) {
                intakes.pivotSetPos(constants.intakePivotScorePos);
                if (counter < 30) {
                    counter++;
                } else {
                    intakes.setIntakePower(constants.intakeScorePow);
                    currentState = AUTO_STATE.MECHANISMS_RESET;
                }
            }
        }
    }

    public void processResetMechanisms() {
        counter = 0;
        counter2 = 0;
        intakes.setIntakePower(0);
        intakes.pivotSetPos(constants.intakePivotMidPos);
        slides.slideSetPos(0);
        if (slides.getLeftPos() < 200) {
            pivot.pivotSetPos(-60);
            if (pivot.getPos() < 0) {
                pivot.resetPos();
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
            }
        }
    }

    public void processFirstSpike() {
        makePath(firstSpikePos);
        currentState = AUTO_STATE.PATH_ACTIVE;
        primerState = AUTO_STATE.PICKUP_SPIKE;
    }

    public void processSecondSpike() {
        makePath(secondSpikePos);
        currentState = AUTO_STATE.PATH_ACTIVE;
        primerState = AUTO_STATE.PICKUP_SPIKE;
    }

    public void processThirdSpike() {
        makePath(thirdSpikePos);
        currentState = AUTO_STATE.PATH_ACTIVE;
        primerState = AUTO_STATE.PICKUP_SPIKE;
    }

    int counter2 = 0;
    int samplesAcquired = 0;
    boolean lastStatePickUp = false;
    public void processSpikePickUp() {
        lastStatePickUp = true;
        intakes.setIntakePower(constants.intakeCollectPow);
        intakes.pivotSetPos(constants.intakePivotGrabPos);
        if (Math.abs(slides.getPow() - 1) < .5)
            slides.setPow(.5);
        if (counter2 < 50) {
            slides.slideSetPos(extendPosAuto);
            counter2++;
        } else {
            samplesAcquired++;
            currentState = AUTO_STATE.MECHANISMS_RESET;
        }
    }

    public void processGoToBasket() {
        makePath(basketScorePos);
        currentState = AUTO_STATE.PATH_ACTIVE;
        primerState = AUTO_STATE.SCORE_BASKET;
    }

    public void processPathActive() {
        if (!pathIsBusy()) {
            currentState = primerState;
        }
    }

    public void processDoNothing() {}
    private void processStateMachine() {
        switch(currentState) {
            case START_STATE: processStartState(); break;
            case PATH_ACTIVE: processPathActive(); break;
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
        telemetry.addData("counter: ", counter);
        telemetry.addData("samples acquired: ", samplesAcquired);
        telemetry.addData("pivot pos: ", pivot.getPos());
        telemetry.addData("current state: ", currentState);
        telemetry.addData("left pos: ", Math.abs(slides.getLeftPos()));
        follower.telemetryDebug(telemetryA);
    }
}


