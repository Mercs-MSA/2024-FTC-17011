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
        TEST,
        START_STATE,
        PATH_ACTIVE,
        DO_NOTHING,
        PATH_TO_BASKET_READY,
        FIRST_SPIKE_READY,
        FIRST_BASKET_READY,
        SECOND_SPIKE_READY,
        SECOND_SPIKE_PICKUP,
        SECOND_BASKET_READY,
        THIRD_SPIKE_READY,
        THIRD_SPIKE_PICKUP,
        SCORE_BASKET,
        FIRST_SPIKE_PICKUP,
        FIRST_SPIKE_FINISH,
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
    //------------------------------------------------------------------------------------------------------------------------
//    public static final double startToSpecHeading = Math.toRadians(270);
//    public static final double specToBasketHeading = Math.toRadians(45);
    public static final double spikeOneHeading = Math.toRadians(0);
    public static final double spikeTwoHeading = Math.toRadians(0);
//    public static final Pose defaultPose = new Pose(-10,-10,0);
//    public static final Point defaultPoint = new Point(-10, -10, Point.CARTESIAN);
//    public static final Point testPoint = new Point(0, -10, Point.CARTESIAN);
//    public static final Path testPath = new Path(new BezierCurve(defaultPoint, testPoint));




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
                    counter = 0;
                    if (samplesAcquired == 0)
                        currentState = AUTO_STATE.FIRST_SPIKE_READY;
                    if (samplesAcquired == 1)
                        currentState = AUTO_STATE.SECOND_SPIKE_READY;
                }
            }
        }
    }

    public void processFirstSpike() {
        makePath(firstSpikePos);
        currentState = AUTO_STATE.PATH_ACTIVE;
        primerState = AUTO_STATE.FIRST_SPIKE_PICKUP;
    }

    public void processSecondSpike() {
        makePath(secondSpikePos);
        currentState = AUTO_STATE.PATH_ACTIVE;
        primerState = AUTO_STATE.SECOND_SPIKE_READY;
    }

    public void processThirdSpike() {
        makePath(thirdSpikePos);
        currentState = AUTO_STATE.PATH_ACTIVE;
        primerState = AUTO_STATE.THIRD_SPIKE_PICKUP;
    }

    int i = 0;
    public void processFirstSpikePickUp() {
        intakes.setIntakePower(0);
        intakes.pivotSetPos(constants.intakePivotMidPos);
        if (i == 0)
            slides.slideSetPos(0);
        if (slides.getLeftPos() < 200) {
            if (i == 0) {
                pivot.pivotSetPos(-60);
                i++;
            }
            if (pivot.getPos() < 0) {
                pivot.resetPos();
                slides.slideSetPos(800);
                currentState = AUTO_STATE.FIRST_SPIKE_FINISH;
                // add next state
            }
        }
    }

    int counter2 = 0;
    int samplesAcquired = 0;
    public void processFirstSpikeFinish() {
        intakes.pivotSetPos(constants.intakePivotGrabPos);
        if (counter2 < 50) {
            slides.slideSetPos(extendPosAuto);
            intakes.setIntakePower(constants.intakeCollectPow);
            counter2++;
        } else {
            samplesAcquired++;
            slides.slideSetPos(0);
            currentState = AUTO_STATE.PATH_TO_BASKET_READY;
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
            case PATH_TO_BASKET_READY: processGoToBasket(); break;
            case FIRST_SPIKE_READY: processFirstSpike(); break;
            case FIRST_SPIKE_PICKUP: processFirstSpikePickUp(); break;
            case FIRST_SPIKE_FINISH: processFirstSpikeFinish(); break;
            case SECOND_SPIKE_READY: processSecondSpike(); break;

            case THIRD_SPIKE_READY: processThirdSpike(); break;
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


