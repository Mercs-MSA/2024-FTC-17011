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
//        END_STATE,
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
        BACK_TO_STAGING,
        END_STATE,
        DO_NOTHING_STATE
    }

    private AUTO_STATE currentState = AUTO_STATE.START_STATE;
    private AUTO_STATE primerState = AUTO_STATE.START_STATE;

    private static ElapsedTime timeoutTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private int timeoutPeriod = 0;
    //------------------------------------------------------------------------------------------------------------------------
    public static final Pose specStagingPose = pointAndHeadingToPose(-11.25, 43.05, 90);
    public static final Pose specScorePose = pointAndHeadingToPose(-11.25, 35.95, 90);
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
        follower.setMaxPower(.3);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetryA.update();
    }

    private static Pose pointAndHeadingToPose(double x, double y, double headingInDegrees) {
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

    private Point poseToPoint(Pose pose) {
        Point returnPoint = new Point(pose.getX(), pose.getY(), Point.CARTESIAN);
        return returnPoint;
    }

    private void makePath(Pose targetPose) {
        Point targetPoint = poseToPoint(targetPose);
        double targetHeading = targetPose.getHeading();
        setupPath(new Path(new BezierCurve(poseToPoint(follower.getPose()), targetPoint)), targetHeading);
    }

    private boolean pathIsBusy() {
        return follower.isBusy();
    }




    private void processStartState() {
        makePath(specStagingPose);
        intakes.pivotSetPos(Constants.intakePivotMidPos);
        intakes.specSetPos(Constants.specimenHoldPos);
        currentState = AUTO_STATE.DO_NOTHING_STATE;
        primerState = AUTO_STATE.STAGING_TO_SCORE_STATE;
    }

    private void processPathActive() {
        if (!pathIsBusy())
            currentState = primerState;
    }

    private void processStaging() {
        pivot.pivotSetPos(constants.pivotUpPos);
        if (Math.abs(pivot.getPos() - Constants.pivotUpPos) < 325) {
            slides.slideSetPos(Constants.highSpecimenPos);
            if (Math.abs(slides.getLeftPos() - Constants.highSpecimenPos) < 50) {
                makePath(specScorePose);
                currentState = AUTO_STATE.PATH_ACTIVE;
                primerState = AUTO_STATE.SPEC_SCORE_STATE;
            }
        }
    }
    private void processSpecScore() {
        if (intakes.specCheckPos() != constants.specimenScorePos)
            slides.slideSetPos(constants.highSpecScorePos);
        if (Math.abs(slides.getLeftPos() - constants.highSpecScorePos) < 15) {
            intakes.specSetPos(constants.specimenScorePos);
            slides.slideSetPos(0);
            currentState = AUTO_STATE.WHILE_PUSHING_STATE;
        }
    }

    private void processPushingSamples() {

    }
    private void processDoNothing() {}

    private void processStateMachine() {
        switch(currentState) {
            case START_STATE: processStartState(); break;
            case PATH_ACTIVE: processPathActive(); break;
            case STAGING_TO_SCORE_STATE: processStaging(); break;
            case SPEC_SCORE_STATE: processSpecScore(); break;
            case DO_NOTHING_STATE: processDoNothing(); break;
        }
    }

    @Override
    public void loop() {
        follower.update();

        processStateMachine();

        telemetry.addData("pivot pos: ", pivot.getPos());
        telemetry.addData("current state: ", currentState);
        telemetry.addData("left pos: ", Math.abs(slides.getLeftPos()));
        follower.telemetryDebug(telemetryA);
    }
}
