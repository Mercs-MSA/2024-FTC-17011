package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    public final static double normalSpeed = 1;
    public final static double slowSpeed = .6;

    //Mechanism Numbers!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    public final static double intakePivotScorePos = 0;
    public final static double intakePivotGrabPos = 0.7; //.55
//    public final static double intakeScorePos = .25;
//    public final static double intakeHoldPos = 0;
    public final static double intakeScorePow = 1;
    public final static double intakeCollectPow = -1;

    public final static double intakeSpinDefault = 0.5;
    public final static double intakeSpinRight = 0.1;
    public final static double intakeSpinLeft = .9;

    public final static double specimenScorePos = 0.45;
    public final static double specimenHoldPos = 0;

    public static int pivotUpPos = 420;
    public static int pivotDownPos = 60;
    public static double pivotP = 18;
    public static double pivotI = 1;
    public static double pivotD = .4;
    public static double pivotF = 1;

    public final static int highBasketPos = 2270;
    public final static int lowBasketPos = 1210;
    public final static int highSpecimenPos = 1410;
    public final static int highSpecScorePos = 1050;
    public final static double lowBasketInch = 0;

    public final static double climbLowInch = 0;



    //AUTON STUFF:
    public final int firstSpikeSlidePos = 0;
}
