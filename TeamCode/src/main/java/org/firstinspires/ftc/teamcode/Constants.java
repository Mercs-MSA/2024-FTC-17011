package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    public final static double normalSpeed = .7;
    public final static double slowSpeed = .35;

    //Mechanism Numbers!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    public final static double intakePivotScorePos = 0;
    public final static double intakePivotMidPos = .3;
    public final static double intakePivotGrabPos = 0.64; //.55
//    public final static double intakeScorePos = .25;
//    public final static double intakeHoldPos = 0;
    public final static double intakeScorePow = 1;
    public final static double intakeCollectPow = -1;

    public final static double intakeSpinDefault = 0.5;
    public final static double intakeSpinRight = 0.25;
    public final static double intakeSpinLeft = .75;
    public final static double intakeSpinBack = 1;

    public final static double specimenScorePos = 0.12; //.45
    public final static double specimenHoldPos = 0.30; //0

    public static int pivotUpPos = 690; //V1 - 420
    public static int pivotDownPos = 60; //V1 - 60
    public static double pivotP = 18;
    public static double pivotI = 1;
    public static double pivotD = .4;
    public static double pivotF = 1;

    public static int highBasketPos = 2200; //V1 - 2270
    public static int lowBasketPos = 900; //V1 - 1210hh
    public static int extendPos = 1400;
    public static int extendPosAuto = 1300;
    public static int highSpecimenPos = 1340; //V1 - 1410
    public static int highSpecScorePos = 960; //V1 - 1050
    public final static double lowBasketInch = 0;

    public final static double climbLowInch = 0;



    //AUTON STUFF:
    public final int firstSpikeSlidePos = 0;
}
