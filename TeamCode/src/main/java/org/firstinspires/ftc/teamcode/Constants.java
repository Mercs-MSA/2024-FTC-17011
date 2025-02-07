package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    public final static double normalSpeed = 1;
    public final static double slowSpeed = .45;

    //Mechanism Numbers!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    public final static double intakePivotScorePos = 0.3; //V2 = 0
    public final static double intakePivotMidPos = .42; //V2 = .3
    public final static double intakePivotGrabPos = 0.98; //V1 = .55; V2 = .64
    public final static double intakeScorePos = .25;
    public final static double intakeHoldPos = 0;
    public final static double intakeScorePow = 1;
    public final static double intakeCollectPow = -1;

    public final static double intakeSpinDefault = 0.5;
    public final static double intakeSpinRight = 0.75;
    public final static double intakeSpinLeft = .25;
    public final static double intakeSpinBack = 1;

    public final static double specimenScorePos = 0.45; //V1 - .45 //V2 - .12
    public final static double specimenHoldPos = 0.63; //V1 - //V2 - .3





    // 0 //V2 - .3

    public static int pivotUpPos = 250; //V1 - 420, V2 - 960
    public static int pivotDownPos = 20; //V1 - 60, V2 - 60
    public static double pivotP = 28;
    public static double pivotI = 1.1;
    public static double pivotD = .4;
    public static double pivotF = .8;

    public static int highBasketPos = 2200; //V1 - 2270
    public static int climbPos = 1000; //V1 - 1210
    public static int extendPos = 1400;
    public static int extendPosAuto = 1400;
    public static int highSpecimenPos = 1320; //V1 - 1410
    public static int highSpecScorePos = 920; //V1 - 1050
    public final static double lowBasketInch = 0;

    public final static double climbLowInch = 0;



    //AUTON STUFF:
    public final int firstSpikeSlidePos = 0;
}
