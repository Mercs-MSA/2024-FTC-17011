package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    public final static double normalSpeed = 1;
    public final static double slowSpeed = .6;
    public final static double slideTickPerIn = 30.05;

    public final static double pivotTickPerDegree = 3.778;

    //Mechanism Numbers!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    public final static double intakePivotScorePos = 0;
    public final static double intakePivotGrabPos = 0.7; //.55
    public final static double intakeScorePos = .25;
    public final static double intakeHoldPos = 0;

    public final static double intakeSpinDefault = 0.5;
    public final static double intakeSpinRight = 0;
    public final static double intakeSpinLeft = 1;

    public final static double specimenScorePos = 0.55;
    public final static double specimenHoldPos = 0.25;

    public static int pivotUpPos = 420;
    public static int pivotDownPos = 60;

    public final static int highBasketPos = 2270;
    public final static int lowBasketPos = 1210;
    public final static int highSpecimenPos = 1410;
    public final static int highSpecScorePos = 1000;
    public final static double lowBasketInch = 0;

    public final static double climbLowInch = 0;

    public static int defaultState = 0;
    public static int highBasketState = 1;
    public static int highSpecimenState = 2;
}
