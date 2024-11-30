package org.firstinspires.ftc.teamcode.subSystems;

import static org.firstinspires.ftc.teamcode.Constants.intakePivotScorePos;
//import static org.firstinspires.ftc.teamcode.Constants.intakeScorePos;
import static org.firstinspires.ftc.teamcode.Constants.intakeSpinDefault;

import com.acmerobotics.roadrunner.Line;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intakes {
    private CRServo intakeRight;
    private CRServo intakeLeft;
    private Servo intakeSpin;
    private Servo intakePivot;
    private Servo specimenIntake;

    public Intakes(HardwareMap hardwareMap) throws InterruptedException {
        intakeRight = hardwareMap.get(CRServo.class, "intakeRightWheel");

        intakeLeft = hardwareMap.get(CRServo.class, "intakeLeftWheel");

        intakeSpin = hardwareMap.get(Servo.class, "intakeSpin");

        intakePivot = hardwareMap.get(Servo.class, "intakePivot");

        specimenIntake = hardwareMap.get(Servo.class, "specimenIntake");

        intakeLeft.setDirection(CRServo.Direction.REVERSE);



        intakeRight.setPower(0);

        intakeLeft.setPower(0);
//        intakePivot.setPosition(intakePivotScorePos); V1
    }

//    public void clawSetPos(double pos) { V1
//        intake.setPosition(pos);
//    }
    public void setIntakePower(double pow) {
        intakeRight.setPower(pow);
        intakeLeft.setPower(pow);
    }

    public void spinSetPos(double pos) {
        intakeSpin.setPosition(pos);
    }

    public void pivotSetPos(double pos) {
        intakePivot.setPosition(pos);
    }

//    public double clawGetPos() { return intake.getPosition(); } V1

    public void specSetPos(double pos) {
        specimenIntake.setPosition(pos);
    }
    public double specCheckPos() {
        return specimenIntake.getPosition();
    }
}
