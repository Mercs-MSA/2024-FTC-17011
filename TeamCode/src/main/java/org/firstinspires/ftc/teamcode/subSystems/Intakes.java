package org.firstinspires.ftc.teamcode.subSystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.Constants.intakePivotScorePos;
//import static org.firstinspires.ftc.teamcode.Constants.intakeScorePos;
import static org.firstinspires.ftc.teamcode.Constants.intakeSpinDefault;

import com.acmerobotics.roadrunner.Line;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intakes {
    private CRServo intakeRight;
    private CRServo intakeLeft;
    private Servo intakeSpin;
    private Servo intakePivot;
    private Servo specimenIntake;

    private Telemetry telemetry;

    public Intakes(HardwareMap hardwareMap, Telemetry telemetry) throws InterruptedException {
        intakeRight = hardwareMap.get(CRServo.class, "intakeRightWheel");

        intakeLeft = hardwareMap.get(CRServo.class, "intakeLeftWheel");

        intakeSpin = hardwareMap.get(Servo.class, "intakeSpin");

        intakePivot = hardwareMap.get(Servo.class, "intakePivot");

        specimenIntake = hardwareMap.get(Servo.class, "specimenIntake");

        intakeLeft.setDirection(CRServo.Direction.REVERSE);

        this.telemetry = telemetry;


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
        telemetry.addData("Intake Pivot Pos: ", pos);
        intakePivot.setPosition(pos);
    }

    public double pivotGetPos() {return intakePivot.getPosition();}

//    public void pivotSetPow(double pow) {intakePivot.setPower(pow);}

//    public double pivotGetPow() {return intakePivot.getPower();}

//    public double clawGetPos() { return intake.getPosition(); } V1

    public void specSetPos(double pos) {
        specimenIntake.setPosition(pos);
    }
    public double specCheckPos() {
        return specimenIntake.getPosition();
    }

    public double getRightPower() {
        return intakeRight.getPower();
    }
    public double getLeftPower() {
        return intakeLeft.getPower();
    }
}
