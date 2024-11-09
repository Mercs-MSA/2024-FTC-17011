package org.firstinspires.ftc.teamcode.subSystems;

import static org.firstinspires.ftc.teamcode.Constants.intakePivotScorePos;
import static org.firstinspires.ftc.teamcode.Constants.intakeScorePos;
import static org.firstinspires.ftc.teamcode.Constants.intakeSpinDefault;

import com.acmerobotics.roadrunner.Line;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intakes {
    private Servo intake;
    private Servo intakeSpin;
    private Servo intakePivot;
    private Servo specimenIntake;

    public Intakes(HardwareMap hardwareMap) throws InterruptedException {
        intake = hardwareMap.get(Servo.class, "intake");

        intakeSpin = hardwareMap.get(Servo.class, "intakeSpin");

        intakePivot = hardwareMap.get(Servo.class, "intakePivot");

        specimenIntake = hardwareMap.get(Servo.class, "specimenIntake");

        intake.setPosition(intakeScorePos);

        intakeSpin.setPosition(intakeSpinDefault);

        intakePivot.setPosition(intakePivotScorePos);
    }

    public void clawSetPos(double pos) {
        intake.setPosition(pos);
    }

    public void spinSetPos(double pos) {
        intakeSpin.setPosition(pos);
    }

    public void pivotSetPos(double pos) {
        intakePivot.setPosition(pos);
    }

    public double clawGetPos() { return intake.getPosition(); }

    public void specSetPos(double pos) {
        specimenIntake.setPosition(pos);
    }
}
