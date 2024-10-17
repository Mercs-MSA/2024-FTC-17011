package org.firstinspires.ftc.teamcode.roadRunnerActions;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
public class Intake {
    private Servo claw;
    private Servo score;
    private Servo axis;

    public Intake(HardwareMap hardwareMap) {
        claw = hardwareMap.get(Servo.class, "claw");
        claw.setPosition(0.1);

        score = hardwareMap.get(Servo.class, "score");
        score.setPosition(0.1);

        axis = hardwareMap.get(Servo.class, "axis");
        score.setPosition(0.1);
    }
}