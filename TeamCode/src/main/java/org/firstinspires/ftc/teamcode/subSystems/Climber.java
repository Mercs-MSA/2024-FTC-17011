package org.firstinspires.ftc.teamcode.subSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Climber {
    private Servo leftClimb;
    private Servo rightClimb;

    public Climber(HardwareMap hardwareMap) throws InterruptedException {
        leftClimb = hardwareMap.get(Servo.class, "leftClimb");

        rightClimb = hardwareMap.get(Servo.class, "rightClimb");
    }

    public void setClimbers(double pos) {
        leftClimb.setPosition(pos);
        rightClimb.setPosition(pos);
    }

    public double getLeftClimber() {
        return leftClimb.getPosition();
    }

    public double getRightClimber() {
        return rightClimb.getPosition();
    }
}
