package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.startingPose;

public class TestingAuto extends OpMode {
    public Follower follower;
    public double botHeading = startingPose.getHeading();

    public void init() {
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startingPose);
    }
    public void loop() {

    }
}
