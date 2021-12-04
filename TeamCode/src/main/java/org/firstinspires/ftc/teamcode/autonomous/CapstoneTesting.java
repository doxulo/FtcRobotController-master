package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;

@Autonomous
public class CapstoneTesting extends LinearOpMode {

    Servo capstoneLift;

    public void runOpMode() {
        capstoneLift = hardwareMap.servo.get("CapstoneLift");
        capstoneLift.scaleRange(0, 1);
        waitForStart();
        for (double i = 0; i < 1; i=i+0.01) {
            capstoneLift.setPosition(i);
        }
    }
}
