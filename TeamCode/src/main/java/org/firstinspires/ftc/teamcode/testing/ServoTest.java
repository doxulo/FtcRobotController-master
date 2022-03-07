package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(group = "Test")
@Disabled
public class ServoTest extends LinearOpMode {

    Servo twist;


    @Override
    public void runOpMode() throws InterruptedException {
        twist = hardwareMap.servo.get("Twist");
        waitForStart();

        twist.setPosition(1);
        sleep(1000);
        twist.setPosition(0);
        sleep(1000);
        twist.setPosition(1);
        sleep(1000);
        twist.setPosition(0);
        sleep(1000);
        twist.setPosition(1);
        sleep(1000);
        twist.setPosition(0);
        sleep(1000);

    }
}
