package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class ServoTest extends LinearOpMode {

    Servo twist;


    @Override
    public void runOpMode() throws InterruptedException {
        twist = hardwareMap.servo.get("LeftOdometryServo");
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
