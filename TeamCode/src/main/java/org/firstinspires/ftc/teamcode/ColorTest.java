package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class ColorTest extends LinearOpMode {


    public void runOpMode() {

        RevBlinkinLedDriver lights = hardwareMap.get(RevBlinkinLedDriver.class, "BoxLight");


    }
}
