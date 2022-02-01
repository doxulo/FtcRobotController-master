package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp
public class Danny_ClassProject extends LinearOpMode {
    public DcMotor LF; // 1
    public DcMotor RF; // 0
    public DcMotor LB; // 3
    public DcMotor RB; // 2
    public DcMotor Roller; // EH1

    @Override
    public void runOpMode() throws InterruptedException {
        LF.setPower(gamepad1.left_stick_y);
        LB.setPower(gamepad1.left_stick_y);
        RF.setPower(gamepad1.right_stick_y);
        RB.setPower(gamepad1.right_stick_y);
        Roller.setPower(.7);
        if (gamepad1.a) { Roller.setPower(0);  }

}}
