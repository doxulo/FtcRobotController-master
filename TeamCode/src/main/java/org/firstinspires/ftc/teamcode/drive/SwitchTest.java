package org.firstinspires.ftc.teamcode.drive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;


@Autonomous
public class SwitchTest extends LinearOpMode {

    DigitalChannel switch_1;

    @Override
    public void runOpMode() throws InterruptedException {

        switch_1 = hardwareMap.get(DigitalChannel.class, "switch_1");
        switch_1.setMode(DigitalChannel.Mode.INPUT);
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("Switch state:", switch_1.getState());
            telemetry.update();
        }
    }
}
