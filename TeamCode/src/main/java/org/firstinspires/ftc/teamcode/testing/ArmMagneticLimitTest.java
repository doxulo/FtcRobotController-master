package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(group = "Test")
public class ArmMagneticLimitTest extends LinearOpMode {
    // Define variables for our touch sensor and motor
    TouchSensor magnetic;

    @Override
    public void runOpMode() {
        // Get the touch sensor and motor from hardwareMap
        magnetic = hardwareMap.get(TouchSensor.class, "magnetic");

        // Wait for the play button to be pressed
        waitForStart();

        // Loop while the Op Mode is running
        while (opModeIsActive()) {
            // If the Magnetic Limit Swtch is pressed, stop the motor
            if (magnetic.isPressed()) {
                telemetry.addData("Magnetic Switch: ", "activated");
            } else { // Otherwise, run the motor
                  telemetry.addData("Magnetic Switch:", "no activated");
            }
            telemetry.update();
        }
    }
}