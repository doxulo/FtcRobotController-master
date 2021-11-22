package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.HashMap;

public class Map extends LinearOpMode {

    // TODO: Implement class to clean up code
    // TODO: Test class
    // TODO: Use different method to get hardware map
    HashMap<String, DcMotor> objectMap = new HashMap<>();
    String[] names = new String[] {
            "LF",
            "RF",
            "LB",
            "RB",
            "Intake",
            "ArmMotor",
            "Duck_Wheel"
    };

    DcMotorSimple.Direction[] directions = new DcMotorSimple.Direction[] {
            DcMotorSimple.Direction.FORWARD,
            DcMotorSimple.Direction.FORWARD,
            DcMotorSimple.Direction.FORWARD,
            DcMotorSimple.Direction.REVERSE,
            DcMotorSimple.Direction.FORWARD,
            DcMotorSimple.Direction.FORWARD,
            DcMotorSimple.Direction.FORWARD
    };

    DcMotor.RunMode[] runModes = new DcMotor.RunMode[] {
            DcMotor.RunMode.RUN_WITHOUT_ENCODER,
            DcMotor.RunMode.RUN_WITHOUT_ENCODER,
            DcMotor.RunMode.RUN_WITHOUT_ENCODER,
            DcMotor.RunMode.RUN_WITHOUT_ENCODER,
            DcMotor.RunMode.RUN_WITHOUT_ENCODER,
            DcMotor.RunMode.STOP_AND_RESET_ENCODER,
            DcMotor.RunMode.RUN_WITHOUT_ENCODER
    };

    DcMotor.ZeroPowerBehavior[] behaviors = new DcMotor.ZeroPowerBehavior[] {
            DcMotor.ZeroPowerBehavior.FLOAT,
            DcMotor.ZeroPowerBehavior.FLOAT,
            DcMotor.ZeroPowerBehavior.FLOAT,
            DcMotor.ZeroPowerBehavior.FLOAT,
            DcMotor.ZeroPowerBehavior.FLOAT,
            DcMotor.ZeroPowerBehavior.BRAKE,
            DcMotor.ZeroPowerBehavior.BRAKE

    };

    /**
     * Possible function to initialize and setup future motors
     *
     * @param motorName         Name to index the motor
     * @param direction         Direction of the motor's power
     * @param runMode           Reverse or Forward power motion
     * @param zeroPowerBehavior Zero Power Behavior of the motor
     * @return Setup motor
     */
    private DcMotor initMotor(
            String motorName,
            DcMotorSimple.Direction direction,
            DcMotor.RunMode runMode,
            DcMotor.ZeroPowerBehavior zeroPowerBehavior
    ) {
        DcMotor motor = hardwareMap.dcMotor.get(motorName);
        motor.setDirection(direction);
        motor.setMode(runMode);
        motor.setZeroPowerBehavior(zeroPowerBehavior);

        return motor;
    }

    public void runOpMode() {}

    public void initMotors() {
        for (int i = 0; i < names.length; i++) {
            objectMap.put(names[i], initMotor(
                    names[i],
                    directions[i],
                    runModes[i],
                    behaviors[i]
            ));
        }
    }

    public DcMotor get(String name) {
        return this.get(name);
    }
}
