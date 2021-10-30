package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class MainAuto extends LinearOpMode {


    /** Global comments:
     * GamePad1 == For movements,
     * GamePad2 == Gadgets,
     */


    /**
     * Initialize all motors that control the robot's wheels
     *
     * RF - Right Front wheel
     * RB - Right Back wheel
     * LF - Left Front wheel
     * LB - Left Back wheel
     */
    public DcMotor LF; // 1
    public DcMotor RF; // 0
    public DcMotor LB; // 3
    public DcMotor RB; // 2

    /**
     * Initialize all motors that control the robot's accessories
     *
     * Duck_Wheel - Vertically propped motor that controls the duck dropper
     * Intake - Intake Motor
     * ArmMotor - Arm Motor
     */
    public DcMotor Duck_Wheel;

    public DcMotor Intake;

    public DcMotor ArmMotor;

    /**
     * Possible function to initialize and setup future motors
     *
     * @param motorName             Name to index the motor
     * @param direction             Direction of the motor's power
     * @param runMode               Reverse or Forward power motion
     * @param zeroPowerBehavior     Zero Power Behavior of the motor
     * @return                      Setup motor
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

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        LF = initMotor(
                "LF",
                DcMotorSimple.Direction.FORWARD,
                DcMotor.RunMode.RUN_USING_ENCODER,
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        RF = initMotor(
                "RF",
                DcMotorSimple.Direction.FORWARD,
                DcMotor.RunMode.RUN_WITHOUT_ENCODER,
                DcMotor.ZeroPowerBehavior.FLOAT

        );

        LB = initMotor(
                "LB",
                DcMotorSimple.Direction.FORWARD,
                DcMotor.RunMode.RUN_WITHOUT_ENCODER,
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        RB = initMotor(
                "RB",
                DcMotorSimple.Direction.REVERSE,
                DcMotor.RunMode.RUN_WITHOUT_ENCODER,
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        Intake = initMotor(
                "Intake",
                DcMotorSimple.Direction.FORWARD,
                DcMotor.RunMode.RUN_WITHOUT_ENCODER,
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        // TODO: gamepad2.rt = down, gamepad2.lt = up,
        // TODO: Change name on DriverHub from Lift to ArmMotor
        ArmMotor = initMotor(
                "ArmMotor",
                DcMotorSimple.Direction.FORWARD,
                DcMotor.RunMode.RUN_WITHOUT_ENCODER,
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        Duck_Wheel = initMotor(
                "Duck_Wheel",
                DcMotorSimple.Direction.FORWARD,
                DcMotor.RunMode.RUN_WITHOUT_ENCODER,
                DcMotor.ZeroPowerBehavior.BRAKE
        );
        waitForStart();


    }
}
