package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class MecanumDrive extends LinearOpMode {

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

    public DcMotor Duck_Wheel;

    public DcMotor Intake;

    public DcMotor LiftMotor;

    /**
     * Applies the needed power to move the robot
     *
     * @param y     First Variable used in the calculation of the power
     * @param x     Second variable used in the calculations of the power
     * @param rx    Third variable used in the calculations of the power
     */
    public void mecanum(double y, double x, double rx) {
        // Change if strafe bad
        double STRAFING_CORRECTION = 1.0D;
        // TODO: Change gamepad1.left_stick_x to x?
        x = gamepad1.left_stick_x * STRAFING_CORRECTION;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double LF_power = (y + x + rx) / denominator/3;
        double LB_power = (y - x + rx) / denominator/3;
        double RF_power = (y - x - rx) / denominator/3;
        double RB_power = (y + x - rx) / denominator/3;
        //denominator/power = speed

        LF.setPower(LF_power);
        RF.setPower(RF_power);
        LB.setPower(LB_power);
        RB.setPower(RB_power);
    }

    /**
     * Applies the same power to all of the wheels
     *
     * @param power     The power to be set to all of the motors
     */
    public void wheels_power(double power) {
        LF.setPower(power);
        RB.setPower(power);
        LB.setPower(power);
        RF.setPower(power);
    }

    /**
     * Main method that executes upon code run
     */
    @Override
    public void runOpMode() {
        double lastTimeDuck = 0D;
        double lastTimeIntake = 0D;

        boolean intakeOn = false;
        boolean duckWheelOn = false;

        boolean liftDownOn = false;
        boolean liftUpOn = false;

        LF = hardwareMap.dcMotor.get("LF");
        RF = hardwareMap.dcMotor.get("RF");
        LB = hardwareMap.dcMotor.get("LB");
        RB = hardwareMap.dcMotor.get("RB");

        //right side motors are reversed, change to left side if needed
        LF.setDirection(DcMotorSimple.Direction.FORWARD);
        RF.setDirection(DcMotorSimple.Direction.FORWARD);
        RB.setDirection(DcMotorSimple.Direction.REVERSE);//r
        LB.setDirection(DcMotorSimple.Direction.FORWARD);

        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        Duck_Wheel = hardwareMap.dcMotor.get("Duck_Wheel");
        Duck_Wheel.setDirection(DcMotorSimple.Direction.FORWARD);
        Duck_Wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Duck_Wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Intake = hardwareMap.dcMotor.get("Intake");
        Intake.setDirection(DcMotorSimple.Direction.FORWARD);
        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Not sure if it works
        // TODO: gamepad2.rt = down, gamepad2.lt = up,
        LiftMotor = hardwareMap.dcMotor.get("Lift");
        LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();
        while (opModeIsActive()) {
            int encoder_LF = LF.getCurrentPosition();
            int encoder_LB = LB.getCurrentPosition();
            int encoder_RF = RF.getCurrentPosition();
            int encoder_RB = RB.getCurrentPosition();
            telemetry.addData("LF encoder: ", encoder_LF);
            telemetry.addData("LB encoder: ", encoder_LB);
            telemetry.addData("RF encoder: ", encoder_RF);
            telemetry.addData("RB encoder: ", encoder_RB);
            telemetry.addData("Left Trigger:", gamepad2.left_trigger);
            telemetry.addData("Right Trigger: ", gamepad2.right_trigger);
            telemetry.update();

            //drive train
            mecanum(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);


            if (gamepad2.x && System.currentTimeMillis() - lastTimeDuck > 500) {
                lastTimeDuck = System.currentTimeMillis();
                duckWheelOn = !duckWheelOn;
                Intake.setPower(duckWheelOn ? 0.54 : 0);
            }

            if (gamepad2.a && System.currentTimeMillis() - lastTimeIntake > 500) {
                lastTimeIntake = System.currentTimeMillis();
                intakeOn = !intakeOn;
                Intake.setPower(intakeOn ? 0.56 : 0);
            }

            if (gamepad2.left_trigger > 0) {
                if (!liftUpOn) {
                    liftDownOn = false;
                    liftUpOn = true;
                    LiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                    LiftMotor.setPower(gamepad2.left_trigger);
                }
            } else if (gamepad2.right_trigger > 0) {
                if (!liftDownOn) {
                    liftUpOn = false;
                    liftDownOn = true;
                    LiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                    LiftMotor.setPower(gamepad2.right_trigger);
                }
            }
        }
    }
}