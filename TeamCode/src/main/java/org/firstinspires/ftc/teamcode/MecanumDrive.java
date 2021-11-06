package org.firstinspires.ftc.teamcode;

import androidx.lifecycle.Lifecycle;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.A;

@TeleOp
public class MecanumDrive extends LinearOpMode {

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
    public Servo Twist;
    public Servo Gate;
    public DcMotor Duck_Wheel;
    public DcMotor Intake;
    public DcMotor ArmMotor;
    IntegratingGyroscope gyro;
    ModernRoboticsI2cGyro modernRoboticsI2cGyro;

    /**
     * Initialize all motors that control the robot's accessories
     *
     * Duck_Wheel - Vertically propped motor that controls the duck dropper
     * Intake - Intake Motor
     * ArmMotor - Arm Motor
     */

    public double limit = 1.0D;
    public double limitPower = 0.75D;
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

    /**
    private DcMotor initMotor(
            String motorName,
            DcMotorSimple.Direction direction,
            DcMotor.RunMode runMode
    ) {
        return initMotor(
                motorName,
                direction,
                runMode,
                DcMotor.ZeroPowerBehavior.FLOAT
        );
    }

    private DcMotor initMotor(
            String motorName,
            DcMotorSimple.Direction direction
    ) {
        return initMotor(
                motorName,
                direction,
                DcMotor.RunMode.RUN_USING_ENCODER
        );
    }

    private DcMotor initMotor(
            String motorName
    ) {
        return initMotor(
                motorName,
                DcMotorSimple.Direction.FORWARD
        );
    }
     */
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
        x = x * STRAFING_CORRECTION;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1)*limit;
        double LF_power = (y + x + rx) / denominator;
        double LB_power = (y - x + rx) / denominator;
        double RF_power = (y - x - rx) / denominator;
        double RB_power = (y + x - rx) / denominator;
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

    public double lerp(double p0, double p1, double t) {
        return (1-t)*p0 + t*p1;
    }

    /**
     * Main method that executes upon code run
     */
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        limitPower = 1/limitPower;

        long inputDelay = 500; // Seconds: inputDelay/1000
        double lastTimeDuck = 0D;
        double lastTimeIntake = 0D;

        double lastTimeLimit = 0D;

        double startArmPower = -0.15;
        double Gate_close = 0;
        double Gate_open = 1;
        double Twist_default  = 0 ;
        double Twist_active = 0.5;
        double Top_Start = 0.1445545;
        double Close_Point = 0.067889;

        boolean intakeOn = false;
        boolean duckWheelOn = false;
        boolean limitOn = false;
        boolean armOn = false;
        boolean lastResetState = false;
        boolean curResetState  = false;


        LF = initMotor(
            "LF",
                DcMotorSimple.Direction.FORWARD,
                DcMotor.RunMode.RUN_WITHOUT_ENCODER,
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

        // TODO: Uncomment tfod.loadModeLFromAsset(TFOD_MODEL_ASSET, LABELS); statements
        // TODO: gamepad2.rt = down, gamepad2.lt = up,
        // TODO: Change name on DriverHub from Lift to ArmMotor

        ArmMotor = initMotor(
                "ArmMotor", // TODO: change to ArmMotor
                DcMotorSimple.Direction.REVERSE,
                DcMotor.RunMode.RUN_USING_ENCODER,
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        Duck_Wheel = initMotor(
                "Duck_Wheel",
                DcMotorSimple.Direction.FORWARD,
                DcMotor.RunMode.RUN_WITHOUT_ENCODER,
                DcMotor.ZeroPowerBehavior.BRAKE
        );
      //  ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Twist = hardwareMap.servo.get("Twist");
        Gate = hardwareMap.servo.get("Gate");
        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope)modernRoboticsI2cGyro;
        telemetry.log().add("Gyro Calibrating. Do Not Move!");
        modernRoboticsI2cGyro.calibrate();
        timer.reset();
        while (!isStopRequested() && modernRoboticsI2cGyro.isCalibrating())  {
            telemetry.addData("calibrating", "%s", Math.round(timer.seconds())%2==0 ? "|.." : "..|");
            telemetry.update();
            sleep(50);
        }

        telemetry.log().clear(); telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear(); telemetry.update();
        waitForStart();
        ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (opModeIsActive()) {

            int encoder_LF = LF.getCurrentPosition();
            int encoder_LB = LB.getCurrentPosition();
            int encoder_RF = RF.getCurrentPosition();
            int encoder_RB = RB.getCurrentPosition();
            int encoder_Arm = ArmMotor.getCurrentPosition();
            int heading = modernRoboticsI2cGyro.getHeading();
            int zAccumulated;


            telemetry.addData("LF encoder: ", encoder_LF);
            telemetry.addData("LB encoder: ", encoder_LB);
            telemetry.addData("RF encoder: ", encoder_RF);
            telemetry.addData("RB encoder: ", encoder_RB);
            telemetry.addData("Left Trigger: ", gamepad2.left_trigger);
            telemetry.addData("Right Trigger: ", gamepad2.right_trigger);
            telemetry.addData("Arm encoder: ", encoder_Arm);
            telemetry.addData("Arm Position: ", ArmMotor.getCurrentPosition());
            telemetry.addData("Arm Power: ", lerp(startArmPower, 0, (double) ArmMotor.getCurrentPosition()/500));
            telemetry.addData("heading", "%3d deg", heading);
            telemetry.update();

            //drive train
            mecanum(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            if (gamepad1.y && System.currentTimeMillis() - lastTimeLimit > inputDelay) {
                limit = ( limitOn ? 1 : limitPower );
                limitOn = !limitOn;
            }

            if (gamepad2.x && System.currentTimeMillis() - lastTimeDuck > inputDelay) {
                lastTimeDuck = System.currentTimeMillis();
                duckWheelOn = !duckWheelOn;
                Duck_Wheel.setPower(duckWheelOn ? ( gamepad2.a ? 1 : 0.54 ) : 0);
            }

            if (gamepad1.b || gamepad1.a) {
                if (System.currentTimeMillis() - lastTimeIntake > inputDelay) {
                    if (gamepad1.b) {
                        Intake.setDirection(DcMotorSimple.Direction.REVERSE);
                    } else {
                        Intake.setDirection(DcMotorSimple.Direction.FORWARD);
                    }
                    lastTimeIntake = System.currentTimeMillis();
                    intakeOn = !intakeOn;
                    Intake.setPower(intakeOn ? 0.56 : 0);
                }
            }

            if (gamepad2.left_trigger > 0) {
                ArmMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                ArmMotor.setPower(gamepad2.left_trigger/10);
            } else if (gamepad2.right_trigger > 0) {
                ArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                ArmMotor.setPower(gamepad2.right_trigger/10);
            } else {
                ArmMotor.setPower(0);
            }
            if (gamepad2.a) {
                Twist.setPosition(0);
            }
            else if (gamepad2.b) {
                Twist.setPosition(0.25);
            }
            if (gamepad2.dpad_up) {
            zAccumulated = modernRoboticsI2cGyro.getIntegratedZValue();
            if (zAccumulated > 195) {
                ArmMotor.setPower(Top_Start);
            }
            if (zAccumulated < 160) {
                ArmMotor.setPower(-Top_Start);
            }

            if (zAccumulated > 161) {
                ArmMotor.setPower(Close_Point);
            }
            if (zAccumulated > 194) {
                ArmMotor.setPower(Close_Point);
            }

            }
            /*
            if (armOn && gamepad2.dpad_up) {
                ArmMotor.setTargetPosition(500);
                ArmMotor.setPower(
                        lerp(
                                startArmPower,
                                -0.01,
                                (double) (ArmMotor.getCurrentPosition())/500
                        )
                );
            } else if (gamepad2.dpad_up) {
                armOn = true;
                ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                ArmMotor.setTargetPosition(500);
                Twist.setPosition(0.25);
                ArmMotor.getCurrentPosition();
                ArmMotor.setPower(startArmPower);
                ArmMotor.setTargetPosition(500);
            } else if (armOn) {
                armOn = false;
            }

            if (gamepad2.dpad_left) {
                ArmMotor.setTargetPosition(-500);
                ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ArmMotor.setPower(startArmPower);


            }
            */

            if (gamepad2.right_bumper) {
                ArmMotor.setPower(-0.14);
            } else if (gamepad2.left_bumper) {
                ArmMotor.setPower(0.14);
            }
        }
    }}
