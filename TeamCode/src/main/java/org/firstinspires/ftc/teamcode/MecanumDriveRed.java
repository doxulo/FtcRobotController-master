package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Debounce;
import org.firstinspires.ftc.teamcode.util.DebounceObject;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.util.Switch;


@TeleOp
public class MecanumDriveRed extends LinearOpMode {

    /** Global comments:
     * GamePad1 == For movements,
     * GamePad2 == Gadgets,
     */


    /**
     * Initialize all motors that control the robot's wheels
     * <p>
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
    public ColorSensor BoxSensor;

    /**
     * Initialize all motors that control the robot's accessories
     * <p>
     * Duck_Wheel - Vertically propped motor that controls the duck dropper
     * Intake - Intake Motor
     * ArmMotor - Arm Motor
     */

    public double limit = 1.0D;
    public double limitPower = 0.75D;

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

    /**
     * Applies the needed power to move the robot
     *
     * @param y  First Variable used in the calculation of the power
     * @param x  Second variable used in the calculations of the power
     * @param rx Third variable used in the calculations of the power
     */
    public void mecanum(double y, double x, double rx) {
        // Change if strafe bad
        double STRAFING_CORRECTION = 1.0D;
        x = x * STRAFING_CORRECTION;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1) * limit;
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

    public double lerp(double p0, double p1, double t) {
        return (1-t)*p0 + p1*t;
    }

    // 647
    // 815
    // 940
    public double getPowerTill(double maxPower, double targetPosition, double currentPosition) {
        currentPosition = currentPosition == 0 ? 0.1 : currentPosition;
        return lerp(maxPower, 0, targetPosition/currentPosition);
    }
    /**
     * Main method that executes upon code run
     */
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        limitPower = 1 / limitPower;

        Debounce debounces = new Debounce(
                new DebounceObject("Duck", 750),
                new DebounceObject("Intake", 500),
                new DebounceObject("Arm", 500),
                new DebounceObject("Limit", 500),
                new DebounceObject("Gate", 500),
                new DebounceObject("Twist", 500),
                new DebounceObject("Integral", 500),
                new DebounceObject("Derivative", 500)
        );

        Switch armMotorSwitch = new Switch(false);
        Switch duckMotorSwitch = new Switch(false);

        // Change kI
        PIDController controller = new PIDController(
                0.005,
                1.0e-10,
                0.003,
                new double[] {
                        0.05, -0.10
                },
                1440,
                -235);
        
        double[] twistPositions = new double[] {
                0.615D, 0.7D, 0.9D
        };
        double defaultPower = 0.56D;
        double intIncrement = 0.00001;
        long startDuck = 0;

        int twistIndex = -1;
        boolean intakeOn = false;
        boolean duckWheelOn = false;
        boolean limitOn = false;
        boolean resetArmPower = false;

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

        ArmMotor = initMotor(
                "ArmMotor", // TODO: change to ArmMotor
                DcMotorSimple.Direction.REVERSE,
                DcMotor.RunMode.RUN_USING_ENCODER,
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        Duck_Wheel = initMotor(
                "Duck_Wheel",
                DcMotorSimple.Direction.FORWARD,
                DcMotor.RunMode.RUN_WITHOUT_ENCODER,
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        Twist = hardwareMap.servo.get("Twist");
        BoxSensor = hardwareMap.colorSensor.get("Boxsensor");

        waitForStart();
        ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        long lastTime = System.currentTimeMillis();
        while (opModeIsActive()) {
            long currentSystemTime = System.currentTimeMillis();
            int encoder_LF = LF.getCurrentPosition();
            int encoder_LB = LB.getCurrentPosition();
            int encoder_RF = RF.getCurrentPosition();
            int encoder_RB = RB.getCurrentPosition();
            int encoder_Arm = Math.abs(ArmMotor.getCurrentPosition());
            double power = 0;

            telemetry.addData("LF encoder: ", encoder_LF);
            telemetry.addData("LB encoder: ", encoder_LB);
            telemetry.addData("RF encoder: ", encoder_RF);
            telemetry.addData("RB encoder: ", encoder_RB);
            telemetry.addData("Left Trigger: ", gamepad2.left_trigger);
            telemetry.addData("Right Trigger: ", gamepad2.right_trigger);
            telemetry.addData("Arm encoder: ", encoder_Arm);
            telemetry.addData("Arm Power: ", ArmMotor.getPower());
            telemetry.addData("Dt: ", currentSystemTime-lastTime);
            telemetry.addData(String.format("Red: %d, Green: %d, Blue: %d", BoxSensor.red(), BoxSensor.green(), BoxSensor.blue()), "");
            telemetry.addData("Integral: ", controller.summation);
            telemetry.addData("kD: ", controller.kI);
            telemetry.addData("Power: ", power);
            telemetry.addData("Increment: ", intIncrement);
            telemetry.addData("Buttons: ", gamepad2.dpad_right);
            telemetry.update();
            lastTime = currentSystemTime;
            //drive train
            mecanum(-Math.pow(gamepad1.left_stick_y, 3D), Math.pow(gamepad1.left_stick_x, 3D), Math.pow(gamepad1.right_stick_x, 3D));

            if (gamepad1.y && debounces.checkAndUpdate("Limit")) {
                limit = (limitOn ? 1 : limitPower);
                limitOn = !limitOn;
            }

            if (gamepad1.b || gamepad1.a) {
                if (debounces.checkAndUpdate("Intake")) {
                    if (gamepad1.b) {
                        Intake.setDirection(DcMotorSimple.Direction.REVERSE);
                    } else {
                        Intake.setDirection(DcMotorSimple.Direction.FORWARD);
                    }
                    intakeOn = !intakeOn;
                    Intake.setPower(intakeOn ? 0.56 : 0);
                }
            }

            if (gamepad2.a && debounces.checkAndUpdate("Twist")) {
                Twist.setPosition(twistPositions[0]);
            } else if (gamepad2.b) {
                Twist.setPosition(twistPositions[1]);
            } else if (gamepad2.y) {
                Twist.setPosition(twistPositions[2]);
            }
            /*
            double modifiedHeading = (double) heading;
            if (heading > 350) {
                modifiedHeading = 0D;
            }
            modifiedHeading++;
            if (gamepad2.dpad_up) {
                ArmMotor.setPower(getPowerTill(-1, 150D, modifiedHeading));
            } else if (gamepad2.dpad_left) {
                ArmMotor.setPower(getPowerTill(-1, 190D, modifiedHeading));
            } else if (gamepad2.dpad_down) {
                ArmMotor.setPower(getPowerTill(-1, 220D, modifiedHeading));
            } else if (gamepad2.x) {
                ArmMotor.setPower(getPowerTill(-0.5, 1D, modifiedHeading));
            }
             */

            // 647
            // 815
            // 940


            if (gamepad2.right_stick_button && debounces.checkAndUpdate("Integral")) {
                controller.kI += intIncrement;
            } else if (gamepad2.dpad_right && debounces.checkAndUpdate("Integral")) {
                controller.kI -= intIncrement;
            } else if (gamepad2.left_bumper && debounces.checkAndUpdate("Integral")) {
                intIncrement *= 10;
            } else if (gamepad2.right_bumper && debounces.checkAndUpdate("Integral")) {
                intIncrement /= 10;
            }

            if (gamepad2.dpad_up) {
                power = controller.calculate(647D, encoder_Arm);
            } else if (gamepad2.dpad_left) {
                power = controller.calculate(815D, encoder_Arm);
            } else if (gamepad2.dpad_down) {
                power = controller.calculate(940D, encoder_Arm);
            } else if (gamepad2.x) {
                if (encoder_Arm > 70) {
                    power = controller.calculate(20D, encoder_Arm)/2;
                } else if (encoder_Arm > 0) {
                    power = controller.lerp(0.15, 0, (double) encoder_Arm/70);
                }
            } else if (gamepad2.left_stick_button) {
                ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } else if (gamepad2.left_trigger > 0) {
                ArmMotor.setPower(-gamepad2.left_trigger);
            } else if (gamepad2.right_trigger > 0) {
                ArmMotor.setPower(gamepad2.right_trigger);
            }  else if (resetArmPower) {
                resetArmPower = false;
                power = 0;
                controller.pauseAndReset();
                ArmMotor.setPower(0);
            } else if (ArmMotor.getPower() != 0) {
                ArmMotor.setPower(0);
            }
            // 200
            if (power != 0) {
                resetArmPower = true;
                ArmMotor.setPower(power/2.5);
            }
            /*
            if (gamepad2.dpad_up) {
                resetArmPower = true;
                ArmMotor.setTargetPosition(650);
                ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ArmMotor.setPower(-1);
            } else if (gamepad2.dpad_left) {
                resetArmPower = true;
                ArmMotor.setTargetPosition(815);
                ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ArmMotor.setPower(-1);
            } else if (gamepad2.dpad_down) {
                resetArmPower = true;
                ArmMotor.setTargetPosition(945);
                ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ArmMotor.setPower(-1);
            } else if (gamepad2.x) {
                resetArmPower = true;
                ArmMotor.setTargetPosition(0);
                ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ArmMotor.setPower(-0.4);
            } else if (resetArmPower) {
                resetArmPower = false;
                ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                ArmMotor.setPower(0);
            }

             */
            /*
            if (gamepad2.dpad_left && debounces.checkAndUpdate("Arm")) {
                ArmMotor.setTargetPosition(700);
                ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ArmMotor.setPower(0.5);
            }
            if (gamepad2.dpad_right && debounces.checkAndUpdate("Arm")) {
                ArmMotor.setTargetPosition(850);
                ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ArmMotor.setPower(0.5);
            }
            if (gamepad2.dpad_down && debounces.checkAndUpdate("Arm")) {
                ArmMotor.setTargetPosition(950);
                ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ArmMotor.setPower(0.5);
            }
            if (gamepad2.dpad_up && debounces.checkAndUpdate("Arm")) {
                ArmMotor.setTargetPosition(0);
                ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ArmMotor.setPower(-0.5);
            }
            */
            /*
            if (gamepad2.left_bumper) {
                if (startDuck == 0) {
                    duckMotorSwitch.setTrue();
                    startDuck = System.currentTimeMillis();
                    Duck_Wheel.setPower(0.56);
                } else {
                    if (System.currentTimeMillis() - startDuck > 750) {
                        Duck_Wheel.setPower(1);
                    }
                }
            } else if (gamepad2.right_bumper) {
                if (startDuck == 0) {
                    duckMotorSwitch.setTrue();
                    startDuck = System.currentTimeMillis();
                    Duck_Wheel.setPower(-0.56);
                } else {
                    if (System.currentTimeMillis() - startDuck > 750) {
                        Duck_Wheel.setPower(-1);
                    }
                }
            } else if (duckMotorSwitch.check()){
                duckMotorSwitch.trigger();
                startDuck = 0;
                Duck_Wheel.setPower(0);
            }


             */
        }
    }
}
