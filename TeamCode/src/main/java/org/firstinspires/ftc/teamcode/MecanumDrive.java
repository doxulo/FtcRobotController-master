package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Debounce;
import org.firstinspires.ftc.teamcode.util.DebounceObject;
import org.firstinspires.ftc.teamcode.util.Map;
import org.firstinspires.ftc.teamcode.util.Switch;


/**
 * GamePad1 == For movements,
 * GamePad2 == Gadgets,
 */

@Disabled
public class MecanumDrive extends LinearOpMode {

    /**
     * Map variable to get motors
     */
    Map objectMap = new Map();

    /**
     * Define all motors that control the robot's wheels
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

    /**
     * Define motors that controls arm movement
     */
    public Servo Twist;
    public DcMotor ArmMotor;

    /**
     * Define motors that control accessories
     */
    public DcMotor Duck_Wheel;
    public DcMotor Intake;

    /**
     * Define Gyro
     */
    IntegratingGyroscope gyro;
    ModernRoboticsI2cGyro modernRoboticsI2cGyro;

    /**
     * Initialize utility variables that help control movement
     */

    public double limit = 1.0D;
    public double limitPower = 0.75D;

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

    /**
     * Linearly interpolates between 2 points
     *
     * @param p0        Start point
     * @param p1        End point
     * @param t         Percent of the way to the End Pointer
     * @return          Interpolated value between 'p0' and 'p1' 't' percent of the way
     */
    public double lerp(double p0, double p1, double t) {
        return (1-t)*p0 + p1*t;
    }

    /**
     * Wrapper towards the lerp function to get the needed power to reach a point
     * @param maxPower          Maximum power
     * @param targetPosition    Target position/degrees
     * @param currentPosition   Current position/degrees
     * @return                  Power needed to apply to the motor (on a scale of maxPower - 0
     *                          provided 'targetPosition'/'currentPosition' are in bounds of 0 - 1)
     */
    public double getPowerTill(double maxPower, double targetPosition, double currentPosition) {
        currentPosition = currentPosition == 0 ? 1 : currentPosition;
        return lerp(maxPower, 0, targetPosition/currentPosition);
    }

    /**
     * Main method that executes upon code run
     */
    @Override
    public void runOpMode() {
        limitPower = 1 / limitPower;

        Debounce debounces = new Debounce(
                new DebounceObject("Duck", 750),
                new DebounceObject("Intake", 500),
                new DebounceObject("Arm", 500),
                new DebounceObject("Limit", 500),
                new DebounceObject("Gate", 500),
                new DebounceObject("Twist", 500)
        );

        Switch duckMotorSwitch = new Switch(false);
        Switch armMotorSwitch = new Switch(false);

        ElapsedTime timer = new ElapsedTime();

        final int TOP_LEVEL_ENCODER_POSITION = 650;
        final int MIDDLE_LEVEL_POSITION = 815;
        final int BOTTOM_LEVEL_POSITION = 945;

        double Twist_default = 0.16D;
        double Twist_active = 0.51D;
        double defaultPower = 0.56D;
        double armPower = -1.0D;

        long startDuck = 0;

        int armTargetPosition = -1;
        int lastArmPosition = 0;

        boolean intakeOn = false;
        boolean duckWheelOn = false;
        boolean limitOn = false;
        boolean twistOn = false;
        boolean resetArmPower = false;
        // TODO: Uncomment tfod.loadModeLFromAsset(TFOD_MODEL_ASSET, LABELS); statements

        /*
          Initialize motors using the Map object
         */
        objectMap.initMotors();
        LF = objectMap.get("LF");
        RF = objectMap.get("RF");
        LB = objectMap.get("LB");
        RB = objectMap.get("RB");
        Intake = objectMap.get("Intake");
        ArmMotor = objectMap.get("ArmMotor");
        Duck_Wheel = objectMap.get("Duck_Wheel");

        Twist = hardwareMap.servo.get("Twist");

        /*
         * Initialize gyro motor


        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope) modernRoboticsI2cGyro;
        telemetry.log().add("Gyro Calibrating. Do Not Move!");
        modernRoboticsI2cGyro.calibrate();
        timer.reset();
        while (!isStopRequested() && modernRoboticsI2cGyro.isCalibrating()) {
            telemetry.addData("calibrating", "%s", Math.round(timer.seconds()) % 2 == 0 ? "|.." : "..|");
            telemetry.update();
            sleep(50);
        }

        telemetry.log().clear();
        telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear();
        telemetry.update();
         */
        ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();

        /*
         * Main loop
         */
        while (opModeIsActive()) {

            int encoder_LF = LF.getCurrentPosition();
            int encoder_LB = LB.getCurrentPosition();
            int encoder_RF = RF.getCurrentPosition();
            int encoder_RB = RB.getCurrentPosition();
            int encoder_Arm = ArmMotor.getCurrentPosition();
            int heading = modernRoboticsI2cGyro.getHeading();

            /*
             * Output to Driver Hub
             */
            telemetry.addData("LF encoder: ", encoder_LF);
            telemetry.addData("LB encoder: ", encoder_LB);
            telemetry.addData("RF encoder: ", encoder_RF);
            telemetry.addData("RB encoder: ", encoder_RB);
            telemetry.addData("Left Trigger: ", gamepad2.left_trigger);
            telemetry.addData("Right Trigger: ", gamepad2.right_trigger);
            telemetry.addData("Arm encoder: ", encoder_Arm);
            telemetry.addData("Arm Position: ", ArmMotor.getCurrentPosition());
            telemetry.addData("Arm Power: ", ArmMotor.getPower());
            telemetry.addData("heading", "%3d deg", heading);
            telemetry.addData("Twist Position: ", Twist.getPosition());
            telemetry.addData("Twist Orientation: ", Twist.getDirection());
            telemetry.update();

            /*
             * Move robot
             */
            mecanum(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            /*
             * Movement limiter
             */
            if (gamepad1.y && debounces.checkAndUpdate("Limit")) {
                limit = (limitOn ? 1 : limitPower);
                limitOn = !limitOn;
            }

            /*
             * Intake motor movement
             */
            if (gamepad1.b || gamepad1.a) {
                if (debounces.checkAndUpdate("Intake")) {
                    intakeOn = !intakeOn;
                    // gamepad1.b => reversed | gamepad1.a normal
                    int multiple = 1;
                    if (gamepad1.b) {
                        multiple = -1;
                    }
                    Intake.setPower(intakeOn ? 0.56*multiple : 0);
                }
            }

            /*
             * Arm motor movement
             */
            if (gamepad2.dpad_up) {
                armTargetPosition = TOP_LEVEL_ENCODER_POSITION;
            } else if (gamepad2.dpad_left) {
                armTargetPosition = MIDDLE_LEVEL_POSITION;
            } else if (gamepad2.dpad_down) {
                armTargetPosition = BOTTOM_LEVEL_POSITION;
            } else if (gamepad2.x) {
                armTargetPosition = 1; // Start Position
                armPower = -0.4;
            } else if (gamepad2.left_trigger > 0) {
                armMotorSwitch.setTrue();
                ArmMotor.setPower(gamepad2.left_trigger);
            } else if (gamepad2.right_trigger > 0) {
                armMotorSwitch.setTrue();
                ArmMotor.setPower(-gamepad2.right_trigger);
            } else if (armMotorSwitch.checkAndTrigger()) {
                armPower = -1;
                ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                ArmMotor.setPower(0);
            }

            /*
             * Arm Run_TO_POSITION case
             */
            if (armTargetPosition != -1) {
                armMotorSwitch.setTrue();
                armTargetPosition = -1;
                ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ArmMotor.setPower(armPower);
            }

            /*
             * Arm accessory movement
             */
            if (gamepad2.a && debounces.checkAndUpdate("Twist")) {
                twistOn = !twistOn;
                Twist.setPosition(twistOn ? Twist_active : Twist_default);
            }

            /*
             * Duck motor movement
             */
            if (gamepad2.left_bumper || gamepad2.right_bumper) {
                int multiple = 1;
                if (gamepad2.right_bumper) {
                    multiple = -1;
                }
                if (startDuck == 0) {
                    duckMotorSwitch.setTrue();
                    startDuck = System.currentTimeMillis();
                    Duck_Wheel.setPower(0.56*multiple);
                } else if (System.currentTimeMillis() - startDuck > 750) {
                    Duck_Wheel.setPower(multiple);
                }
            } else if (duckMotorSwitch.checkAndTrigger()){
                startDuck = 0;
                Duck_Wheel.setPower(0);
            }
        }
    }
}
