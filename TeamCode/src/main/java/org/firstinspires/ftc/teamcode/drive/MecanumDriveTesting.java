package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Debounce;
import org.firstinspires.ftc.teamcode.util.DebounceObject;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.OldPIDController;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.util.Scheduler;
import org.firstinspires.ftc.teamcode.util.Switch;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;

@TeleOp
@Config
public class MecanumDriveTesting extends LinearOpMode {

    private enum LiftStates {
        LEVEL_1, LEVEL_2, LEVEL_3, RESET
    }

    public static double kP = 0.002;
    public static double kI = 0;
    public static double kD = 0;

    public static double targetVerticalOrientation = 20D;

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
    public DcMotor Duck_Wheel1;
    public DcMotor Duck_Wheel2;
    public DcMotor Intake;
    public DcMotor ArmMotor;
    public ColorSensor BoxSensor;
    public CRServo tapeExtension;
    public CRServo tapeVerticalOrientation;
    public CRServo tapeHorizontalOrientation;
    public Servo[] odometryServos = new Servo[3];

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
    public String formatRaw(int rawValue) {
        return String.format("%d", rawValue);
    }

    public String formatRate(float rate) {
        return String.format("%.3f", rate);
    }

    public String formatFloat(float rate) {
        return String.format("%.3f", rate);
    }
    /**
     * Main method that executes upon code run
     */
    ElapsedTime timer = new ElapsedTime();

    ModernRoboticsI2cGyro armGyro;
    ModernRoboticsI2cGyro tapeGyro;

    @Override
    public void runOpMode() {

        FtcDashboard dash = FtcDashboard.getInstance();

        tapeVerticalOrientation = hardwareMap.crservo.get("TapeVerticalOrientation");
        tapeHorizontalOrientation = hardwareMap.crservo.get("TapeHorizontialOrientation");
        tapeVerticalOrientation.setPower(.5);

        armGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "ArmGyro");
        tapeGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "TapeGyro");

        telemetry.log().add("Gyros Calibrating. Do Not Move!");
        armGyro.calibrate();

        timer.reset();
        while (!isStopRequested() && armGyro.isCalibrating())  {
            telemetry.addData("calibrating, ", "%f seconds passed", timer.seconds());
            telemetry.update();
            sleep(50);
        }

        tapeVerticalOrientation.setPower(0);
        tapeGyro.calibrate();
        while (!isStopRequested() && armGyro.isCalibrating())  {
            telemetry.addData("calibrating, ", "%f seconds passed", timer.seconds());
            telemetry.update();
            sleep(50);
        }

        telemetry.addLine("Done calibrating");
        telemetry.update();
        limitPower = 1 / limitPower;

        Debounce debounces = new Debounce(
                new DebounceObject("Duck", 750),
                new DebounceObject("Intake", 500),
                new DebounceObject("Arm", 500),
                new DebounceObject("Limit", 500),
                new DebounceObject("Gate", 500),
                new DebounceObject("Twist", 500),
                new DebounceObject("Integral", 500),
                new DebounceObject("Derivative", 500),
                new DebounceObject("Servo", 300),
                new DebounceObject("OdometryServo", 1000),
                new DebounceObject("Test.", 1000)
        );

        Switch duckMotorSwitch = new Switch(false);


        Scheduler scheduler = new Scheduler();

        PIDController controller = new PIDController(
                0.001,
                0,// 0.000001,// 0.000001, // TODO: Tune this
                0,// 0.05,
                new double[] {
                        0.05, -0.10
                },
                360,
                0,
                new double[] {
                        -0.15, 0.15, 25D
                });

        PIDController tapeController = new PIDController(
                0.02,
                0.0001,
                0.01,
                new double[] {
                        0, 0
                },
                360,
                0,
                new double[] {
                        -0.25, 0.25, 10D
                }
        );

        PIDController initTapeController = new PIDController(
                0.01,
                0,
                0,
                new double[] {
                        0, 0
                },
                360,
                0,
                new double[] {
                        -0.05, 0.005, 10D
                }
        );

        OldPIDController downController = new OldPIDController(
                0.004,
                0,
                0.0075,
                new double[] {
                        0.05, -0.10
                },
                1440,
                0);

        double[] twistPositions = new double[] {
                0.35D, 0.58D, 0.74D
        };

        double[] testPositions = new double[] {
                0D, 1D
        };

        double[] activeOdometryPosition = new double[] {
                0.3666D, 0.3666D, 0.3666D
        };

        double defaultPower = 0.56D;
        double targetHeading = 0D;
        double currentHorizontalOrientation = 1D;
        // double targetVerticalOrientation = 20D;

        long startDuck = 0;

        int lastLevel = 0;
        int currentLevel = 0;
        int headingOffset = 0;

        boolean intakeOn = false;
        boolean limitOn = true;
        boolean resetArmPower = false;
        boolean targetSet = false;
        boolean on = false;

        limit = limitPower;

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

        Duck_Wheel1 = initMotor(
                "Duck_Wheel1",
                DcMotorSimple.Direction.FORWARD,
                DcMotor.RunMode.RUN_WITHOUT_ENCODER,
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        Duck_Wheel2 = initMotor(
                "Duck_Wheel2",
                DcMotorSimple.Direction.FORWARD,
                DcMotor.RunMode.RUN_WITHOUT_ENCODER,
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        Twist = hardwareMap.servo.get("Twisty");
        BoxSensor = hardwareMap.colorSensor.get("Boxsensor");
        tapeExtension = hardwareMap.crservo.get("TapeExtension");

        odometryServos = new Servo[] {
                hardwareMap.servo.get("LeftOdometryServo"),
                hardwareMap.servo.get("FrontOdometryServo"),
                hardwareMap.servo.get("RightOdometryServo"),
        };


        Method setPowerMethod = null;

        try {
            setPowerMethod = Intake.getClass().getMethod("setPower", double.class);
        } catch (NoSuchMethodException e) {
            e.printStackTrace();
        }

        tapeVerticalOrientation.setPower(0);

        waitForStart();
        /*
        ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        */
        long lastTime = System.currentTimeMillis();

        while (true) {
            controller.kP = kP;
            controller.kI = kI;
            controller.kD = kD;

            long currentSystemTime = System.currentTimeMillis();

            int heading = armGyro.getHeading();
            int tapeGyroHeading = tapeGyro.getHeading();

            if (tapeGyroHeading > 280) {
                tapeGyroHeading = 0;
            }

            heading = heading > 350 ? 0 : heading;
            int redColor = BoxSensor.red();

            try {
                scheduler.checkAndExecute();
            } catch (InvocationTargetException e) {
                e.printStackTrace();
            } catch (IllegalAccessException e) {
                e.printStackTrace();
            }

            double power = 0;

            telemetry.addData("Arm Power: ", ArmMotor.getPower());
            telemetry.addData("Dt: ", currentSystemTime - lastTime);

            // telemetry.addData(String.format("Red: %d, Green: %d, Blue: %d", redColor, BoxSensor.green(), BoxSensor.blue()), "");
            telemetry.addData("Red: ", redColor);
            telemetry.addData("tape measurer heading: ", tapeGyroHeading);
            telemetry.addData("tape measure target heading: ", targetVerticalOrientation);
            telemetry.addData("Summation: ", tapeController.summation);
            telemetry.addData("arm heading: ", "%3d deg", heading);
            telemetry.addData("Arm target position: ", targetHeading);
            telemetry.addData("Twist position: ", Twist.getPosition());

            for (Servo s : odometryServos) {
                telemetry.addData("Odometry servo position: ", s.getPosition());
            }
            // telemetry.addData("integrated Z", "%3d", integratedZ);
            //telemetry.addLine()
            //        .addData("rawX", formatRaw(rawX))
            //        .addData("rawY", formatRaw(rawY))
            //        .addData("rawZ", formatRaw(rawZ));
            //telemetry.addLine().addData("z offset", zAxisOffset).addData("z coeff", zAxisScalingCoefficient);
            telemetry.update();
            lastTime = currentSystemTime;
            //drive train
            mecanum(-Math.pow(gamepad1.left_stick_y, 3D), Math.pow(gamepad1.left_stick_x, 3D), Math.pow(gamepad1.right_stick_x, 3D));

            if (gamepad1.y && debounces.checkAndUpdate("Limit")) {
                limit = (limitOn ? 1 : limitPower);
                limitOn = !limitOn;
            }

            if (gamepad1.b || gamepad1.a && redColor < 100) {
                if (debounces.checkAndUpdate("Intake")) {
                    int multiple = 1;
                    if (gamepad1.b) {
                        multiple = -1;
                    }

                    intakeOn = !intakeOn;
                    Intake.setPower(intakeOn ? multiple : 0);
                }
            }

            if (Intake.getPower() > 0 && redColor > 101) {
                // sleep(1200);
                scheduler.add(
                        setPowerMethod,
                        Intake,
                        0,
                        1000
                );

                // Intake.setPower(0);
            }

            if (gamepad1.dpad_up) {
                tapeExtension.setPower(-1);
            } else if (gamepad1.dpad_down) {
                tapeExtension.setPower(1);
            } else {
                tapeExtension.setPower(0);
            }

            if (gamepad1.y) {
                tapeVerticalOrientation.setPower(-1);
            } else if (gamepad1.right_stick_button) {
                tapeVerticalOrientation.setPower(1);
            } else {
                tapeVerticalOrientation.setPower(0);
            }

            if (gamepad1.left_trigger > 0) {
                // targetSet = false;
                // tapeVerticalOrientation.setPower(-gamepad1.left_trigger/2);
                targetVerticalOrientation += gamepad1.left_trigger/2;
            } else if (gamepad1.right_trigger > 0) {
                // targetSet = false;
                // tapeVerticalOrientation.setPower(gamepad1.right_trigger/2);
                targetVerticalOrientation -= gamepad1.right_trigger/2;
            }

            if (gamepad1.left_bumper) {
                currentHorizontalOrientation -= 0.0025;
            } else if (gamepad1.right_bumper) {
                currentHorizontalOrientation += 0.0025;
            }

            if (currentHorizontalOrientation > 1) {
                currentHorizontalOrientation = 1.5;
            } else if (currentHorizontalOrientation < -1) {
                currentHorizontalOrientation = -1.5;
            }

            if (targetVerticalOrientation < 0) {
                targetVerticalOrientation = 0;
            } else if (targetVerticalOrientation > 360) {
                targetVerticalOrientation = 360;
            }

            if (targetVerticalOrientation == tapeGyroHeading) {
                tapeController.pauseAndReset();
                tapeController.resume();
            }

            tapeHorizontalOrientation.setPower(currentHorizontalOrientation);
            tapeVerticalOrientation.setPower(-MathUtil.clamp(tapeController.calculate(Math.round(targetVerticalOrientation), tapeGyroHeading), -1, 1));
            telemetry.addData("Power sent: ", -MathUtil.clamp(tapeController.calculate(Math.round(targetVerticalOrientation), tapeGyroHeading), -1, 1));

            if (gamepad2.dpad_up) {
                currentLevel = 1;
                targetHeading = 162D;
            } else if (gamepad2.dpad_left) {
                currentLevel = 2;
                targetHeading = 210D;
            } else if (gamepad2.dpad_down) {
                currentLevel = 3;
                targetHeading = 240D;
            } else if (gamepad2.x) {
                currentLevel = 0;
                targetHeading = 0D;
            } else if (gamepad2.left_stick_button) {
                headingOffset = heading;
            } else if (gamepad2.left_trigger > 0) {
                targetHeading = -1;
                currentLevel = -1;
                ArmMotor.setPower(-gamepad2.left_trigger/3);
            } else if (gamepad2.right_trigger > 0) {
                targetHeading = -1;
                currentLevel = -1;
                ArmMotor.setPower(gamepad2.right_trigger/3);
            }  else if (resetArmPower) {
                resetArmPower = false;
                targetHeading = -1;
                currentLevel = 0;
                controller.pauseAndReset();
                ArmMotor.setPower(0);
            } else if (ArmMotor.getPower() != 0) {
                ArmMotor.setPower(0);
            }

            if (targetHeading != -1) {
                power = controller.calculate(targetHeading, heading-headingOffset);

                if (targetHeading == 0) {
                    power *= 0.75;
                }
                if (currentLevel != lastLevel) {
                    controller.pauseAndReset();
                    controller.resume();
                }


                lastLevel = currentLevel;
                resetArmPower = true;
                ArmMotor.setPower(power);
            }

            if (gamepad2.right_bumper) {

                if (startDuck == 0) {
                    duckMotorSwitch.setTrue();
                    startDuck = System.currentTimeMillis();
                    Duck_Wheel1.setPower(defaultPower);
                    Duck_Wheel2.setPower(-defaultPower);
                } else if (System.currentTimeMillis() - startDuck > 750) {
                    Duck_Wheel1.setPower(0.75);
                    Duck_Wheel2.setPower(-0.75);
                }
            } else if (duckMotorSwitch.check()){
                duckMotorSwitch.trigger();
                startDuck = 0;
                Duck_Wheel1.setPower(0);
                Duck_Wheel2.setPower(0);
            }


            if (debounces.check("Servo") && redColor < 85) {
                Twist.setPosition(twistPositions[0]);
            } else if (redColor > 86 && !gamepad2.y && !gamepad2.a && debounces.check("Servo")) {
                Twist.setPosition(twistPositions[1]);
            } else if (redColor > 5 && gamepad2.y && debounces.checkAndUpdate("Servo")) {
                Twist.setPosition(twistPositions[2]);
                // sleep(1000);
            } else if (redColor > 86 && gamepad2.a && debounces.checkAndUpdate("Servo")) {
                Twist.setPosition(twistPositions[0]);
                // sleep(650);
            }




//            if (opModeIsActive()) {
//                for (Servo odometryServo : odometryServos) {
//                    if (odometryServo.getPosition() != 0.01) {
//                        odometryServo.setPosition(0.01);
//                    }
//                }
            if (!opModeIsActive()) {
                for (int i = 0; i < activeOdometryPosition.length; i++) {
                    odometryServos[i].setPosition(activeOdometryPosition[i]);
                }
                sleep(100);
                break;
            }
        }

    }

}