package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Light;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Axis;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.autonomous.R_DuckBox_Old;
import org.firstinspires.ftc.teamcode.util.Arm;
import org.firstinspires.ftc.teamcode.util.Debounce;
import org.firstinspires.ftc.teamcode.util.DebounceObject;
import org.firstinspires.ftc.teamcode.util.OldPIDController;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.util.Scheduler;
import org.firstinspires.ftc.teamcode.util.Switch;
import org.firstinspires.ftc.teamcode.util.MathUtil;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;

@TeleOp
@Config
public class MecanumDrive extends LinearOpMode {

    private enum LiftStates {
        LEVEL_0, LEVEL_1, LEVEL_2, LEVEL_3
    }

    private enum ExtensionStates {
        ARM_EXTENT, ARM_RETRACT, ARM_REST, ARM_SAFE_RETRACT, ARM_SAFE_EXTEND
    }


    public static double kP = 0.02;
    public static double kI = 0;
    public static double kD = 0.2;

    public static double point1 = -0.4;
    public static double point2 = 0.4;

    public static double offset = 0;

    public static boolean runArmUsingEncoders = false;

    public int LEVEL_3_TARGET_HEADING = 300;
    public int LEVEL_2_TARGET_HEADING = 400;
    public int LEVEL_1_TARGET_HEADING = 500;

    public static double position = 1D;


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
    public Servo BoxFlip;
    public DcMotor Duck_Wheel1;
    public DcMotor Intake;
    public DcMotor ArmMotor;
    public DcMotor Arm_Slides;
    public ColorSensor BoxSensor;
    public CRServo tapeExtension;
    public CRServo tapeExtension1;
    public CRServo tapeVerticalOrientation;
    public CRServo tapeHorizontalOrientation;
    public Servo[] odometryServos = new Servo[3];
    public RevBlinkinLedDriver Lights;
    public AnalogInput potentiometer;

    public BNO055IMU imu;

    public static double rest = 0.46D;
    public static double close = 0.89D;
    public static double end = 0.28D;

    double[] restingPositions = new double[] {
            0.35D,
            0.5D,
            0.1D
    };

    /**
     * Initialize all motors that control the robot's accessories
     * <p>
     * Duck_Wheel - Vertically propped motor that controls the duck dropper
     * Intake - Intake Motor
     * ArmMotor - Arm Motor
     */

    public double limit = 1.0D;
    public static double limitPower = 1.1D;
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

    private int getCorrectedPosition() {
        int currentPosition = ArmMotor.getCurrentPosition()/10;

        return Math.max(currentPosition, 0);
    }
    /**
     * Main method that executes upon code run
     */
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {

        FtcDashboard dash = FtcDashboard.getInstance();
        telemetry = dash.getTelemetry();

        telemetry.addLine("Calibrating... ");
        telemetry.update();

//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
//        parameters.loggingEnabled      = true;
//        parameters.loggingTag          = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        imu.initialize(parameters);


        tapeVerticalOrientation = hardwareMap.crservo.get("TapeVerticalOrientation");
        tapeHorizontalOrientation = hardwareMap.crservo.get("TapeHorizontialOrientation");

        // limitPower = 1 / limitPower;

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
                0.01,
                0,//0.000001,// 0.000001, // TODO: Tune this
                0,
                new double[] {
                        0.4, -0.4
                },
                600,
                0,
                new double[] {
                        -0.15, 0.15, 25D
                });

        PIDController tapeController = new PIDController(
                0.02,
                0,// 0.0001,
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
                0.48D, 0.58D, 0.76D
        };

        double[] activeOdometryPosition = new double[] {
                0.3666D, 0.3666D, 0.3666D
        };

        LiftStates currentArmState = LiftStates.LEVEL_0;
        ExtensionStates currentExtensionState = ExtensionStates.ARM_REST;
        LiftStates targetArmState = LiftStates.LEVEL_0;

        double defaultPower = 0.56D;
        double targetHeading = 0D;
        double currentHorizontalOrientation = 0.5D;
        double targetVerticalOrientation = 0D;

        long startDuck = 0;
        long startExtensionTime = 0;

        int lastLevel = 0;
        int currentLevel = 0;
        int headingOffset = 0;
        long lastArmRunUpdate = System.currentTimeMillis();

        boolean intakeOn = false;
        boolean limitOn = true;
        boolean resetArmPower = false;
        boolean targetSet = false;
        boolean on = false;
        boolean retractOn = false;
        boolean safeRetractOn = true;

        long extensionStartTime = System.currentTimeMillis();

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
                DcMotor.RunMode.RUN_WITHOUT_ENCODER,
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        Duck_Wheel1 = initMotor(
                "Duck_Wheel1",
                DcMotorSimple.Direction.FORWARD,
                DcMotor.RunMode.RUN_WITHOUT_ENCODER,
                DcMotor.ZeroPowerBehavior.FLOAT
        );
        Arm_Slides = initMotor(
                "Arm_Slides",
                DcMotorSimple.Direction.FORWARD,
                DcMotor.RunMode.RUN_WITHOUT_ENCODER,
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        Twist = hardwareMap.servo.get("Twisty");
        BoxFlip = hardwareMap.servo.get("BoxFlip");
        BoxSensor = hardwareMap.colorSensor.get("Boxsensor");
        tapeExtension = hardwareMap.crservo.get("TapeExtension");
        tapeExtension1 = hardwareMap.crservo.get("TapeExtension1");
        Lights = hardwareMap.get(RevBlinkinLedDriver.class, "Lights");

        odometryServos = new Servo[] {
                hardwareMap.servo.get("LeftOdometryServo"),
                hardwareMap.servo.get("FrontOdometryServo"),
                hardwareMap.servo.get("RightOdometryServo"),
        };


        double[] restingPositions = new double[] {
                0.7D,
                0.89D,
                0.5D
        };

        Method setPowerMethod = null;

        try {
            setPowerMethod = Intake.getClass().getMethod("setPower", double.class);
        } catch (NoSuchMethodException e) {
            e.printStackTrace();
        }

        telemetry.addLine("Ready to Start");
        telemetry.update();

        waitForStart();
        ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Arm outtakeArm = new Arm(controller, (DcMotorEx) ArmMotor, (DcMotorEx) Arm_Slides, BoxFlip, 1);
       /*
       ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       */
        long lastTime = System.currentTimeMillis();

        for (int i = 0; i < odometryServos.length; i++) {
            odometryServos[i].setPosition(restingPositions[i]);
        }


        while (true) {

            controller.kP = kP;
            controller.kI = kI;
            controller.kD = kD;

            controller.biasPoints = new double[] {
                    point1, point2
            };

            controller.offset = offset;



            if (System.currentTimeMillis() - lastArmRunUpdate > 1000) {
                lastArmRunUpdate = System.currentTimeMillis();
                if (runArmUsingEncoders) {
                    ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                } else {
                    ArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
            }
            //fancy code
            long currentSystemTime = System.currentTimeMillis();

            // int tapeGyroHeading = tapeGyro.getHeading();

            // if (tapeGyroHeading > 280) {
            //    tapeGyroHeading = 0;
            // }

            int redColor = BoxSensor.red();

            try {
                scheduler.checkAndExecute();
            } catch (InvocationTargetException e) {
                e.printStackTrace();
            } catch (IllegalAccessException e) {
                e.printStackTrace();
            }

            double power = 0;

            // Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


            telemetry.addData("Arm Power: ", ArmMotor.getPower());
            telemetry.addData("Dt: ", currentSystemTime - lastTime);
            // telemetry.addData(String.format("Red: %d, Green: %d, Blue: %d", redColor, BoxSensor.green(), BoxSensor.blue()), "");
            telemetry.addData("Red: ", redColor);
            // telemetry.addData("tape measurer heading: ", tapeGyroHeading);
            telemetry.addData("Summation: ", tapeController.summation);
            telemetry.addData("arm heading: ", ArmMotor.getCurrentPosition());
            telemetry.addData("Target arm heading: ", targetHeading);
//            telemetry.addData("First Axis Orientation: ", orientation.firstAngle);
//            telemetry.addData("Second Axis Orientation: ", orientation.secondAngle);
//            telemetry.addData("Third Axis Orientation: ", orientation.thirdAngle);


            // telemetry.addData("integrated Z", "%3d", integratedZ);
            //telemetry.addLine()
            //        .addData("rawX", formatRaw(rawX))
            //        .addData("rawY", formatRaw(rawY))
            //        .addData("rawZ", formatRaw(rawZ));
            //telemetry.addLine().addData("z offset", zAxisOffset).addData("z coeff", zAxisScalingCoefficient);
            telemetry.update();
            lastTime = currentSystemTime;
            //drive train
            mecanum(-Math.pow(gamepad1.left_stick_y, 1D), Math.pow(gamepad1.left_stick_x, 1D), Math.pow(gamepad1.right_stick_x, 1D));

            if (redColor > 100) {
                Lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIME);
            } else {
                Lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
            }

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
                        -1,
                        250
                );
                scheduler.add(
                        setPowerMethod,
                        Intake,
                        0,
                        1000
                );

                // Intake.setPower(0);
            }

            if (gamepad1.dpad_up) {
                tapeExtension.setPower(1);
                tapeExtension1.setPower(1);
            } else if (gamepad1.dpad_down) {
                tapeExtension.setPower(-1);
                tapeExtension1.setPower(-1);
            } else {
                tapeExtension.setPower(0);
                tapeExtension1.setPower(0);
            }

            if (gamepad1.left_trigger > 0) {
                currentHorizontalOrientation = gamepad1.left_trigger/10;
            } else if (gamepad1.right_trigger > 0) {
                currentHorizontalOrientation = -gamepad1.right_trigger/10;
            } else {
                currentHorizontalOrientation = 0;
            }

            if (gamepad1.left_bumper) {
                targetVerticalOrientation = -0.25;
            } else if (gamepad1.right_bumper) {
                targetVerticalOrientation = 0.25;
            } else {
                targetVerticalOrientation = 0;
            }
//
//            if (targetVerticalOrientation > 1) {
//                targetVerticalOrientation = 1;
//            } else if (targetVerticalOrientation < 0) {
//                targetVerticalOrientation = 0;
//            }

            tapeVerticalOrientation.setPower(targetVerticalOrientation);
            tapeHorizontalOrientation.setPower(currentHorizontalOrientation);

//            tapeVerticalOrientation.setPower(-MathUtil.clamp(tapeController.calculate(Math.round(targetVerticalOrientation), tapeGyroHeading), -1, 1));
//            telemetry.addData("Power sent: ", -MathUtil.clamp(tapeController.calculate(Math.round(targetVerticalOrientation), tapeGyroHeading), -1, 1));



            if (gamepad2.dpad_up) {
                currentLevel = 1;
                targetHeading = 390D;
            } else if (gamepad2.dpad_left) {
                currentLevel = 2;
                targetHeading = 425D;
            } else if (gamepad2.dpad_down) {
                currentLevel = 3;
                targetHeading = 500D;
            } else if (gamepad2.x) {
                currentLevel = 0;
                targetHeading = 0D;
                Intake.setPower(-0.5);

                scheduler.add(
                        setPowerMethod,
                        Intake,
                        0,
                        1
                );
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
                currentLevel = -1;
                controller.pauseAndReset();
                ArmMotor.setPower(0);
            } else if (ArmMotor.getPower() != 0) {
                ArmMotor.setPower(0);
            }


//            if (Math.abs(targetHeading - getCorrectedPosition()) < 10 && targetHeading > 200 && targetHeading < 450) {
//                Arm_Slides.setPower(-1);
//                BoxFlip.setPosition(0.3);
//            }
//
//            if (targetHeading == 0) {
//                Arm_Slides.setPower(1);
//
//                if (getCorrectedPosition() < 100) {
//                    BoxFlip.setPosition(0.88);
//                }
//            }


            if (targetHeading != -1) {
                power = controller.calculate(targetHeading, getCorrectedPosition());

                if (getCorrectedPosition() < 100) {
                    power = power/10;
                }
                telemetry.addData("Power: ", power);
                if (targetHeading == 0) {
                    power *= 0.5;
                }
                if (currentLevel != lastLevel) {
                    controller.pauseAndReset();
                    controller.resume();
                }

                lastLevel = currentLevel;
                resetArmPower = true;

                ArmMotor.setPower(power);
            }




//            if (gamepad2.dpad_up) {
//                outtakeArm.setTargetPosition(Arm.ArmTargetPosition.LEVEL_1);
//            } else if (gamepad2.dpad_left) {
//                outtakeArm.setTargetPosition(Arm.ArmTargetPosition.LEVEL_2);
//            } else if (gamepad2.dpad_down) {
//                outtakeArm.setTargetPosition(Arm.ArmTargetPosition.LEVEL_3);
//            } else if (gamepad2.x) {
//                outtakeArm.setTargetPosition(Arm.ArmTargetPosition.LEVEL_0);
//                Intake.setPower(-0.5);
//
//                scheduler.add(
//                        setPowerMethod,
//                        Intake,
//                        0,
//                        1
//                );
//            }
//
//            outtakeArm.update();

            // BoxFlip.setPosition(position); // 0.3, // 0.88

//            switch (currentArmState) {
//                case LEVEL_0:
//                    ArmMotor.setPower(controller.calculate(0, ArmMotor.getCurrentPosition()));
//
//                    if (gamepad2.dpad_up) {
//                        currentArmState = LiftStates.LEVEL_3;
//                    }
//                    break;
//                case LEVEL_3:
//
//                    ArmMotor.setPower(controller.calculate(LEVEL_3_TARGET_HEADING, ArmMotor.getCurrentPosition()));
//
//                    if (Math.abs(ArmMotor.getCurrentPosition() - LEVEL_3_TARGET_HEADING) < 50 && ArmMotor.getPower() < 0.25) {
//                        if (gamepad2.dpad_left || gamepad2.dpad_down || gamepad2.x) {
//                            if (!retractOn) {
//                                if (gamepad2.dpad_left) {
//                                    targetArmState = LiftStates.LEVEL_2;
//                                } else if (gamepad2.dpad_down) {
//                                    targetArmState = LiftStates.LEVEL_1;
//                                } else if (gamepad2.x) {
//                                    targetArmState = LiftStates.LEVEL_0;
//                                }
//                                currentExtensionState = ExtensionStates.ARM_RETRACT;
//                                retractOn = true;
//                                safeRetractOn = false;
//                                extensionStartTime = System.currentTimeMillis();
//                            }
//                        } else if (safeRetractOn) {
//                            retractOn = false;
//                            safeRetractOn = false;
//                            currentExtensionState = ExtensionStates.ARM_EXTENT;
//                        }
//
//                        if ((System.currentTimeMillis() - extensionStartTime) > 1000) {
//                            currentExtensionState = ExtensionStates.ARM_REST;
//                            currentArmState = targetArmState;
//                        }
//
//                        if (gamepad2.y) {
//                            Twist.setPosition(0.7D);
//                        }
//                    } else {
//                        safeRetractOn = true;
//                        currentExtensionState = ExtensionStates.ARM_SAFE_RETRACT;
//                    }
//
//                    break;
//            }
//
//            switch (currentExtensionState) {
//                case ARM_REST:
//                    Arm_Slides.setPower(0);
//                    break;
//                case ARM_EXTENT:
//                    Arm_Slides.setPower(1);
//                    break;
//                case ARM_RETRACT:
//                    Arm_Slides.setPower(-1);
//                    break;
//            }


            /*
            if (gamepad2.right_stick_button) {
                Arm_Slides.setPower(0);
            } else {
                if (gamepad2.right_stick_x > 0.4) {
                    Arm_Slides.setPower(gamepad2.right_stick_x);
                } else if (gamepad2.right_stick_x < -0.4){
                }
            }

             */

            Arm_Slides.setPower(-gamepad2.right_stick_x);

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
            } else {
                for (int i = 0; i < odometryServos.length; i++) {
                    odometryServos[i].setPosition(restingPositions[i]);
                }
            }
        }

    }

}

