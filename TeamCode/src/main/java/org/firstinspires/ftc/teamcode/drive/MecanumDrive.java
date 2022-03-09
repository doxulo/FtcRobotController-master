package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.Arm;
import org.firstinspires.ftc.teamcode.util.AxesSigns;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.util.Debounce;
import org.firstinspires.ftc.teamcode.util.DebounceObject;
import org.firstinspires.ftc.teamcode.util.OldPIDController;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.util.Scheduler;
import org.firstinspires.ftc.teamcode.util.Slides;
import org.firstinspires.ftc.teamcode.util.Switch;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.concurrent.atomic.AtomicBoolean;

@TeleOp(group = "Control", name = "MAIN MECANUM DRIVE")
@Config
public class MecanumDrive extends LinearOpMode {

    private enum LiftStates {
        LEVEL_0, LEVEL_1, LEVEL_2, LEVEL_3
    }

    private enum ExtensionStates {
        ARM_EXTENT, ARM_RETRACT, ARM_REST, ARM_SAFE_RETRACT, ARM_SAFE_EXTEND
    }


    public static double kP = 0.005;
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
    Orientation lastAngles = new Orientation();

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
        // telemetry = dash.getTelemetry();

        // FIXME: imu added
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled = false;

        imu.initialize(parameters);

        // remap IMU
        BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

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
                new DebounceObject("Test.", 1000),
                new DebounceObject("ArmPosition", 250)
        );

        Switch duckMotorSwitch = new Switch(false);


        Scheduler scheduler = new Scheduler();

        PIDController controller = new PIDController(
                0.02,
                0,//0.000001,// 0.000001, // TODO: Tune this
                0.02,
                new double[] {
                        0.4, -0.4
                },
                600,
                0,
                new double[] {
                        -0.15, 0.15, 25D
                }, telemetry);

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

        PIDController slidesController = new PIDController(
                0.005,
                0,
                0.01,
                new double[] {
                        0, 0
                },
                700,
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
                0.53D, 0.63D, 0.84D
        };

        double[] activeOdometryPosition = new double[] {
                0.3666D, 0.3666D, 0.3666D
        };

        LiftStates currentArmState = LiftStates.LEVEL_0;
        ExtensionStates currentExtensionState = ExtensionStates.ARM_REST;
        LiftStates targetArmState = LiftStates.LEVEL_0;

        double defaultPower = 0.5D;
        double targetHeading = 0D;
        double currentHorizontalOrientation = 0.5D;
        double targetVerticalOrientation = 0D;

        long startDuck = 0;
        long startExtensionTime = 0;

        int lastLevel = 0;
        int currentLevel = 0;
        int headingOffset = 0;
        int extensionTarget = 0;
        long lastArmRunUpdate = System.currentTimeMillis();
        long scheduleRetractArm = 0;

        boolean intakeOn = false;
        boolean limitOn = true;
        boolean resetArmPower = false;
        boolean targetSet = false;
        boolean on = false;
        boolean retractOn = false;
        boolean safeRetractOn = true;
        boolean retractArm = false;
        boolean spinningIntake = true;
        boolean scheduleArmReturn = false;

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
        Method setRetractArmBoolean = null;

        Arm outtakeArm = new Arm(controller, (DcMotorEx) ArmMotor, (DcMotorEx) Arm_Slides, BoxFlip, 30, telemetry);

        Slides slides = new Slides( (DcMotorEx) Arm_Slides, (DcMotorEx) RF);

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
        Arm_Slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm_Slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       /*
       ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       */
        long lastTime = System.currentTimeMillis();

        for (int i = 0; i < odometryServos.length; i++) {
            odometryServos[i].setPosition(restingPositions[i]);
        }


        while (true) {

            slidesController.kP = kP;
            slidesController.kI = kI;
            slidesController.kD = kD;


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

            telemetry.addData("imu heading", lastAngles.firstAngle);
            telemetry.addData("Arm Power: ", ArmMotor.getPower());
            telemetry.addData("Dt: ", currentSystemTime - lastTime);
            telemetry.addData("Arm Slides Power: ", Arm_Slides.getPower());
            // telemetry.addData(String.format("Red: %d, Green: %d, Blue: %d", redColor, BoxSensor.green(), BoxSensor.blue()), "");
            telemetry.addData("Red: ", redColor);
            telemetry.addData("Slides Position: ", Arm_Slides.getCurrentPosition());
            // telemetry.addData("tape measurer heading: ", tapeGyroHeading);
            telemetry.addData("Summation: ", tapeController.summation);
            telemetry.addData("arm heading: ", ArmMotor.getCurrentPosition()/10);
            telemetry.addData("Target arm heading: ", targetHeading);
            telemetry.addData("BoxFLip: ", BoxFlip.getPosition());
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
                tapeExtension1.setPower(-1);
            } else if (gamepad1.dpad_down) {
                tapeExtension.setPower(-1);
                tapeExtension1.setPower(1);
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


            if (gamepad2.right_trigger > 0 && debounces.checkAndUpdate("ArmPosition")) {
                Arm.level1Position -= 1;
            } else if (gamepad2.left_trigger > 0 && debounces.checkAndUpdate("ArmPosition")) {
                Arm.level1Position +=  1;
            }
//            tapeVerticalOrientation.setPower(-MathUtil.clamp(tapeController.calculate(Math.round(targetVerticalOrientation), tapeGyroHeading), -1, 1));
//            telemetry.addData("Power sent: ", -MathUtil.clamp(tapeController.calculate(Math.round(targetVerticalOrientation), tapeGyroHeading), -1, 1));


//
//            if (gamepad2.dpad_up) {
//                currentLevel = 1;
//                targetHeading = 390D;
//            } else if (gamepad2.dpad_left) {
//                currentLevel = 2;
//                targetHeading = 425D;
//            } else if (gamepad2.dpad_down) {
//                currentLevel = 3;
//                targetHeading = 500D;
//            } else if (gamepad2.x) {
//                currentLevel = 0;
//                targetHeading = 0D;
//                Intake.setPower(-0.5);
//
//                scheduler.add(
//                        setPowerMethod,
//                        Intake,
//                        0,
//                        1
//                );
//            } else if (gamepad2.left_trigger > 0) {
//                targetHeading = -1;
//                currentLevel = -1;
//                ArmMotor.setPower(-gamepad2.left_trigger/3);
//            } else if (gamepad2.right_trigger > 0) {
//                targetHeading = -1;
//                currentLevel = -1;
//                ArmMotor.setPower(gamepad2.right_trigger/3);
//            }  else if (resetArmPower) {
//                resetArmPower = false;
//                targetHeading = -1;
//                currentLevel = -1;
//                controller.pauseAndReset();
//                ArmMotor.setPower(0);
//            } else if (ArmMotor.getPower() != 0) {
//                ArmMotor.setPower(0);
//            }


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


//            if (targetHeading != -1) {
//                power = controller.calculate(targetHeading, getCorrectedPosition());
//
//                if (getCorrectedPosition() < 100) {
//                    power = power/10;
//                }
//                telemetry.addData("Power: ", power);
//                if (targetHeading == 0) {
//                    power *= 0.5;
//                }
//                if (currentLevel != lastLevel) {
//                    controller.pauseAndReset();
//                    controller.resume();
//                }
//
//                lastLevel = currentLevel;
//                resetArmPower = true;
//
//                ArmMotor.setPower(power);
//            }




            if (gamepad2.dpad_up) {
                Intake.setPower(1);
                outtakeArm.setTargetPosition(Arm.ArmTargetPosition.LEVEL_1);

                scheduler.add(
                        setPowerMethod,
                        Intake,
                        0,
                        1000
                );

            } else if (gamepad2.dpad_left) {
                Intake.setPower(1);
                outtakeArm.setTargetPosition(Arm.ArmTargetPosition.LEVEL_2);

                scheduler.add(
                        setPowerMethod,
                        Intake,
                        0,
                        1000
                );

            } else if (gamepad2.dpad_down) {
                Intake.setPower(1);
                outtakeArm.setTargetPosition(Arm.ArmTargetPosition.LEVEL_3);

                scheduler.add(
                        setPowerMethod,
                        Intake,
                        0,
                        1000
                );

            } else if (gamepad2.dpad_right  && outtakeArm.targetPosition != Arm.ArmTargetPosition.LEVEL_0 && Arm_Slides.getCurrentPosition() < 200) {
                outtakeArm.setTargetPosition(Arm.ArmTargetPosition.LEVEL_0);
                Intake.setPower(-1);

                scheduler.add(
                        setPowerMethod,
                        Intake,
                        0,
                        1000
                );

            }

            outtakeArm.update();

            if (debounces.check("Servo") && redColor < 85) {
                Twist.setPosition(twistPositions[0]);
            } else if (redColor > 86 && !gamepad2.y && !gamepad2.a && !gamepad2.right_stick_button && debounces.check("Servo")) {
                Twist.setPosition(twistPositions[1]);
            } else if (redColor > 5 && ( gamepad2.y || gamepad2.right_stick_button ) && debounces.checkAndUpdate("Servo")) {
                Twist.setPosition(twistPositions[2]);

                scheduleRetractArm = System.currentTimeMillis();
            } else if (redColor > 86 && gamepad2.a && debounces.checkAndUpdate("Servo")) {
                Twist.setPosition(twistPositions[0]);
            }

            if (System.currentTimeMillis() - scheduleRetractArm > 500 && scheduleRetractArm != 0) {
                scheduleRetractArm = 0;
                retractArm = true;
            }

            if (retractArm) {

                Arm_Slides.setTargetPosition(0);
                Arm_Slides.setPower(1);
                Arm_Slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                scheduleArmReturn = true;
                retractArm = false;
            } else if (gamepad2.x) {
                if (outtakeArm.targetPosition == Arm.ArmTargetPosition.LEVEL_1) {
                    Arm_Slides.setTargetPosition(700);
                } else if (outtakeArm.targetPosition == Arm.ArmTargetPosition.LEVEL_2) {
                    Arm_Slides.setTargetPosition(350);
                } else {
                    Arm_Slides.setTargetPosition(100);
                }
                Arm_Slides.setPower(1);
                Arm_Slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else if (!Arm_Slides.isBusy() && gamepad2.right_stick_y == 0 && Arm_Slides.getCurrentPosition() < 300) {
                Arm_Slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                Arm_Slides.setPower(-0.3);
            } else if (!Arm_Slides.isBusy()) {
                Arm_Slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                Arm_Slides.setPower(-gamepad2.right_stick_y);
            }



            if (scheduleArmReturn && !ArmMotor.isBusy() && Arm_Slides.getCurrentPosition() < 50) {
                scheduleArmReturn = false;
                Intake.setPower(-1);

                scheduler.add(
                        setPowerMethod,
                        Intake,
                        0,
                        2000
                );

                outtakeArm.setTargetPosition(Arm.ArmTargetPosition.LEVEL_0);
            }

            if (gamepad2.right_bumper || gamepad2.left_bumper) {

                if (startDuck == 0) {
                    duckMotorSwitch.setTrue();
                    startDuck = System.currentTimeMillis();

                    if (gamepad2.left_bumper) {
                        Duck_Wheel1.setPower(-defaultPower);
                    } else {
                        Duck_Wheel1.setPower(defaultPower);
                    }
                } else if (System.currentTimeMillis() - startDuck > 750) {
                    if (gamepad2.left_bumper) {
                        Duck_Wheel1.setPower(-0.75);
                    } else {
                        Duck_Wheel1.setPower(0.75);
                    }
                }
            } else if (duckMotorSwitch.check()){
                duckMotorSwitch.trigger();
                startDuck = 0;
                Duck_Wheel1.setPower(0);
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

