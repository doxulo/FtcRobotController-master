package org.firstinspires.ftc.teamcode.autonomous.state;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.Arm;
import org.firstinspires.ftc.teamcode.util.BarcodeDetector;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.atomic.AtomicInteger;


@Autonomous(name = "State Red Duckbox", group = "State", preselectTeleOp = "MAIN MECANUM DRIVE")
public class State_Red_Duckbox extends LinearOpMode {
    DcMotorEx LF;
    DcMotorEx RF;
    DcMotorEx RB;
    DcMotorEx LB;
    DcMotorEx Arm_Slides;
    DcMotorEx ArmMotor;
    DcMotorEx Intake;
    DcMotorEx Duck_Wheel1;
    Servo Twist;
    Servo BoxFlip;
    CRServo horizontalServo;
    ColorSensor BoxSensor;
    OpenCvWebcam webcam;
    ModernRoboticsI2cGyro armGyro;
    /*
        Declare variables
     */
    PIDController controller = new PIDController(
            0.01,
            0.000001,// 0.000001, // TODO: Tune this
            0.05,
            new double[] {
                    0.05, -0.10
            },
            360,
            0,
            new double[] {
                    -0.15, 0.15, 25D
            });

    public enum ArmStates {
        REST, LEVEL_1, LEVEL_2, LEVEL_3
    }

    ArmStates currentArmState;
    ModernRoboticsI2cGyro orientationGyro;
    IntegratingGyroscope orientationGyroParsed;

    /**
     * A method to initialize a motor
     *
     * @param motorName             The name of the motor
     * @param direction             The direction of the motor
     * @param runMode               The run mode of the motor
     * @param zeroPowerBehavior     The zeroPowerBehavior of the motor
     * @return                      The newly initialized motor
     */
    private DcMotorEx initMotor(
            String motorName,
            DcMotorSimple.Direction direction,
            DcMotor.RunMode runMode,
            DcMotor.ZeroPowerBehavior zeroPowerBehavior
    ) {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, motorName);
        motor.setDirection(direction);
        motor.setMode(runMode);
        motor.setZeroPowerBehavior(zeroPowerBehavior);

        return motor;
    }

    /**
     * Gets the current target Arm Degress
     *
     * @param currentArmStates      The current state of the arm
     * @return                      The target heading of the arm
     */
    public double getArmTargetDegrees(ArmStates currentArmStates) {
        double targetDegrees = 0D;
        switch (currentArmStates) {
            case REST:
                targetDegrees = 0D;
                break;
            case LEVEL_1:
                targetDegrees = 150D;
                break;
            case LEVEL_2:
                targetDegrees = 200D;
                break;
            case LEVEL_3:
                targetDegrees = 220D;
                break;
        }

        return targetDegrees;
    }

    /**
     * Main method
     *
     * @throws InterruptedException
     */
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        ArmMotor = initMotor(
                "ArmMotor", // TODO: change to ArmMotor
                DcMotorSimple.Direction.REVERSE,
                DcMotor.RunMode.RUN_WITHOUT_ENCODER,
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        Intake = initMotor(
                "Intake",
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

        Duck_Wheel1 = initMotor(
                "Duck_Wheel1",
                DcMotorSimple.Direction.FORWARD,
                DcMotor.RunMode.RUN_WITHOUT_ENCODER,
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        Twist = hardwareMap.servo.get("Twisty");
        BoxFlip = hardwareMap.servo.get("BoxFlip");
        BoxSensor = hardwareMap.colorSensor.get("Boxsensor");

        Servo leftOdometryServo = hardwareMap.servo.get("LeftOdometryServo");
        Servo rightOdometryServo = hardwareMap.servo.get("RightOdometryServo");
        Servo frontOdometryServo = hardwareMap.servo.get("FrontOdometryServo");

        horizontalServo = hardwareMap.crservo.get("TapeHorizontialOrientation");
        horizontalServo.setPower(0);

        double LEFT_POSITION = 0.85D;
        double RIGHT_POSITION = 0.13D;
        double FRONT_POSITION = 0D;

        leftOdometryServo.setPosition(LEFT_POSITION);
        rightOdometryServo.setPosition(RIGHT_POSITION);
        frontOdometryServo.setPosition(FRONT_POSITION);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        BarcodeDetector detector = new BarcodeDetector(telemetry);
        webcam.setPipeline(detector);
        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        Arm outtake = new Arm(new PIDController(
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
                }), ArmMotor, Arm_Slides, BoxFlip, 0);


        AtomicInteger currentTargetHeading = new AtomicInteger(0);

        drive.setPoseEstimate(new Pose2d(-35, -65, Math.toRadians(270)));
        TrajectorySequence top = drive.trajectorySequenceBuilder(new Pose2d(-35, -65, Math.toRadians(270)))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-62, -55, Math.toRadians(0)), Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(-62,-62, Math.toRadians(290)))
                .addTemporalMarker(() -> Duck_Wheel1.setPower(-0.4))
                .waitSeconds(5)
                .addTemporalMarker(() -> Duck_Wheel1.setPower(0))
                .lineToLinearHeading(new Pose2d(-62,-55, Math.toRadians(0)))
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(-58, -30, Math.toRadians(90)), Math.toRadians(90))
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    Intake.setPower(1);
                    outtake.setTargetPosition(Arm.ArmTargetPosition.AUTONOMOUS_LEVEL_1); // Level change
                })
                .splineToSplineHeading(new Pose2d(-27, -28, Math.toRadians(180)), Math.toRadians(0))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    Twist.setPosition(0.84D);
                })
                .waitSeconds(0.5)
                .splineToSplineHeading(new Pose2d(-46, -20, Math.toRadians(180)), Math.toRadians(180))
                .addTemporalMarker(() -> {
                    Twist.setPosition(0.6D);
                    Intake.setPower(-1);
                    outtake.setTargetPosition(Arm.ArmTargetPosition.LEVEL_0);
                })
                .splineToSplineHeading(new Pose2d(-58, -35, Math.toRadians(0)),Math.toRadians(270))
                .build();

        TrajectorySequence middle = drive.trajectorySequenceBuilder(new Pose2d(-35, -65, Math.toRadians(270)))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-62, -55, Math.toRadians(0)), Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(-62,-62, Math.toRadians(290)))
                .addTemporalMarker(() -> Duck_Wheel1.setPower(-0.4))
                .waitSeconds(5)
                .addTemporalMarker(() -> Duck_Wheel1.setPower(0))
                .lineToLinearHeading(new Pose2d(-62,-55, Math.toRadians(0)))
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(-58, -30, Math.toRadians(90)), Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    Intake.setPower(1);
                    outtake.setTargetPosition(Arm.ArmTargetPosition.AUTONOMOUS_LEVEL_2); // Level change
                })
                .splineToSplineHeading(new Pose2d(-31, -28, Math.toRadians(180)), Math.toRadians(0))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    Twist.setPosition(0.84D);
                })
                .waitSeconds(0.5)
                .splineToSplineHeading(new Pose2d(-46, -20, Math.toRadians(180)), Math.toRadians(180))
                .addTemporalMarker(() -> {
                    Twist.setPosition(0.6D);
                    Intake.setPower(-1);
                    outtake.setTargetPosition(Arm.ArmTargetPosition.LEVEL_0);
                })
                .splineToSplineHeading(new Pose2d(-58, -35, Math.toRadians(0)),Math.toRadians(270))
                .build();

        TrajectorySequence bottom = drive.trajectorySequenceBuilder(new Pose2d(-35, -65, Math.toRadians(270)))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-62, -55, Math.toRadians(0)), Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(-62,-62, Math.toRadians(290)))
                .addTemporalMarker(() -> Duck_Wheel1.setPower(-0.4))
                .waitSeconds(5)
                .addTemporalMarker(() -> Duck_Wheel1.setPower(0))
                .lineToLinearHeading(new Pose2d(-62,-55, Math.toRadians(0)))
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(-58, -30, Math.toRadians(90)), Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    Intake.setPower(1);
                    outtake.setTargetPosition(Arm.ArmTargetPosition.AUTONOMOUS_LEVEL_3); // Level change
                })
                .splineToSplineHeading(new Pose2d(-35, -28, Math.toRadians(180)), Math.toRadians(0))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    Twist.setPosition(0.84D);
                })
                .waitSeconds(0.5)
                .splineToSplineHeading(new Pose2d(-46, -20, Math.toRadians(180)), Math.toRadians(180))
                .addTemporalMarker(() -> {
                    Twist.setPosition(0.6D);
                    Intake.setPower(-1);
                    outtake.setTargetPosition(Arm.ArmTargetPosition.LEVEL_0);
                })
                .splineToSplineHeading(new Pose2d(-58, -35, Math.toRadians(0)),Math.toRadians(270))
                .build();

        waitForStart();
        drive.followTrajectorySequenceAsync(top);

        Twist.setPosition(0.6D);
        Arm_Slides.setPower(-0.1);
        while (drive.isBusy() && opModeIsActive()) {
            drive.update();
            outtake.update();
        }
    }
}
