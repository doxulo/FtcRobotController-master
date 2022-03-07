package org.firstinspires.ftc.teamcode.autonomous.state;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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
import org.firstinspires.ftc.teamcode.util.BarcodeDetector;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.atomic.AtomicInteger;


@Autonomous(name = "Blue Cycles", group = "State", preselectTeleOp = "MAIN MECANUM DRIVE")
public class State_Blue_Cycles extends LinearOpMode {
    DcMotorEx LF;
    DcMotorEx RF;
    DcMotorEx RB;
    DcMotorEx LB;
    DcMotorEx ArmMotor;
    DcMotorEx Intake;
    Servo Twist;
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



        LF = initMotor(
                "LF",
                DcMotorSimple.Direction.FORWARD,
                DcMotor.RunMode.RUN_USING_ENCODER,
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        RF = initMotor(
                "RF",
                DcMotorSimple.Direction.FORWARD,
                DcMotor.RunMode.RUN_USING_ENCODER,
                DcMotor.ZeroPowerBehavior.FLOAT

        );

        LB = initMotor(
                "LB",
                DcMotorSimple.Direction.FORWARD,
                DcMotor.RunMode.RUN_USING_ENCODER,
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        RB = initMotor(
                "RB",
                DcMotorSimple.Direction.REVERSE,
                DcMotor.RunMode.RUN_USING_ENCODER,
                DcMotor.ZeroPowerBehavior.FLOAT
        );

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

        Twist = hardwareMap.servo.get("Twisty");
        BoxSensor = hardwareMap.colorSensor.get("Boxsensor");

        Servo leftOdometryServo = hardwareMap.servo.get("LeftOdometryServo");
        Servo rightOdometryServo = hardwareMap.servo.get("RightOdometryServo");
        Servo frontOdometryServo = hardwareMap.servo.get("FrontOdometryServo");

        horizontalServo = hardwareMap.crservo.get("TapeHorizontialOrientation");
        horizontalServo.setPower(0);

        double LEFT_POSITION = 0.85D;
        double RIGHT_POSITION = 0.1D;
        double FRONT_POSITION = 0.65D;

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
                webcam.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        waitForStart();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        AtomicInteger currentTargetHeading = new AtomicInteger(0);
        drive.setPoseEstimate(new Pose2d(10, 65, Math.toRadians(90)));

        /*
            Trajectory for the Top level
         */

        TrajectorySequence top = drive.trajectorySequenceBuilder(new Pose2d(10, 65, Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(10, 60, Math.toRadians(60)))
                .waitSeconds(2)
                .splineToSplineHeading(new Pose2d(40, 65, Math.toRadians(0)), 0)
                .waitSeconds(2)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(10, 60, Math.toRadians(60)), Math.toRadians(250))
                .setReversed(false)
                .waitSeconds(2)
                .splineToSplineHeading(new Pose2d(40, 65, Math.toRadians(0)), 0)
                .setReversed(true)
                .waitSeconds(2)
                .splineToSplineHeading(new Pose2d(10, 60, Math.toRadians(60)), Math.toRadians(250))
                .waitSeconds(2)
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(40, 65, Math.toRadians(0)), 0)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(10, 60, Math.toRadians(60)), Math.toRadians(250))
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(40, 65, Math.toRadians(0)), 0)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(10, 60, Math.toRadians(60)), Math.toRadians(250))
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(40, 65, Math.toRadians(0)), 0)
                .build();

        TrajectorySequence top_Linear = drive.trajectorySequenceBuilder(new Pose2d(10, 65, Math.toRadians(90)))
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(10, 55, Math.toRadians(60)))
                .waitSeconds(0.1)
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(10,67, Math.toRadians(0)))
                .waitSeconds(0.1)
                .lineToLinearHeading(new Pose2d(40, 67, Math.toRadians(0)))
                .waitSeconds(0.1)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(10,67 , Math.toRadians(0)))
                .waitSeconds(0.1)
                .lineToLinearHeading(new Pose2d(10, 55, Math.toRadians(60)))
                .waitSeconds(0.1)
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(10,67, Math.toRadians(0)))
                .waitSeconds(0.1)
                .lineToLinearHeading(new Pose2d(40, 67, Math.toRadians(0)))
                .waitSeconds(0.1)
                .setReversed(true)
                .waitSeconds(0.1)
                .lineToLinearHeading(new Pose2d(10,67 , Math.toRadians(0)))
                .waitSeconds(0.1)
                .lineToLinearHeading(new Pose2d(10, 55, Math.toRadians(60)))
                .waitSeconds(0.1)
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(10,67, Math.toRadians(0)))
                .waitSeconds(0.1)
                .lineToLinearHeading(new Pose2d(40, 67, Math.toRadians(0)))
                .waitSeconds(0.1)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(10,67 , Math.toRadians(0)))
                .waitSeconds(0.1)
                .lineToLinearHeading(new Pose2d(10, 55, Math.toRadians(60)))
                .waitSeconds(0.1)
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(10,67, Math.toRadians(0)))
                .waitSeconds(0.1)
                .lineToLinearHeading(new Pose2d(40, 67, Math.toRadians(0)))
                .waitSeconds(0.1)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(10,67 , Math.toRadians(0)))
                .waitSeconds(0.1)
                .lineToLinearHeading(new Pose2d(10, 55, Math.toRadians(60)))
                .waitSeconds(0.1)
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(10,67, Math.toRadians(0)))
                .waitSeconds(0.1)
                .lineToLinearHeading(new Pose2d(40, 67, Math.toRadians(0)))
                .build();

        waitForStart();
            drive.followTrajectorySequenceAsync(top);
            while (drive.isBusy() && opModeIsActive()) {
                drive.update();
            }
    }
}
