package org.firstinspires.ftc.teamcode.autonomous.league;

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
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.BarcodeDetector;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.atomic.AtomicInteger;


@Autonomous(group = "League")
public class R_Cycle extends LinearOpMode {

    final double CYCLES_NUMBER = 2;

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

    public void run() {
        while (opModeIsActive()) {
            double targetArmDegrees = getArmTargetDegrees(currentArmState);
            double power = 0;
            ArmMotor.setPower(power);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {


        armGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "ArmGyro");

        telemetry.log().add("Gyros Calibrating. Do Not Move!");
        armGyro.calibrate();

        while (!isStopRequested() && armGyro.isCalibrating())  {
            telemetry.addData("calibrating, ", "%f seconds passed");
            telemetry.update();
            sleep(50);
        }

        telemetry.addLine("Done calibrating");
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
        drive.setPoseEstimate(new Pose2d(13, -65, Math.toRadians(270)));

        //top

        TrajectorySequence lv1 = drive.trajectorySequenceBuilder(new Pose2d(13, -65, Math.toRadians(270)))
                .addDisplacementMarker(() -> {
                    currentTargetHeading.set(155);
                })
                .setReversed(true)
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(25))
                .splineToSplineHeading(new Pose2d(-7, -47, Math.toRadians(270)), Math.toRadians(160))
                .setReversed(false)
                .addDisplacementMarker(() -> {
                    sleep(200);
                    Twist.setPosition(0.8D);
                    sleep(500);
                    Twist.setPosition(0.48D);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    currentTargetHeading.set(1);
                    Intake.setPower(-0.5);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                    Intake.setPower(0.8);
                })
                .lineToLinearHeading(new Pose2d(10, -68, Math.toRadians(359)))
                .splineToSplineHeading(new Pose2d(45, -68, Math.toRadians(0)), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(25))
                .forward(1, SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(25))
                .addDisplacementMarker(() -> {
                    Twist.setPosition(0.58D);
                    Intake.setPower(-1);
                })
                //.lineToLinearHeading(new Pose2d(40, 70))
                .setReversed(true)
                .addDisplacementMarker(() -> {
                    currentTargetHeading.set(155);
                })
                .splineToSplineHeading(new Pose2d(-7, -43, Math.toRadians(270)), Math.toRadians(90))
                .setReversed(false)
                .addDisplacementMarker(() -> {
                    sleep(200);
                    Twist.setPosition(0.75D);
                    sleep(500);
                    Twist.setPosition(0.48D);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    currentTargetHeading.set(1);
                    Intake.setPower(-1);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                    Intake.setPower(0.8);
                })
                .lineToLinearHeading(new Pose2d(10, -68, Math.toRadians(359)))
                .lineToConstantHeading(new Vector2d(47, -68), SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(25))
                //.splineToSplineHeading(new Pose2d(40, -72, Math.toRadians(0)), Math.toRadians(0))
                .forward(2, SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(25))
                .addDisplacementMarker(() -> {
                    Twist.setPosition(0.58D);
                    Intake.setPower(-1);
                })
                .setReversed(true)
                .addDisplacementMarker(() -> {
                    currentTargetHeading.set(155);
                })
                .splineToSplineHeading(new Pose2d(-7, -43, Math.toRadians(270)), Math.toRadians(90))
                .setReversed(false)
                .addDisplacementMarker(() -> {
                    sleep(200);
                    Twist.setPosition(0.75D);
                    sleep(500);
                    Twist.setPosition(0.48D);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    currentTargetHeading.set(0);
                    Intake.setPower(-1);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                    Intake.setPower(0.8);
                })
                .lineToLinearHeading(new Pose2d(10, -68, Math.toRadians(359)))
                .splineToSplineHeading(new Pose2d(45, -68, Math.toRadians(0)), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(25))
                .forward(3, SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(25))
                .addDisplacementMarker(() -> {
                    Twist.setPosition(0.58D);
                    Intake.setPower(-0.8);
                })
                .addDisplacementMarker(() -> {
                    Intake.setPower(-1);
                })
                .build();
        //middle
        TrajectorySequence lv2 = drive.trajectorySequenceBuilder(new Pose2d(13, -65, Math.toRadians(270)))
                .addDisplacementMarker(() -> {
                    currentTargetHeading.set(193);
                })
                .setReversed(true)
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(25))
                .splineToSplineHeading(new Pose2d(-7, -50, Math.toRadians(270)), Math.toRadians(160))
                .setReversed(false)
                .addDisplacementMarker(() -> {
                    sleep(200);
                    Twist.setPosition(0.8D);
                    sleep(500);
                    Twist.setPosition(0.48D);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    currentTargetHeading.set(1);
                    Intake.setPower(-0.5);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                    Intake.setPower(0.8);
                })
                .lineToLinearHeading(new Pose2d(10, -68, Math.toRadians(359)))
                .splineToSplineHeading(new Pose2d(45, -68, Math.toRadians(0)), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(25))
                .forward(1, SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(25))
                .addDisplacementMarker(() -> {
                    Twist.setPosition(0.58D);
                    Intake.setPower(-1);
                })
                //.lineToLinearHeading(new Pose2d(40, 70))
                .setReversed(true)
                .addDisplacementMarker(() -> {
                    currentTargetHeading.set(155);
                })
                .splineToSplineHeading(new Pose2d(-7, -43, Math.toRadians(270)), Math.toRadians(90))
                .setReversed(false)
                .addDisplacementMarker(() -> {
                    sleep(200);
                    Twist.setPosition(0.75D);
                    sleep(500);
                    Twist.setPosition(0.48D);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    currentTargetHeading.set(1);
                    Intake.setPower(-1);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                    Intake.setPower(0.8);
                })
                .lineToLinearHeading(new Pose2d(10, -68, Math.toRadians(359)))
                .lineToConstantHeading(new Vector2d(47, -68), SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(25))
                //.splineToSplineHeading(new Pose2d(40, -72, Math.toRadians(0)), Math.toRadians(0))
                .forward(2, SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(25))
                .addDisplacementMarker(() -> {
                    Twist.setPosition(0.58D);
                    Intake.setPower(-1);
                })
                .setReversed(true)
                .addDisplacementMarker(() -> {
                    currentTargetHeading.set(155);
                })
                .splineToSplineHeading(new Pose2d(-7, -43, Math.toRadians(270)), Math.toRadians(90))
                .setReversed(false)
                .addDisplacementMarker(() -> {
                    sleep(200);
                    Twist.setPosition(0.75D);
                    sleep(500);
                    Twist.setPosition(0.48D);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    currentTargetHeading.set(0);
                    Intake.setPower(-1);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                    Intake.setPower(0.8);
                })
                .lineToLinearHeading(new Pose2d(10, -68, Math.toRadians(359)))
                .splineToSplineHeading(new Pose2d(45, -68, Math.toRadians(0)), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(25))
                .forward(3, SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(25))
                .addDisplacementMarker(() -> {
                    Twist.setPosition(0.58D);
                    Intake.setPower(-0.8);
                })
                .addDisplacementMarker(() -> {
                    Intake.setPower(-1);
                })
                .build();
        //bottom
        TrajectorySequence lv3 = drive.trajectorySequenceBuilder(new Pose2d(13, -65, Math.toRadians(270)))
                .addDisplacementMarker(() -> {
                    currentTargetHeading.set(220);
                })
                .setReversed(true)
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(25))
                .splineToSplineHeading(new Pose2d(-7, -54, Math.toRadians(270)), Math.toRadians(160))
                .setReversed(false)
                .addDisplacementMarker(() -> {
                    sleep(200);
                    Twist.setPosition(0.8D);
                    sleep(500);
                    Twist.setPosition(0.48D);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    currentTargetHeading.set(1);
                    Intake.setPower(-0.5);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                    Intake.setPower(0.8);
                })
                .lineToLinearHeading(new Pose2d(10, -68, Math.toRadians(359)))
                .splineToSplineHeading(new Pose2d(45, -68, Math.toRadians(0)), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(25))
                .forward(1, SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(25))
                .addDisplacementMarker(() -> {
                    Twist.setPosition(0.58D);
                    Intake.setPower(-1);
                })
                //.lineToLinearHeading(new Pose2d(40, 70))
                .setReversed(true)
                .addDisplacementMarker(() -> {
                    currentTargetHeading.set(155);
                })
                .splineToSplineHeading(new Pose2d(-7, -43, Math.toRadians(270)), Math.toRadians(90))
                .setReversed(false)
                .addDisplacementMarker(() -> {
                    sleep(200);
                    Twist.setPosition(0.75D);
                    sleep(500);
                    Twist.setPosition(0.48D);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    currentTargetHeading.set(1);
                    Intake.setPower(-1);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                    Intake.setPower(0.8);
                })
                .lineToLinearHeading(new Pose2d(10, -68, Math.toRadians(359)))
                .lineToConstantHeading(new Vector2d(47, -68), SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(25))
                //.splineToSplineHeading(new Pose2d(40, -72, Math.toRadians(0)), Math.toRadians(0))
                .forward(2, SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(25))
                .addDisplacementMarker(() -> {
                    Twist.setPosition(0.58D);
                    Intake.setPower(-1);
                })
                .setReversed(true)
                .addDisplacementMarker(() -> {
                    currentTargetHeading.set(155);
                })
                .splineToSplineHeading(new Pose2d(-7, -43, Math.toRadians(270)), Math.toRadians(90))
                .setReversed(false)
                .addDisplacementMarker(() -> {
                    sleep(200);
                    Twist.setPosition(0.75D);
                    sleep(500);
                    Twist.setPosition(0.48D);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    currentTargetHeading.set(0);
                    Intake.setPower(-1);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                    Intake.setPower(0.8);
                })
                .lineToLinearHeading(new Pose2d(10, -68, Math.toRadians(359)))
                .splineToSplineHeading(new Pose2d(45, -68, Math.toRadians(0)), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(25))
                .forward(3, SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(25))
                .addDisplacementMarker(() -> {
                    Twist.setPosition(0.58D);
                    Intake.setPower(-0.8);
                })
                .addDisplacementMarker(() -> {
                    Intake.setPower(-1);
                })
                .build();

        waitForStart();

        Twist.setPosition(0.58D);
        BarcodeDetector.Location barcode = detector.getLocation();
        webcam.stopStreaming();

        switch (barcode) {
            case LEFT:
                telemetry.addData("location: ", "left");
                break;
            case RIGHT:
                telemetry.addData("location: ", "right");
                break;
            case MIDDLE:
                telemetry.addData("location: ", "middle");
        }
        telemetry.update();

        if(isStopRequested()) return;

        switch (barcode) {
            case LEFT:
                drive.followTrajectorySequenceAsync(lv3);
                break;
            case RIGHT:
                drive.followTrajectorySequenceAsync(lv1);
                break;
            case MIDDLE:
                drive.followTrajectorySequenceAsync(lv2);
        }

        while (drive.isBusy() && opModeIsActive()) {
            drive.update();

            int heading = armGyro.getHeading();

            if (heading > 300) {
                heading = 0;
            }

            ArmMotor.setPower(controller.calculate(currentTargetHeading.get(), heading));
        }
    }
}
