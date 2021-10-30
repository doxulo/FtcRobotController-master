package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
@Disabled
public class MainAuto extends LinearOpMode {


    /** Global comments:
     * GamePad1 == For movements,
     * GamePad2 == Gadgets,
     */

    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
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

    /**
     * Initialize all motors that control the robot's accessories
     *
     * Duck_Wheel - Vertically propped motor that controls the duck dropper
     * Intake - Intake Motor
     * ArmMotor - Arm Motor
     */
    public DcMotor Duck_Wheel;

    public DcMotor Intake;

    public DcMotor ArmMotor;

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

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
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
        /*
        ArmMotor = initMotor(
                "Lift", // TODO: change name back to "ArmMotor"
                DcMotorSimple.Direction.FORWARD,
                DcMotor.RunMode.RUN_WITHOUT_ENCODER,
                DcMotor.ZeroPowerBehavior.FLOAT
        );
         */
        Duck_Wheel = initMotor(
                "Duck_Wheel",
                DcMotorSimple.Direction.FORWARD,
                DcMotor.RunMode.RUN_WITHOUT_ENCODER,
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        /**
         * Initialize trajectories
         */
        Trajectory strafeRight = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(24)
                .build();
        /**
        Trajectory d1 = drive.trajectoryBuilder(new Pose2d())
                .forward(48)
                .build();
        Trajectory d2 = drive.trajectoryBuilder(new Pose2d())
                .back(24)
                .build();
        Trajectory c1 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(-72, 20, Math.PI))
                .build();
        Trajectory b1 = drive.trajectoryBuilder(new Pose2d())
                .back(24)
                .build();
        Trajectory b2 = drive.trajectoryBuilder(new Pose2d())
                .back(24)
                .build();
         **/

        Pose2d startPosition = new Pose2d(-60, -60, 0);
        TrajectorySequence autonomousSequence = drive.trajectorySequenceBuilder(startPosition)
                .strafeRight(24)
                .turn(Math.toRadians(90))
                .forward(48)
                .turn(Math.PI/2)
                .back(24)
                .lineToLinearHeading(new Pose2d(-72, 20, Math.PI))
                .back(24)
                .turn(Math.PI/2)
                .back(48)
                .build();




        // TODO: gamepad2.rt = down, gamepad2.lt = up,
        // TODO: Change name on DriverHub from Lift to ArmMotor

        waitForStart();

        // Go to f1
        drive.followTrajectoryAsync(strafeRight);
        // Spin spiny thing
        Duck_Wheel.setPower(0.5D);
        wait(1000);
        Duck_Wheel.setPower(0);
        drive.followTrajectorySequenceAsync(autonomousSequence);
        /**
        // Turn left 90 degrees
        drive.turnAsync(Math.PI/2);
        // Go to d1
        drive.followTrajectoryAsync(d1); -
        // Turn left 90 degrees
        drive.turnAsync(Math.PI/2); -
        // Go to d2
        drive.followTrajectoryAsync(d2); -
        // Turn left 90 degrees and go to c1 at the same time -
        drive.followTrajectoryAsync(c1);
        // Go backward to b1 -
        drive.followTrajectory(b1);
        // Turn left 90 degrees -
        drive.turn(Math.PI/2);
        // Go backward to b2 -
        drive.followTrajectory(b2);
        // Strafe right to a2 -
        drive.followTrajectory(a2);
        **/
    }
}
