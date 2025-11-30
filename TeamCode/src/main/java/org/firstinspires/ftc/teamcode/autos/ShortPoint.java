package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.autos.AutoUtils.scoreShortBallsPose;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.RobotContainer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;


/**
 * base on the blue alliance field
 * Using the TeleOP mode to measure the point and change the current one into correct one
 */
final class Positions {
    public static final Translation2d START_POINT = new Translation2d(0, 0);
    public static final Rotation2d START_FACING = Rotation2d.fromDegrees(0);
    public static final Translation2d SHOOTING_POINT = new Translation2d(-0.56, -0.94);
    public static final Rotation2d SHOOTING_FACING = Rotation2d.fromDegrees(-50);
    public static final Rotation2d INTAKE_FACING = Rotation2d.fromDegrees(-90);
    public static final Translation2d LINE_1_LEFT_BALL = new Translation2d(-0.73, -0.47);
    public static final Translation2d LINE_1_MID_BALL = new Translation2d(-0.77, -0.53);
    public static final Translation2d LINE_1_RIGHT_BALL = new Translation2d(-0.78, -0.67);
    public static final Translation2d LINE_2_LEFT_BALL = new Translation2d(-1.36, -0.47);
    public static final Translation2d LINE_2_MID_BALL = new Translation2d(-1.35, -0.51);
    public static final Translation2d LINE_2_RIGHT_BALL = new Translation2d(-1.36, -0.67);
    public static final Translation2d LINE_3_LEFT_BALL = new Translation2d(-1.70, -0.33);
    public static final Translation2d LINE_3_MID_BALL = new Translation2d(-1.73, -0.46);
    public static final Translation2d LINE_3_RIGHT_BALL = new Translation2d(-1.75, -0.52);
}


public class ShortPoint implements Auto{
    @Override
    public Command getAutonomousCommands(RobotContainer robotContainer) {
        final SequentialCommandGroup sequence = new SequentialCommandGroup();
        sequence.addRequirements(robotContainer.driveSubsystem,robotContainer.intakeSubsystem,robotContainer.rotSubsystem,robotContainer.shooterSubsystem, robotContainer.odometry);

        // Step 0: Reset Odometry
        sequence.addCommands(new InstantCommand(() -> robotContainer.driveSubsystem.setPose(new Pose2d())));

        // <-- Step 1: Score preloaded Balls and shoot -->
       sequence.addCommands(AutoUtils.driveToShortPoseAndShot(robotContainer));

        // <-- Step 2:  Intake and score the first three Balls -->
        Command driveToFirstLine = robotContainer.driveSubsystem.followPath(
               new Pose2d(scoreShortBallsPose.getTranslation(),Rotation2d.fromDegrees(40)),
                new Translation2d[]{new Translation2d(0.5,0.5)},
                new Pose2d(Positions.LINE_1_RIGHT_BALL,Rotation2d.fromDegrees(-90)),
                Rotation2d.fromDegrees(45),
                0.3
        );
        sequence.addCommands(driveToFirstLine);

        sequence.addCommands(AutoUtils.driveToIntakeContinuousLy(robotContainer));

        sequence.addCommands(AutoUtils.driveToShortPoseAndShot(robotContainer));


        // <-- Step 3:  Intake and score the Second three Balls -->
        Command driveToSecondLine = robotContainer.driveSubsystem.followPath(
                new Pose2d(scoreShortBallsPose.getTranslation(),Rotation2d.fromDegrees(40)),
                new Translation2d[]{new Translation2d(0.5,0.5)},
                new Pose2d(Positions.LINE_2_RIGHT_BALL,Rotation2d.fromDegrees(-90)),
                Rotation2d.fromDegrees(45),
                0.3
                );
        sequence.addCommands(driveToSecondLine);

        sequence.addCommands(AutoUtils.driveToIntakeContinuousLy(robotContainer));

        sequence.addCommands(AutoUtils.driveToShortPoseAndShot(robotContainer));



        return sequence;

    }
}
