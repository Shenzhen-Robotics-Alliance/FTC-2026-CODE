//===========ShortPoint===============
package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.autos.AutoUtils.scoreShortBallsPose;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.RobotContainer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;


/**
 * base on the blue alliance field
 * Using the TeleOP mode to measure the point and change the current one into correct one
 */
final class Positions {
    public static final Translation2d START_POINT = new Translation2d(0, 0);
    public static final Rotation2d START_FACING = Rotation2d.fromDegrees(0);
    public static final Translation2d SHOOTING_POINT = new Translation2d(0.77,-0.42);
    public static final Rotation2d SHOOTING_FACING = Rotation2d.fromDegrees(0);
    public static final Rotation2d INTAKE_FACING = Rotation2d.fromDegrees(0);
    public static final Translation2d LINE_1_LEFT_BALL = new Translation2d(0,-0.73);
    public static final Translation2d LINE_1_MID_BALL = new Translation2d(0.53,-0.77);
    public static final Translation2d LINE_1_RIGHT_BALL = new Translation2d(0.57,-0.79);

    //new
    public static final Translation2d LINE_1_ENDING = new Translation2d(0.18,-0.74);

    public static final Translation2d LINE_2_LEFT_BALL = new Translation2d(0,-1.36);
    public static final Translation2d LINE_2_MID_BALL = new Translation2d(0.51,-1.35);
    public static final Translation2d LINE_2_RIGHT_BALL = new Translation2d( 0.46,-1.36);
    //new
    public static final Translation2d LINE_2_ENDING = new Translation2d(0.11,-1.36);

    public static final Translation2d LINE_3_LEFT_BALL = new Translation2d(0,-1.70);
    public static final Translation2d LINE_3_MID_BALL = new Translation2d(0.46,-1.73);
    public static final Translation2d LINE_3_RIGHT_BALL = new Translation2d(0.52,-1.75);
    public static final Translation2d LINE_3_ENDING = new Translation2d(-0.19,-1.94);

}


public class ShortPoint implements Auto{
    @Override
    public Command getAutonomousCommands(RobotContainer robotContainer) {
        final SequentialCommandGroup sequence = new SequentialCommandGroup();
        // Step 0: Reset Odometry
        sequence.addCommands(new InstantCommand(() -> robotContainer.driveSubsystem.setPose(new Pose2d())));

        // <-- Step 1: Score preloaded Balls and shoot -->
       sequence.addCommands(AutoUtils.driveToShortPoseAndShot(robotContainer,Positions.START_POINT));

        // <-- Step 2:  Intake and score the first three Balls -->
        Command driveToFirstLine = robotContainer.driveSubsystem.followPath(
                new Pose2d(scoreShortBallsPose.getTranslation(),Rotation2d.fromDegrees(0)),
                new Translation2d[]{new Translation2d(0.72,-0.78)},
                new Pose2d(Positions.LINE_1_RIGHT_BALL,Rotation2d.fromDegrees(90)),
                Rotation2d.fromDegrees(0),
                0.7
        );
        sequence.addCommands(driveToFirstLine
                .andThen(AutoUtils.driveToIntakeFirstLineContinuousLy(robotContainer)));


        sequence.addCommands(AutoUtils.driveToShortPoseAndShot(robotContainer, Positions.LINE_1_ENDING));


        // <-- Step 3:  Intake and score the Second three Balls -->
        Command driveToSecondLine = robotContainer.driveSubsystem.followPath(
                new Pose2d(scoreShortBallsPose.getTranslation(),Rotation2d.fromDegrees(0)),
                new Translation2d[]{new Translation2d(0.60,-1.34)},
                new Pose2d(Positions.LINE_2_RIGHT_BALL,Rotation2d.fromDegrees(90)),
                Rotation2d.fromDegrees(0),
                0.7);

        sequence.addCommands(driveToSecondLine
                .andThen(AutoUtils.driveToIntakeSecondLineContinuousLy(robotContainer)));


        sequence.addCommands(AutoUtils.driveToShortPoseAndShot(robotContainer,Positions.LINE_2_ENDING));
        // <-- Step 4:  Intake and score the Third three Balls -->

        Command driveToThirdLine = robotContainer.driveSubsystem.followPath(
                new Pose2d(scoreShortBallsPose.getTranslation(),Rotation2d.fromDegrees(0)),
                new Translation2d[]{new Translation2d(0.58,-1.99)},
                new Pose2d(Positions.LINE_3_RIGHT_BALL,Rotation2d.fromDegrees(90)),
                Rotation2d.fromDegrees(0),
                0.7);

        sequence.addCommands(driveToThirdLine
                .andThen(AutoUtils.driveToIntakeThirdLineContinuousLy(robotContainer)));


        sequence.addCommands(AutoUtils.driveToShortPoseAndShot(robotContainer,Positions.LINE_3_ENDING));



        return sequence;


    }
}
