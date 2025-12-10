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
final class bluePositions {
    public static final Translation2d START_POINT = new Translation2d(0, 0);
    public static final Rotation2d START_FACING = Rotation2d.fromDegrees(0);
    public static final Translation2d SHOOTING_POINT = new Translation2d(0.77,-0.42);
    public static final Rotation2d SHOOTING_FACING = Rotation2d.fromDegrees(0);
    public static final Rotation2d INTAKE_FACING = Rotation2d.fromDegrees(0);
    public static final Translation2d LINE_1_LEFT_BALL = new Translation2d(0,-0.73);
    public static final Translation2d LINE_1_MID_BALL = new Translation2d(0.53,-0.77);
    public static final Translation2d LINE_1_RIGHT_BALL = new Translation2d(0.57,-0.79);

    //new
    public static final Translation2d LINE_1_ENDING = LINE_1_RIGHT_BALL.plus(new Translation2d(-0.7,0));

    public static final Translation2d LINE_2_LEFT_BALL = new Translation2d(0,-1.36);
    public static final Translation2d LINE_2_MID_BALL = new Translation2d(0.31,-1.35);
    public static final Translation2d LINE_2_RIGHT_BALL = new Translation2d( 0.57,-1.36);
    //new
    public static final Translation2d LINE_2_ENDING = LINE_2_RIGHT_BALL.plus(new Translation2d(-0.9,0));

    public static final Translation2d LINE_3_LEFT_BALL = new Translation2d(0,-1.70);
    public static final Translation2d LINE_3_MID_BALL = new Translation2d(0.46,-1.73);
    public static final Translation2d LINE_3_RIGHT_BALL = new Translation2d(0.52,-1.77);
    public static final Translation2d LINE_3_ENDING = LINE_3_RIGHT_BALL.plus(new Translation2d(-0.9,0));
    public static final Translation2d PARKING_POINT = new Translation2d(0.14,-1.00);
    public static final Rotation2d PARKING_FACING = Rotation2d.fromDegrees(-90);


}


public class blueAutoThreePlusNineBalls implements Auto{
    @Override
    public Command getAutonomousCommands(RobotContainer robotContainer) {
        final SequentialCommandGroup sequence = new SequentialCommandGroup();
        // Step 0: Reset Odometry
        sequence.addCommands(new InstantCommand(() -> robotContainer.driveSubsystem.setPose(new Pose2d())));

        // <-- Step 1: Score preloaded Balls and shoot -->
       sequence.addCommands(AutoUtils.preloadDriveToShortPoseAndShot(robotContainer,bluePositions.START_POINT));

        // <-- Step 2:  Intake and score the first three Balls -->
        Command driveToFirstLine = robotContainer.driveSubsystem.followPath(
                new Pose2d(scoreShortBallsPose.getTranslation(),Rotation2d.fromDegrees(0)),
                new Translation2d[]{new Translation2d(0.72,-0.78)},
                new Pose2d(bluePositions.LINE_1_RIGHT_BALL,Rotation2d.fromDegrees(90)),
                Rotation2d.fromDegrees(0),
                0.7
        );
        sequence.addCommands(driveToFirstLine
                .withTimeout(800));
        sequence.addCommands(AutoUtils.driveToIntakeFirstLineContinuousLy(robotContainer).withTimeout(1000));
        sequence.addCommands(AutoUtils.firstLineDriveToShortPoseAndShot(robotContainer, bluePositions.LINE_1_ENDING,1300));


        // <-- Step 3:  Intake and score the Second three Balls -->
        Command driveToSecondLine = robotContainer.driveSubsystem.followPath(
                new Pose2d(scoreShortBallsPose.getTranslation(),Rotation2d.fromDegrees(0)),
                new Translation2d[]{new Translation2d(1,-1.34)},
                new Pose2d(bluePositions.LINE_2_RIGHT_BALL,Rotation2d.fromDegrees(90)),
                Rotation2d.fromDegrees(0),
                0.7
        );

        sequence.addCommands(driveToSecondLine
                .withTimeout(1200));

        sequence.addCommands(AutoUtils.driveToIntakeSecondLineContinuousLy(robotContainer).withTimeout(1400));
        sequence.addCommands(AutoUtils.secondLineDriveToShortPoseAndShot(robotContainer,bluePositions.LINE_2_ENDING,2000));

        // -- Step 4: Back to Original Point -->
        Command backToOriginalPoint = robotContainer.driveSubsystem.followPath(
                new Pose2d(bluePositions.SHOOTING_POINT,Rotation2d.fromDegrees(90)),
                new Translation2d[]{},
                new Pose2d(bluePositions.START_POINT,Rotation2d.fromDegrees(0)),
                Rotation2d.fromDegrees(0),
                0.7
        );
        sequence.addCommands(backToOriginalPoint);

        return sequence;
    }
}
