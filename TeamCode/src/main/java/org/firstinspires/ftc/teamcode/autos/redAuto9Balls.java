package org.firstinspires.ftc.teamcode.autos;

//===========ShortPoint===============

import static org.firstinspires.ftc.teamcode.autos.AutoUtils.RedScoreShortBallsPose;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.RobotContainer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * base on the red alliance field
 * Using the TeleOP mode to measure the point and change the current one into correct one
 */
final class redPositions {
    public static final Translation2d START_POINT = new Translation2d(0, 0);
    public static final Rotation2d START_FACING = Rotation2d.fromDegrees(0);
    public static final Translation2d FAR_SHOOTING_POINT = new Translation2d(0,0);
    public static final Rotation2d SHOOTING_FACING = Rotation2d.fromDegrees(0);
    public static final Rotation2d INTAKE_FACING = Rotation2d.fromDegrees(0);
    public static final Translation2d LINE_1_LEFT_BALL = new Translation2d(0,0.73);
    public static final Translation2d LINE_1_MID_BALL = new Translation2d(0.53,0.77);
    public static final Translation2d LINE_1_RIGHT_BALL = new Translation2d(0.57,0.69);

    //new
    public static final Translation2d LINE_1_ENDING = LINE_1_RIGHT_BALL.plus(new Translation2d(-0.7,0));

    public static final Translation2d LINE_2_LEFT_BALL = new Translation2d(0,1.36);
    public static final Translation2d LINE_2_MID_BALL = new Translation2d(0.31,1.35);
    public static final Translation2d LINE_2_RIGHT_BALL = new Translation2d( 0.47,1.39);
    //new
    public static final Translation2d LINE_2_ENDING = LINE_2_RIGHT_BALL.plus(new Translation2d(-0.7,0));

    public static final Translation2d LINE_3_LEFT_BALL = new Translation2d(0,1.70);
    public static final Translation2d LINE_3_MID_BALL = new Translation2d(0.46,1.73);
    public static final Translation2d LINE_3_RIGHT_BALL = new Translation2d(0.47,2.1);
    public static final Translation2d FAR_SHOOTING_LINE_3_RIGHT_BALL = new Translation2d(-0.63,0.72);
    public static final Translation2d FAR_SHOOTING_ENDING_LINE_3_ENDING = FAR_SHOOTING_LINE_3_RIGHT_BALL.plus(new Translation2d(-0.7,0));
    public static final Translation2d LINE_3_ENDING = LINE_3_RIGHT_BALL.plus(new Translation2d(-0.7,0));
    public static final Translation2d GATE_POINT = new Translation2d(-0.08,1.00);
    public static final Rotation2d PARKING_FACING = Rotation2d.fromDegrees(-90);

}

public class redAuto9Balls implements Auto{
    @Override
    public Command getAutonomousCommands(RobotContainer robotContainer) {
        final SequentialCommandGroup sequence = new SequentialCommandGroup();
        // Step 0: Reset Odometry
        sequence.addCommands(new InstantCommand(() -> robotContainer.driveSubsystem.setPose(new Pose2d())));

        // <-- Step 1: Score preloaded Balls and shoot -->
        sequence.addCommands(AutoUtils.RedPreloadDriveToShortPoseAndShot(robotContainer,redPositions.START_POINT));

        // <-- Step 2:  Intake and score the first three Balls -->
        Command driveToFirstLine = robotContainer.driveSubsystem.followPath(
                new Pose2d(RedScoreShortBallsPose.getTranslation(),Rotation2d.fromDegrees(0)),
                new Translation2d[]{new Translation2d(0.72, 0.78)},
                new Pose2d(redPositions.LINE_1_RIGHT_BALL,Rotation2d.fromDegrees(90)),
                Rotation2d.fromDegrees(0),
                0.75
        );
        sequence.addCommands(driveToFirstLine
                .withTimeout(1000));
        sequence.addCommands(AutoUtils.RedDriveToIntakeFirstLineContinuousLy(robotContainer).withTimeout(1200));
        sequence.addCommands(AutoUtils.RedFirstLineDriveToShortPoseAndShot(robotContainer, redPositions.LINE_1_ENDING,1300));


        // <-- Step 3:  Intake and score the Second three Balls -->
        Command driveToSecondLine = robotContainer.driveSubsystem.followPath(
                new Pose2d(RedScoreShortBallsPose.getTranslation(),Rotation2d.fromDegrees(0)),
                new Translation2d[]{new Translation2d(0.69,1.0)},
                new Pose2d(redPositions.LINE_2_RIGHT_BALL,Rotation2d.fromDegrees(90)),
                Rotation2d.fromDegrees(0),
                0.85
        );
        Command driveToGATE = robotContainer.driveSubsystem.followPath(
                new Pose2d(redPositions.LINE_2_ENDING,Rotation2d.fromDegrees(60)),
                new Translation2d[]{new Translation2d(0.31,1.36)},
                new Pose2d(redPositions.GATE_POINT,Rotation2d.fromDegrees(90)),
                Rotation2d.fromDegrees(0),
                0.9
        );

        sequence.addCommands(driveToSecondLine.withTimeout(1400));  //narrow
        sequence.addCommands(AutoUtils.RedDriveToFarPoseAndShot(robotContainer,redPositions.FAR_SHOOTING_POINT,2000).withTimeout(1400));

        sequence.addCommands(driveToGATE.withTimeout(1500));  //narrow
        sequence.addCommands(AutoUtils.RedSecondLineDriveToShortPoseAndShot(robotContainer,redPositions.GATE_POINT,1500));

        // -- Step 4: Back to ordinary Point -->
        Command backToOriginalPoint = robotContainer.driveSubsystem.followPath(
                new Pose2d(RedScoreShortBallsPose.getTranslation(),Rotation2d.fromDegrees(0)),
                new Translation2d[]{},
                new Pose2d(redPositions.START_POINT,Rotation2d.fromDegrees(90)),
                Rotation2d.fromRotations(0),
                0.8
        );

        sequence.addCommands(backToOriginalPoint);

        return sequence;



    }
}

