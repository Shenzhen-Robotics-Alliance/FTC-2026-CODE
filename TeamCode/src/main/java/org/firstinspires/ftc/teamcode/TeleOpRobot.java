package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.commands.drive.JoystickDriveFactory;
import org.firstinspires.ftc.teamcode.commands.intakeCommands.IntakeContinueCommand;
import org.firstinspires.ftc.teamcode.commands.shotCommands.ManualRotCommand;
import org.firstinspires.ftc.teamcode.commands.shotCommands.ShootCommand;
import org.firstinspires.ftc.teamcode.utils.MapleJoystickDriveInput;
import com.arcrobotics.ftclib.command.Command;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * robot during teleop stage'
 * here we bind all the commands to a key on the GamePad
 * */
public class TeleOpRobot extends Robot {
    private final RobotContainer robotContainer;
    private final GamepadEx pilotGamePad, copilotGamePad;

    private ManualRotCommand manualRotateCommand;
    private IntakeContinueCommand intakeOn;
    private InstantCommand intakeOff;
    private IntakeContinueCommand intakeContinuous;

    private final Runnable calibrateOdometry;
    private com.arcrobotics.ftclib.command.Command activeSequence = null;
    public TeleOpRobot(RobotContainer robotContainer, Gamepad pilotGamePad, Gamepad copilotGamePad) {
        super();
        this.robotContainer = robotContainer;
        this.pilotGamePad = new GamepadEx(pilotGamePad);
        this.copilotGamePad = new GamepadEx(copilotGamePad);
        intakeOn = new IntakeContinueCommand(robotContainer.intakeSubsystem);
        intakeOff = new InstantCommand(
                () -> robotContainer.intakeSubsystem.setStopIntake(),
                robotContainer.intakeSubsystem
        );

        this.calibrateOdometry = () -> robotContainer.driveSubsystem.setPose(new Pose2d());
        calibrateOdometry.run();

        configureKeyBindings();
    }


    private void configureKeyBindings() {
        this.pilotGamePad.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenHeld(JoystickDriveFactory.joystickDrive(
                robotContainer.driveSubsystem,
                MapleJoystickDriveInput.leftHandedJoystick(pilotGamePad),
                () -> true));

        robotContainer.driveSubsystem.setDefaultCommand(JoystickDriveFactory.joystickDrive(
                robotContainer.driveSubsystem,
                MapleJoystickDriveInput.leftHandedJoystick(pilotGamePad),
                () -> true));

        this.pilotGamePad.getGamepadButton(GamepadKeys.Button.START).whenPressed(calibrateOdometry);

        manualRotateCommand = new ManualRotCommand(
                robotContainer.rotSubsystem,
                copilotGamePad::getLeftX
        );

        this.copilotGamePad.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(manualRotateCommand);

        this.copilotGamePad.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(robotContainer.autoRotCommand);

        //pilot use right bumper to control the intake
        this.pilotGamePad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .toggleWhenPressed(
                        new RunCommand(() -> robotContainer.intakeSubsystem.intake.runPower(0.8), robotContainer.intakeSubsystem),
                        new InstantCommand(robotContainer.intakeSubsystem.intake::setMotorsStop, robotContainer.intakeSubsystem)
                );

        //pilot use left bumper to control outtake
        this.pilotGamePad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .toggleWhenPressed(
                        new RunCommand(() -> robotContainer.intakeSubsystem.intake.runPower(-0.8), robotContainer.intakeSubsystem),
                        new InstantCommand(robotContainer.intakeSubsystem.intake::setMotorsStop, robotContainer.intakeSubsystem)
                );

        //pilot use the left trigger to control the shooter to shoot the short

        new Trigger(() -> pilotGamePad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5).whenActive(
                () -> {
                    activeSequence = robotContainer.shootCommand.shootShortContinuously();
                    activeSequence.schedule();
                });

        //pilot use the right trigger to control the shooter to shoot the far one
        new Trigger(() -> pilotGamePad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5).whenActive(
                () -> {
                    activeSequence = robotContainer.shootCommand.shootFarContinuously();
                    activeSequence.schedule();
                });


    }

    @Override
    public void reset() {
        super.reset();
        try {
            robotContainer.close();
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }
}
