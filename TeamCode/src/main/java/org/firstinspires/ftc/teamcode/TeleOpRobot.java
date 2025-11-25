package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.commands.drive.JoystickDriveFactory;
import org.firstinspires.ftc.teamcode.commands.shotCommands.ManualRotCommand;
import org.firstinspires.ftc.teamcode.utils.MapleJoystickDriveInput;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * robot during teleop stage'
 * here we bind all the commands to a key on the GamePad
 * */
public class TeleOpRobot extends Robot {
    private final RobotContainer robotContainer;
    private final GamepadEx pilotGamePad, copilotGamePad;

    private ManualRotCommand manualRotateCommand;

    private final Runnable calibrateOdometry;
    public TeleOpRobot(RobotContainer robotContainer, Gamepad pilotGamePad, Gamepad copilotGamePad) {
        super();
        this.robotContainer = robotContainer;
        this.pilotGamePad = new GamepadEx(pilotGamePad);
        this.copilotGamePad = new GamepadEx(copilotGamePad);

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

        //pilot use the Button start to reset the encoder
        this.pilotGamePad.getGamepadButton(GamepadKeys.Button.START).whenPressed(calibrateOdometry);

        //manual use LeftX control the Rotate
        manualRotateCommand = new ManualRotCommand(
                robotContainer.rotSubsystem,
                copilotGamePad::getLeftX
        );

        //copilot use the B to get the manual Rotate
        this.copilotGamePad.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(manualRotateCommand);

        //copilot press the A to control the auto Rotate
        this.copilotGamePad.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(robotContainer.autoRotCommand);

        //pilot use right bumper to control the outtake
        this.pilotGamePad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .toggleWhenActive(
                        robotContainer.intakeCommand.intakeContinuously(),
                        robotContainer.intakeCommand.stopIntake()
                );

        //pilot use left bumper to control intake
        this.pilotGamePad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .toggleWhenActive(
                        robotContainer.intakeCommand.outtakeContinuously(),
                        robotContainer.intakeCommand.stopIntake()
                );


        //pilot use the left trigger to control the shooter to shoot the short
        new Trigger(() -> pilotGamePad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5)
                .toggleWhenActive(
                        robotContainer.shootCommand.fixShootShortContinuously(),
                        robotContainer.shootCommand.shootStop()
                );

        //pilot use the right trigger to control the shooter to shoot the far one
        new Trigger(() -> pilotGamePad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5)
                .toggleWhenActive(
                robotContainer.shootCommand.fixShootFarContinuously(),
                        robotContainer.shootCommand.shootStop()
        );
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
