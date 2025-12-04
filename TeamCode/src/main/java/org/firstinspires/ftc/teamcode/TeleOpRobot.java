package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.commands.drive.JoystickDriveFactory;
import org.firstinspires.ftc.teamcode.commands.shotCommands.ManualRotCommand;
import org.firstinspires.ftc.teamcode.utils.MapleJoystickDriveInput;

import java.util.HashMap;

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

    //State machine to make sure if AutoShootingMode
    private boolean isAutoShootingMode = false;

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

        // <-- pilot control the chassis -->
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

        new Trigger(() -> copilotGamePad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5)
                .toggleWhenActive(
                        new SelectCommand(
                                new HashMap<Object, Command>() {{
                                    put(false, robotContainer.shootCommand.fixShootFarContinuously());
                                    put(true, new InstantCommand());
                                }},
                                () -> isAutoShootingMode
                        ),
                        robotContainer.shootCommand.shootStop()
                );

     this.copilotGamePad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenActive(() -> isAutoShootingMode = false);

        this.copilotGamePad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenActive(() -> isAutoShootingMode = true);


        new Trigger(() -> pilotGamePad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5)
                       .toggleWhenActive(
                        robotContainer.intakeCommand.intakeContinuously(),
                        robotContainer.intakeSubsystem.enableStopMotor()
                );

            this.pilotGamePad.getGamepadButton(GamepadKeys.Button.X)
                .toggleWhenActive(
                        robotContainer.intakeCommand.outtakeContinuously(),
                        robotContainer.intakeSubsystem.enableStopMotor()
                );

        new Trigger(() -> pilotGamePad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5)
                .toggleWhenActive(
                        new SelectCommand(
                                new HashMap<Object, Command>() {{
                                    put(false, robotContainer.shootCommand.fixShootShortContinuously());
                                    put(true, robotContainer.shootCommand.shootAutoVelocity());
                                }},
                                () -> isAutoShootingMode
                        ),
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
