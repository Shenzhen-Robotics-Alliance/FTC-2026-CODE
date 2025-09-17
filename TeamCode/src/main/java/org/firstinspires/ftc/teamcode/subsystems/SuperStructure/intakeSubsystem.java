package org.firstinspires.ftc.teamcode.subsystems.SuperStructure;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import edu.wpi.first.math.controller.PIDController;

public class intakeSubsystem extends SubsystemBase {
    private MotorEx intakeMotor;

    private PIDController velocityController;
    private PIDController posistionController;


    // encoder: 2000 PPR (tricks/evolution) × 13.7 = 27400 CPR (count/revolution)
    private static final double MOTOR_MAX_RPM = 482;
    private static final double ENCODER_CPR = 27400;


    //velocity PID
    private static final double VELOCITY_KP = 0.012;
    private static final double VELOCITY_KI = 0.0008;
    private static final double VELOCITY_KD = 0.0003;

    //position PID
    private static final double POSITION_KP = 0.003;
    private static final double POSITION_KI = 0.0001;
    private static final double POSITION_KD = 0.0001;

    public enum IntakeControlMode{
        VELOCITY_MODE,
        POSITION_MODE;
    }

    public enum IntakeState{
        STOPPED,
        INTAKING,
        OUTTAKING,
        HOLDING,
    }

    







}
