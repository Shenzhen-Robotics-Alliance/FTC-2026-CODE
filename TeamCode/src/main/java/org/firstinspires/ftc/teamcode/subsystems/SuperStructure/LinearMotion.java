package org.firstinspires.ftc.teamcode.subsystems.SuperStructure;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.constants.SystemConstants;
import org.firstinspires.ftc.teamcode.utils.SuperStructure.SimpleMechanism;

import java.util.Optional;
import java.util.function.BooleanSupplier;

public class LinearMotion implements SimpleMechanism, Subsystem {
    private String name;
    private DcMotorEx[] motors;
    private final boolean[] motorsReversed;
    private final DcMotorEx encoder;
    private final boolean encoderReversed;
    private final double maximumSpeed;
    private final PIDController controller;
    private final double kG, kV, kS;
    private double setPoint;
    private final Optional<BooleanSupplier> optionalLimitSwitch;
    private boolean calibrated = false;


    public LinearMotion(
            String name,
            DcMotor[] motors, boolean[] motorsReversed,
            DcMotorEx encoder, boolean encoderReversed,
            Optional<BooleanSupplier> optionalLimitSwitch,
            double maximumSpeed,
            double kG, double kV, double kP, double kS) {

        this.name = name;
        this.motorsReversed = motorsReversed;
        this.encoder = encoder;
        this.encoderReversed = encoderReversed;
        this.maximumSpeed = maximumSpeed;
        this.optionalLimitSwitch = optionalLimitSwitch;
        this.controller = new PIDController(kP,0,0);
        this.kG = kG;
        this.kV = kV;
        this.kS = kS;

        for(DcMotor motor: motors){
           motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
           motor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        encoder.setDirection(DcMotorSimple.Direction.FORWARD);


        previousSetPoint = setPoint = 0; // robot must be in starting configuration
    }

    private double previousSetPoint;
    private double encoderZeroPosition = 0.0;


    @Override
    public void periodic() {
        if (optionalLimitSwitch.isPresent() && optionalLimitSwitch.get().getAsBoolean())
            encoderZeroPosition = encoder.getCurrentPosition();
        if (optionalLimitSwitch.isPresent() && optionalLimitSwitch.get().getAsBoolean())
            calibrated = true;

        double currentPosition =(encoder.getCurrentPosition() - encoderZeroPosition) * (encoderReversed ? -1:1) / maximumSpeed;
        double desiredVelocity = (setPoint - previousSetPoint) * SystemConstants.ROBOT_UPDATE_RATE_HZ;
        if ((!calibrated && optionalLimitSwitch.isPresent())
                || (setPoint == 0 && currentPosition > 0.03)) {
            runPower(-0.4);
            return;
        }
        previousSetPoint = setPoint;
        double feedForwardPower = desiredVelocity * kV
                + Math.signum(desiredVelocity) * kS
                + kG;
        double feedBackPower = controller.calculate(currentPosition, setPoint);

        runPower(feedForwardPower + feedBackPower);

        SystemConstants.telemetry.addData(name + "CurrentPosition", currentPosition);
        SystemConstants.telemetry.addData(name + "DesiredVelocity", desiredVelocity);
        SystemConstants.telemetry.addData(name + "FFPower", feedForwardPower);
        SystemConstants.telemetry.addData(name + "FBPower", feedBackPower);
    }

    @Override
    public void goToPosition(double setPoint) {
        this.setPoint = setPoint;
    }
    public double getCurrentSetPoint() {
        return this.setPoint;
    }

    private void runPower(double power) {
        for (int i = 0; i < motors.length; i++)
            motors[i].setPower((power) * (motorsReversed[i] ? -1:1));
    }
}
