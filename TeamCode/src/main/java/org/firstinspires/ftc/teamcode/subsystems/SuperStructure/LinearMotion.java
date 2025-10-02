package org.firstinspires.ftc.teamcode.subsystems.SuperStructure;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.constants.SystemConstants;
import org.firstinspires.ftc.teamcode.utils.SuperStructure.SimpleMechanism;

import edu.wpi.first.math.controller.PIDController;

public class LinearMotion implements SimpleMechanism, Subsystem {
    public final String name;

    private final DcMotor[] motors;
    private final boolean[] motorsReversed;
    private final DcMotor encoder;
    private final boolean encoderReversed;
    private final double maximumSpeed;
    private final PIDController controller;
    private final double kG, kV, kS;

    private double setPoint;

    public LinearMotion(
            String name,
            DcMotor[] motors, boolean[] motorsReversed,
            DcMotor encoder, boolean encoderReversed,
            double maximumSpeed,
            double kG,double kV,double kS, double kP
    ){
        this.name = name;

        this.motors = motors;
        this.motorsReversed = motorsReversed;
        this.encoder = encoder;
        this.encoderReversed = encoderReversed;
        this.maximumSpeed = maximumSpeed;
        this.controller = new PIDController(kP,0,0);
        this.kG = kG;
        this.kV = kV;
        this.kS = kS;

        for (DcMotor motor:motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        encoder.setDirection(DcMotorSimple.Direction.FORWARD);

        previousSetPoint = setPoint = 0; // robot must be in starting configuration
    }
    private double previousSetPoint;
    private double encoderZeroPosition = 0.0;

    public void periodic(){

        double currentPosition =(encoder.getCurrentPosition() - encoderZeroPosition) * (encoderReversed ? -1:1) / maximumSpeed;
        double desiredVelocity = (setPoint - previousSetPoint) * SystemConstants.ROBOT_UPDATE_RATE_HZ;

        previousSetPoint = setPoint;
        double feedForwardPower = desiredVelocity * kV
                + Math.signum(desiredVelocity) * kS
                + kG;
        double feedBackPower = controller.calculate(currentPosition, setPoint);

        runPower(feedForwardPower + feedBackPower);
    }

    public void runPower(double power){
        for (int i = 0; i < motors.length; i++) {
            motors[i].setPower((power) * (motorsReversed[i] ? -1:1));
        }
    }

    @Override
    public void goToPosition(double setPoint) {
        this.setPoint = setPoint;
    }

    public double getCurrentSetPoint() {
        return this.setPoint;
    }

    public void setMotorsStop(){
        for (int i = 0; i < motors.length; i++) {
            motors[i].setPower(0);
        }
    }
}
