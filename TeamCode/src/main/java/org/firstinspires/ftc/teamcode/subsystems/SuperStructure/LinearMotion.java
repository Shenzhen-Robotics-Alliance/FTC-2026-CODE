package org.firstinspires.ftc.teamcode.subsystems.SuperStructure;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.constants.SystemConstants;
import org.firstinspires.ftc.teamcode.utils.SuperStructure.SimpleMechanism;

import edu.wpi.first.math.controller.PIDController;

public class LinearMotion implements SimpleMechanism, Subsystem {
    public final String name;

    private final DcMotorEx[] motors;
    private final boolean[] motorsReversed;
    private final DcMotorEx encoder;
    private final boolean encoderReversed;
    private final double maximumSpeed;
    private final PIDController controller;
    private final double kG, kV, kS;

    private double setPoint;

    /**
     *  @param kG the percent motor power required to balance gravity, 0.0 if the linear motion stays still on its own
     *  @param kV the velocity gain in motor power / mechanism velocity
     *  (velocity is in position/second, where position is 0~1)
     *  @param kP the proportion gain in motor power / position error (where position is 0~1)
     *  @param kS the static gain, or the percent power required to move the mechanism
     * */

    public LinearMotion(
            String name,
            DcMotorEx[] motors, boolean[] motorsReversed,
            DcMotorEx encoder, boolean encoderReversed,
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

        if (this.encoder != null) {
            this.encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //  STOP_AND_RESET_ENCODER if need
            this.encoder.setDirection(encoderReversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        }
    }

    private double previousSetPoint;
    private double encoderZeroPosition = 0.0;

    public void setTargetVelocity(double targetVelocity) {
        this.setPoint = targetVelocity;
    }

    // Get current velocity from the encoder
    public double getCurrentVelocity() {
        if (encoder == null) {
            // using the first encoder
            return motors[0].getVelocity();
            // 或者抛出错误或返回0，取决于你的设计
        }
        // getVelocity() returns ticks/second
        return encoder.getVelocity(); // encoderReversed 已经在 setDirection 中处理
    }


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
