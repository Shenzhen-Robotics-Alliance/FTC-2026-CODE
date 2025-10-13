package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import java.util.List;


/**
 * A subsystem that encapsulates all logic for the Limelight 3 camera.
 */
public class VisionSubsystem extends SubsystemBase {

    public final Limelight3A limelight;

    // These variables will store the latest data from the camera
    public boolean hasTarget = false;
    public double targetX = 0.0;
    public double targetY = 0.0;
    public int targetID = 0;
    private int pipelineIndex = 0;
    
    // PID Controller for target tracking
    public static double kP = 0.05;
    public static double kI = 0.1;
    public static double kD = 0.002;
    public static double kF = 0.0;
    private PIDFController pidfController = new PIDFController(kP, kI, kD, kF);
    
    // Additional data from Limelight
    private Pose3D botpose;
    private double captureLatency = 0.0;
    private double targetingLatency = 0.0;
    private double parseLatency = 0.0;
    private LLStatus status;
    private List<LLResultTypes.FiducialResult> fiducialResults;
    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public VisionSubsystem(HardwareMap hardwareMap) {
        // Initialize the hardware from the robot's configuration
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Configure and start the Limelight immediately
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();
        
        // Initialize runtime
        runtime.reset();
    }


    @Override
    public void periodic() {
        // Get the latest status and result from the Limelight
        status = limelight.getStatus();
        LLResult result = limelight.getLatestResult();

        if (result != null) {
            // Access general information
            botpose = result.getBotpose();
            captureLatency = result.getCaptureLatency();
            targetingLatency = result.getTargetingLatency();
            parseLatency = result.getParseLatency();

            if (result.isValid()) {
                hasTarget = true;
                targetX = result.getTx();
                targetY = result.getTy();


                // Access fiducial results
                fiducialResults = result.getFiducialResults();
                if (!fiducialResults.isEmpty()) {
                    // Safely get the ID of the first detected fiducial tag
                    targetID = fiducialResults.get(0).getFiducialId();
                }
            } else {
                hasTarget = false;
            }
        } else {
            // If no result is available, update the state
            hasTarget = false;
        }
    }


    public double getTargetX() {
        return targetX;
    }

    public double getTargetY() {
        return targetY;
    }

    public int getTargetID() {
        return targetID;
    }

    public boolean hasTarget(){
        return hasTarget;
    }
    

    /**
     * Calculate PID output for target tracking
     * @param target The target position (usually 0.0 for center)
     * @return PID output power
     */
    public double calculatePIDOutput(double target) {
        if (hasTarget) {
            double currentTx = targetX;
            return pidfController.calculate(currentTx, target);
        }
        return 0.0;
    }

    /**
     * Get the robot's pose from the Limelight
     * @return Pose3D object representing the robot's position
     */
    public Pose3D getBotpose() {
        return botpose;
    }

    /**
     * Get the capture latency
     * @return capture latency in milliseconds
     */
    public double getCaptureLatency() {
        return captureLatency;
    }

    /**
     * Get the targeting latency
     * @return targeting latency in milliseconds
     */
    public double getTargetingLatency() {
        return targetingLatency;
    }

    /**
     * Get the parse latency
     * @return parse latency in milliseconds
     */
    public double getParseLatency() {
        return parseLatency;
    }

    /**
     * Get the total latency (capture + targeting)
     * @return total latency in milliseconds
     */
    public double getTotalLatency() {
        return captureLatency + targetingLatency;
    }

    /**
     * Get the Limelight status
     * @return LLStatus object containing status information
     */
    public LLStatus getStatus() {
        return status;
    }

    /**
     * Get the fiducial results
     * @return List of FiducialResult objects
     */
    public List<LLResultTypes.FiducialResult> getFiducialResults() {
        return fiducialResults;
    }
    

    /**
     * Switches the active pipeline on the Limelight.
     * @param pipelineIndex The index of the pipeline to switch to (0-9).
     */
    public void setPipeline(int pipelineIndex) {
        limelight.pipelineSwitch(pipelineIndex);
    }

    /**
     * Stops the Limelight polling.
     */
    public void stop() {
        limelight.stop();
    }

    public void start(){limelight.start();}
}
