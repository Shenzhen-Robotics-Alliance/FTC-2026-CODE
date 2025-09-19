package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

/**
 * A subsystem that encapsulates all logic for the Limelight 3 camera.
 */
public class LimelightSubsystem extends SubsystemBase {

    public final Limelight3A limelight;

    // These variables will store the latest data from the camera
    public boolean hasTarget = false;
    public double targetX = 0.0;
    public static double targetY = 0.0;
    public int targetID = 0;
    private  int pipelineIndex = 0;

    public LimelightSubsystem(HardwareMap hardwareMap) {
        // Initialize the hardware from the robot's configuration
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Configure and start the Limelight immediately
        limelight.setPollRateHz(100);
        limelight.start();
    }

    @Override
    public void periodic() {
        // Get the latest result from the Limelight
        LLResult result = limelight.getLatestResult();

        // Check if the result is valid and has a target
        if (result.isValid() && !result.getFiducialResults().isEmpty()) {
            hasTarget = true;
            targetX = result.getTx();
            targetY = result.getTy();

            // Safely get the ID of the first detected fiducial tag
            targetID = result.getFiducialResults().get(0).getFiducialId();
        } else {
            // If no valid target is found, update the state
            hasTarget = false;
        }
    }

    public double getTargetX() {
        return targetX;
    }

    public static double getTargetY() {
        return targetY;
    }

    public int getTargetID() {
        return targetID;
    }

    public boolean hasTarget(){
        return hasTarget = true;
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
}