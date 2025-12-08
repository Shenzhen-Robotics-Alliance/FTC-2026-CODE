package org.firstinspires.ftc.teamcode.utils.SuperStructure;

public class KalmanFilter {

    /**
     * Constructor for the Kalman Filter.
     *
     * @param Q Process noise covariance (How much we trust the physics/prediction).
     *          Higher Q = system is volatile/fast-moving.
     * @param R Measurement noise covariance (How much we trust the sensor/Limelight).
     *          Higher R = sensor is noisy, filter will smooth more (more lag).
     */
    public KalmanFilter(double Q, double R) {
        // TODO: Initialize state variables and covariance matrices here.
    }

    /**
     * Prediction Step (Time Update).
     * Extrapolates the state based on the physical model (e.g., x = x + v * dt).
     * This should be called in every loop cycle, regardless of whether a target is seen.
     *
     * @param dt The time elapsed since the last update (in seconds).
     */
    public void predict(double dt) {
        // TODO: Implement the prediction logic:
        // x_pred = x + v * dt
        // P_pred = F * P * F^T + Q
    }

    /**
     * Update Step (Measurement Update).
     * Corrects the predicted state using the actual sensor measurement.
     * This is only called when the Limelight sees a valid target.
     *
     * @param measurement The raw measurement from the sensor (e.g., Limelight tx).
     */
    public void update(double measurement) {
        // TODO: Implement the correction logic:
        // K = P * H^T * inv(H * P * H^T + R)
        // x = x + K * (measurement - x)
        // P = (I - K * H) * P
    }

    /**
     * Gets the filtered position estimate.
     *
     * @return The estimated position (e.g., smoothed tx).
     */
    public double getPositionEstimate() {
        // TODO: Return the state variable 'x'
        return 0.0;
    }

    /**
     * Gets the estimated velocity.
     * Useful for feedforward control (shooting while moving).
     *
     * @return The estimated velocity (e.g., d(tx)/dt).
     */
    public double getVelocityEstimate() {
        // TODO: Return the state variable 'v'
        return 0.0;
    }
}