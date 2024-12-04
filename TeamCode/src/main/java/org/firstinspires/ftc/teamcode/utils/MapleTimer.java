package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.teamcode.constants.SystemConstants;

public class MapleTimer {
    public static void wait(double seconds) {
        try {
            Thread.sleep((long)(seconds * 1000));
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public static double getMatchTimeSeconds() {
        final long timeMillis = System.currentTimeMillis() - SystemConstants.matchStartTimeMillis;
        return timeMillis / 1000.0;
    }

    private long startTimeNanos = System.nanoTime();
    public void reset() {
        startTimeNanos = System.nanoTime();
    }
    public double getTimeSeconds() {
        return (System.nanoTime() - startTimeNanos) / 1_000_000_000.0;
    }
}
