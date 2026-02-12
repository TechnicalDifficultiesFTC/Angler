package org.firstinspires.ftc.teamcode.Main.Helpers;

/**
 * Singleton that lives inside Shooter.java
 */
public class ShooterTracker {
    private final double VELOCITY_DROP_THRESHOLD_PERCENT =
            Config.ShooterConstants.ShooterTracker.VELOCITY_DROP_THRESHOLD_PERCENTAGE;
    private static final int SAMPLE_BUFFER_SIZE = 10;

    private final double[] velocitySamples = new double[SAMPLE_BUFFER_SIZE];
    private int sampleIndex = 0;
    private int sampleCount = 0;
    private int shotsFired = 0;
    private boolean monitoring = false;
    private boolean shotDetectionCooldown = false;
    private long cooldownStartTime = 0;
    private double targetVelocityPercent = 0;
    private static final long COOLDOWN_MS =
            Config.ShooterConstants.ShooterTracker.BALL_FIRING_COOLDOWN_MS;

    public ShooterTracker() {}

    /**
     * Start monitoring for shots, only call when flywheel has reached target velocity.
     * @param targetVelocityPercent the velocity the flywheel is trying to maintain
     */
    public void startMonitoring(double targetVelocityPercent) {
        this.targetVelocityPercent = targetVelocityPercent;
        monitoring = true;
        shotsFired = 0;
        sampleIndex = 0;
        sampleCount = 0;
        shotDetectionCooldown = false;
    }

    public void stopMonitoring() {
        monitoring = false;
    }

    public void update(double currentVelocityPercent, double targetVelocityPercent) {
        this.targetVelocityPercent = targetVelocityPercent;
        if (!monitoring) return;

        // Handle cooldown after detecting a shot
        if (shotDetectionCooldown) {
            //Condition for ending shot cooldown
            if (System.currentTimeMillis() - cooldownStartTime > COOLDOWN_MS) {
                shotDetectionCooldown = false;
                sampleCount = 0; // Reset buffer after cooldown
                sampleIndex = 0;
            } else {
                return; // Skip detection during cooldown
            }
        }

        // Add sample to circular buffer
        velocitySamples[sampleIndex] = currentVelocityPercent;
        sampleIndex = (sampleIndex + 1) % SAMPLE_BUFFER_SIZE;
        if (sampleCount < SAMPLE_BUFFER_SIZE) {
            sampleCount++;
        }

        if (sampleCount < SAMPLE_BUFFER_SIZE) return;

        double oldAvg = getAverageOfRange(0, SAMPLE_BUFFER_SIZE / 2);
        double newAvg = getAverageOfRange(SAMPLE_BUFFER_SIZE / 2, SAMPLE_BUFFER_SIZE);

        double velocityDrop = oldAvg - newAvg;

        // Only detect shot if:
        // 1. Velocity dropped significantly
        // 2. Old average was near target (we were at intended speed, not transitioning)
        // 3. Flywheel was actually spinning
        boolean wasNearTarget = Math.abs(oldAvg - targetVelocityPercent) < 10;
        boolean significantDrop = velocityDrop >= VELOCITY_DROP_THRESHOLD_PERCENT;
        boolean wasSpinning = oldAvg > 20;

        if (significantDrop && wasNearTarget && wasSpinning) {
            shotsFired++;
            shotDetectionCooldown = true;
            cooldownStartTime = System.currentTimeMillis();
        }
    }

    private double getAverageOfRange(int startOffset, int endOffset) {
        double sum = 0;
        int count = endOffset - startOffset;
        for (int i = startOffset; i < endOffset; i++) {
            int actualIndex = (sampleIndex + i) % SAMPLE_BUFFER_SIZE;
            sum += velocitySamples[actualIndex];
        }
        return sum / count;
    }

    public boolean shotWasFired() { return shotsFired > 0; }
    public int getShotsFired() { return shotsFired; }
    public boolean isMonitoring() { return monitoring; }
    public void resetShotCount() { shotsFired = 0; }
}