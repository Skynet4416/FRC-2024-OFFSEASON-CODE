package frc.robot.subsystems.Vision.Limelight;

public interface LimelightObserver {
    void onLimelightDataUpdate(boolean targetVisible, double[] botPose);
}