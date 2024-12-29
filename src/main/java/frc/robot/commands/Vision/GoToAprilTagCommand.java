// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import java.sql.Driver;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Vision.Limelight.LimelightObserver;
import frc.robot.subsystems.Vision.Limelight.LimelightSubsystem;

public class GoToAprilTagCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final LimelightSubsystem limelightSubsystem;
    private final DriveSubsystem driveSubsystem;
    private boolean targetVisible;
    private double[] botPose;
    private DoubleSupplier speedmodSupplier;

    public GoToAprilTagCommand(LimelightSubsystem limelightSubsystem, DriveSubsystem driveSubsystem,
            DoubleSupplier speedmodSupplier) {
        this.limelightSubsystem = limelightSubsystem;
        this.driveSubsystem = driveSubsystem;
        this.speedmodSupplier = speedmodSupplier;

        addRequirements(limelightSubsystem, driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        driveSubsystem.setSetpointArrivalX(1);
        driveSubsystem.setSetpointArrivalY(1);
        this.driveSubsystem.yawRotationPIDSetPoint(0);

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double distanceX = driveSubsystem.updatePose.getX();
        double distanceY = driveSubsystem.updatePose.getY();
        double yawRotationNeededInDegrees = driveSubsystem.updatePose.getRotation().getDegrees();
        // added -1 to stop 1 meter away from the apriltag
        driveSubsystem.setSetpointArrivalX(distanceX);
        driveSubsystem.setSetpointArrivalY(distanceY);
        driveSubsystem.setModules(this.driveSubsystem.calculateArrivalSpeedWithXPID(distanceX),
                this.driveSubsystem.calculateArrivalSpeedWithYPID(distanceY),
                this.driveSubsystem.calculateYawRotationInPID(yawRotationNeededInDegrees),
                this.speedmodSupplier.getAsDouble());

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}
