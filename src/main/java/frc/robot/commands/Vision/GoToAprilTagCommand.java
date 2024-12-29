// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Vision.Limelight.LimelightObserver;
import frc.robot.subsystems.Vision.Limelight.LimelightSubsystem;

public class GoToAprilTagCommand extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final LimelightSubsystem limelightSubsystem;
    private final DriveSubsystem driveSubsystem;
    private boolean targetVisible;
    private double[] botPose;
    private DoubleSupplier speedmodSupplier;

    public GoToAprilTagCommand(LimelightSubsystem limelightSubsystem, DriveSubsystem driveSubsystem, DoubleSupplier speedmodSupplier) {
        this.limelightSubsystem = limelightSubsystem;
        this.driveSubsystem = driveSubsystem;
        this.speedmodSupplier = speedmodSupplier;

        addRequirements(limelightSubsystem, driveSubsystem);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.targetVisible = false;
        this.botPose = new double[6];
    }
 
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (targetVisible) {
            double distanceX = botPose[0];
            double distanceY = botPose[1];
            double yawRotationNeededInDegrees = botPose[5];
            // PID proportional gain ? 
            // double kP = 0.1; // ! DO NOT USE IN PRODUCTION, THIS VALUE HAS NOT BEEN TESTED -Nordy
            // double speedX =  distanceX * 0.1;
            // double speedY = distanceY * 0.1;
            
            this.driveSubsystem.yawRotationPIDSetPoint(yawRotationNeededInDegrees);
            driveSubsystem.setModules(x,y,this.driveSubsystem.calculateYawRotationInPID(yawRotationNeededInDegrees), this.speedmodSupplier.getAsDouble());
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }


}
