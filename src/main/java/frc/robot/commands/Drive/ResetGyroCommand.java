package frc.robot.commands.Drive;

import frc.robot.Constants;
import frc.robot.Constants.Drive;
import frc.robot.subsystems.Drive.DriveSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ResetGyroCommand extends Command {
    private final DriveSubsystem m_driveSubsystem;
    private double start_time;
    private boolean finished;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public ResetGyroCommand(DriveSubsystem driveSubsystem) {
        this.m_driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        start_time = Timer.getFPGATimestamp();
        finished = false;
    }

    @Override
    public void execute() {
        if(Timer.getFPGATimestamp() - start_time >= 1.0 ){
            System.out.println("AWWA");
            m_driveSubsystem.resetGyroOffset();
                end(false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        finished = true;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return finished;
    }
}
