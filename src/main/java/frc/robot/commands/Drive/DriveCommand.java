package frc.robot.commands.Drive;

import frc.robot.Constants;
import frc.robot.Constants.Drive;
import frc.robot.subsystems.Drive.DriveSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class DriveCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private final DoubleSupplier xDoubleSupplier;
    private final DoubleSupplier yDoubleSupplier;
    private final DoubleSupplier rotationDoubleSupplier;
    private boolean isConstantSpeed;
    private DoubleSupplier speedmodSupplier;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public DriveCommand(DriveSubsystem driveSubsystem, DoubleSupplier xValue, DoubleSupplier yValue,
            DoubleSupplier rotationValue) {
        this.driveSubsystem = driveSubsystem;
        this.xDoubleSupplier = xValue;
        this.yDoubleSupplier = yValue;
        this.rotationDoubleSupplier = rotationValue;
        this.isConstantSpeed = false;

        addRequirements(driveSubsystem);
    }

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public DriveCommand(DriveSubsystem driveSubsystem, DoubleSupplier xValue, DoubleSupplier yValue,
            DoubleSupplier rotationValue, DoubleSupplier speedmodValue) {
        this.driveSubsystem = driveSubsystem;
        this.xDoubleSupplier = xValue;
        this.yDoubleSupplier = yValue;
        this.rotationDoubleSupplier = rotationValue;
        this.speedmodSupplier = speedmodValue;
        
        addRequirements(driveSubsystem);
    }


    /**
     * Eliminates the drift from the joystick input
     */
    public double correctJoystickDrift(final double input) {
        return Math.max(0,Math.abs(input) - Constants.OI.kXboxcontrollerDrift) * Math.signum(input);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        driveSubsystem.setModules( 
                correctJoystickDrift(yDoubleSupplier.getAsDouble()) * Drive.Stats.kMaxVelocityMetersPerSecond,
                correctJoystickDrift(xDoubleSupplier.getAsDouble()) * Drive.Stats.kMaxVelocityMetersPerSecond,
                correctJoystickDrift(rotationDoubleSupplier.getAsDouble())
                        * Drive.Stats.kMaxAngularVelocityRadiansPerSecond, speedmodSupplier.getAsDouble());

    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.setModules(0, 0, 0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
