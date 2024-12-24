// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Drive.DriveSubsystem;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Drive.DriveCommand;
import frc.robot.commands.Drive.ResetGyroCommand;
import frc.robot.InRangeObserver;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Intake;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // ?
    // https://www.chiefdelphi.com/t/why-do-many-teams-put-a-m-in-front-of-many-variable-names/377126
    // ? this is why i put m_(variable name)
    // The robot's subsystems and commands are defined here...
    // private final VisionSubsystem m_VisionSubsystem;
    private final DriveSubsystem m_driveSubsystem;
    // private final ClimberSubsystem m_ClimberSubsystem;
    private final OI oi;

    // // Replace with CommandPS4Controller or CommandJoystick if needed
    // private final CommandXboxController m_driverController =
    // new CommandXboxController(OperatorConstants.kDriverControllerPort);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        this.m_driveSubsystem = new DriveSubsystem();

        // Disabled subsystems.
        // this.m_ClimberSubsystem = new ClimberSubsystem();
        // this.m_VisionSubsystem = new VisionSubsystem(null);

        this.oi = new OI();
        configureBindings();
        m_driveSubsystem.setAllModulesToZero();

        // change to shuffleBoard later if you want
        // Another option that allows you to specify the default auto by its name
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
    }

    // Vision is out.
    // public VisionSubsystem getVisionSubsystem() {
    // return m_VisionSubsystem;
    // }

    public DriveSubsystem getDriveSubsystem() {
        return m_driveSubsystem;
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        // m_driveSubsystem.setDefaultCommand(new DriveCommand(m_driveSubsystem,
        // oi.joystickLeft::getX, oi.joystickLeft::getY, oi.joystickRight::getX));

        m_driveSubsystem.setDefaultCommand(new DriveCommand(m_driveSubsystem,
                oi.xboxController::getLeftX,
                oi.xboxController::getLeftY,
                oi.xboxController::getRightX,
                oi.xboxController::getLeftTriggerAxis));

        // oi.commandXboxController.a()
        // .whileTrue(new ParallelCommandGroup(new ArmCommand(m_ArmSubsystem,
        // Arm.Stats.kIntakeAngle),
        // new IntakeNodeCommand(m_IntakeSubsystem, m_ShooterSubsystem)));

        // oi.commandXboxController.rightBumper()
        // .whileTrue(new ParallelCommandGroup(new
        // ShootVoltageCommand(m_ShooterSubsystem, 10),
        // new ArmCommand(m_ArmSubsystem, Arm.Stats.speakerAngle)));

        // the right bumper activates the shooter
        // oi.commandXboxController.a().whileTrue(new
        // ShootVoltageCommand(m_ShooterSubsystem, 12));
        // oi.commandXboxController.b().whileTrue(new
        // TestVoltageCommand(m_IntakeSubsystem,m_ShooterSubsystem,6));

        // if the a button is pressed, the climb will extend. once it's not, the climb
        // will retract.
        // oi.commandXboxController.a().whileTrue(new
        // OpenClimbCommand(m_ClimberSubsystem));
        // oi.commandXboxController.a().onFalse(new
        // CloseClimbCommand(m_ClimberSubsystem));

        // //if the right trigger is pressed the arm moves to the amp angle
        // oi.commandXboxController.rightTrigger().onTrue(new AmpPlace(m_ArmSubsystem));

        // //if the b button is pressed the shooter puts a note in amp
        // oi.commandXboxController.b().onTrue(new PlaceInAmp(m_ShooterSubsystem));

        // //if the x button is pressed the shooter will shoot (if the target is in
        // range)
        // oi.commandXboxController.x().and(inRangeSupplier).onTrue(new
        // ShooterCommand(m_ShooterSubsystem));
        // // //if the vision subsystem doesn't work then uncomment this (human opareted
        // shooting)
        // // oi.commandXboxController.x().onTrue(new
        // IntakePushNote(m_IntakeSubsystem).alongWith(new
        // SpeakerClose(m_ArmSubsystem)).andThen(new
        // ShooterCommand(m_ShooterSubsystem)));

        // //if the left trigger on the xbox is pressed the climbcommand will activate
        // oi.commandXboxController.leftTrigger().onTrue(new
        // IntakeSpinUp(m_IntakeSubsystem, false).alongWith(new
        // FloorIntake(m_ArmSubsystem)));
    }

    class InRangeSupplier implements BooleanSupplier {
        @Override
        public boolean getAsBoolean() {
            return b;
        }
    }

    InRangeSupplier inRangeSupplier = new InRangeSupplier();

    public InRangeSupplier getInRange() {
        return inRangeSupplier;
    }

    private boolean b = false;

}