// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.Drive;

import java.util.Optional;

import com.kauailabs.navx.AHRSProtocol;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.Drive;
import frc.robot.subsystems.Drive.Swerve.*;

public class DriveSubsystem extends SubsystemBase {
     // https://github.com/CrossTheRoadElec/Phoenix6-Examples/tree/main/java/SwerveWithPathPlanner
     // https://github.com/CrossTheRoadElec/SwerveDriveExample/blob/main/src/main/java/frc/robot/CTRSwerve/CTRSwerveModule.java
     private final SwerveModule m_frontLeftModule;
     private final SwerveModule m_frontRightModule;
     private final SwerveModule m_backLeftModule;
     private final SwerveModule m_backRightModule;
     private final AHRS m_navX;
     private double m_navXoffset;
     private SwerveModulePosition[] m_modulePositions;
     private final SwerveDriveOdometry m_odometry;
     private final PIDController m_pidController;
     private ChassisSpeeds m_swerveSpeeds;
     private Pose2d m_lastPose;
     private Pose2d m_currentPose;
     private SwerveDrivePoseEstimator m_poseEstimator;
     private double m_targetAngle;
     private final Field2d field2d = new Field2d();

     public SwerveModule get_fl() {
          return m_frontLeftModule;
     }

     public SwerveModule get_fr() {
          return m_frontRightModule;
     }

     public SwerveModule get_bl() {
          return m_backLeftModule;
     }

     public SwerveModule get_br() {
          return m_backRightModule;
     }

     public void resetGyroOffset() {   
          // TODO: FIX (DOESN'T WORK)
          m_navXoffset = -getHeading();
          System.out.println("OFFSET IS " + m_navXoffset);
     }

     public DriveSubsystem() {
          this.m_frontLeftModule = new SwerveModule(
                    Drive.Motors.kFrontLeftDriveFalconCANID,
                    Drive.Motors.kFrontLeftSteerFalconCANID,
                    Drive.Encoders.kFrontLeftSteerEncoderCANID,
                    Drive.Stats.kFrontLeftModuleOffsetInDegrees, false);
          this.m_frontRightModule = new SwerveModule(
                    Drive.Motors.kFrontRightDriveFalconCANID,
                    Drive.Motors.kFrontRightSteerFalconCANID,
                    Drive.Encoders.kFrontRightSteerEncoderCANID,
                    Drive.Stats.kFrontRightModuleOffsetInDegrees, true);
          this.m_backLeftModule = new SwerveModule(
                    Drive.Motors.kBackLeftDriveFalconCANID,
                    Drive.Motors.kBackLeftSteerFalconCANID,
                    Drive.Encoders.kBackLeftSteerEncoderCANID,
                    Drive.Stats.kBackLeftModuleOffsetInDegrees, false);
          this.m_backRightModule = new SwerveModule(
                    Drive.Motors.kBackRightDriveFalconCANID,
                    Drive.Motors.kBackRightSteerFalconCANID,
                    Drive.Encoders.kBackRightSteerEncoderCANID,
                    Drive.Stats.kBackRightModuleOffsetInDegrees, false);

          m_navX = new AHRS();
          m_navXoffset = (double) m_navX.getCompassHeading();


          m_modulePositions = new SwerveModulePosition[] {
                    new SwerveModulePosition(m_frontLeftModule.getVelocityMetersPerSecond(),
                              m_frontLeftModule.getSteerAngle()),
                    new SwerveModulePosition(m_frontRightModule.getVelocityMetersPerSecond(),
                              m_frontRightModule.getSteerAngle()),
                    new SwerveModulePosition(m_backLeftModule.getVelocityMetersPerSecond(),
                              m_backLeftModule.getSteerAngle()),
                    new SwerveModulePosition(m_backRightModule.getVelocityMetersPerSecond(),
                              m_backRightModule.getSteerAngle())
          };

          m_odometry = new SwerveDriveOdometry(Drive.Stats.kinematics, Rotation2d.fromDegrees(getHeading()),
                    m_modulePositions);

          m_swerveSpeeds = new ChassisSpeeds(0, 0, 0);

          m_currentPose = m_odometry.getPoseMeters(); // todo needs to take the position from vision
          m_pidController = new PIDController(Drive.PID.kP, Drive.PID.kI, Drive.PID.kD);
          m_pidController.enableContinuousInput(0, 360);
          m_poseEstimator = new SwerveDrivePoseEstimator(Drive.Stats.kinematics, getGyroAngleInRotation2d(),
                    m_modulePositions, m_currentPose);
          m_lastPose = m_poseEstimator.getEstimatedPosition();
          m_targetAngle = 90;

          SmartDashboard.putNumber("Turn To angle I", Drive.PID.kI);
          SmartDashboard.putNumber("Turn To angle P", Drive.PID.kP);

     }

     /**
      * Sets the state of all of the swerve modules
      * 
      * @param moduleState
      *                    WPILib's SwerveModuleState library
      */
     public void setModulesStates(SwerveModuleState[] moduleState) {
          m_frontLeftModule.setModuleState(moduleState[0]);
          m_frontRightModule.setModuleState(moduleState[1]);
          m_backLeftModule.setModuleState(moduleState[2]);
          m_backRightModule.setModuleState(moduleState[3]);
     }

     /**
      * this is a function to send to the auto, if we integrate the proper vision
      * stuff from Yotam's branch it would replace this
      * 
      * @return
      */
     public SwerveDrivePoseEstimator getPoseEstimator() {
          return this.m_poseEstimator;
     }

     /**
      * returns the odometry of the drive subsystem
      * 
      * @return
      */
     public SwerveDriveOdometry getOdometry() {
          return m_odometry;
     }

     /**
      * this is a function that recives the current position from the auto and
      * updates the odometry
      */
     public void resetOdometry(Pose2d currentPose) {
          m_odometry.resetPosition(m_navX.getRotation2d(), m_modulePositions, currentPose);
     }

     /**
      * this is a function to send to the auto, if we integrate the proper vision
      * stuff from Yotam's branch it would replace this
      */
     public Pose2d getCurrentPose() {
          return m_currentPose;
     }

     /**
      * this is a function to send to the auto, if we integrate the proper vision
      * stuff from Yotam's branch it would replace this
      */
     public ChassisSpeeds getSwerveSpeeds() {
          return m_swerveSpeeds;
     }

     /**
      * Returns the field oriented corrected velocity for a target velocity
      * 
      * @param targetVelocityX
      *                        The target X velocity (Meters Per Second)
      * @param targetVelocityY
      *                        The target Y velocity (Meters Per Second)
      */
     public double getVelocityFieldOriented_X(double targetVelocityX, double targetVelocityY) {
          double offsetAngle = getGyroAngleInRotation2d().getDegrees() - Drive.Stats.fieldHeadingOffset;
          double corrected_velocity = targetVelocityX * Math.cos(Math.toRadians(offsetAngle))
                    - targetVelocityY * Math.sin(Math.toRadians(offsetAngle));
          return corrected_velocity;
     }

     /**
      * Returns the field oriented corrected velocity for a target velocity
      * 
      * @param targetVelocityX
      *                        The target X velocity (Meters Per Second)
      * @param targetVelocityY
      *                        The target Y velocity (Meters Per Second)
      */
     public double getVelocityFieldOriented_Y(double targetVelocityX, double targetVelocityY) {
          double offsetAngle = getGyroAngleInRotation2d().getDegrees() - Drive.Stats.fieldHeadingOffset;
          double corrected_velocity = targetVelocityX * Math.sin(Math.toRadians(offsetAngle))
                    + targetVelocityY * Math.cos(Math.toRadians(offsetAngle));
          return corrected_velocity;
     }

     /**
      * Sets the Speed / Angle / Stats of all of the modules
      * 
      * @param xVelocityMps
      *                            The X velocity (Meters Per Second)
      * @param yVelocityMps
      *                            The Y velocity (Meters Per Second)
      * @param rotationVelocityRps
      *                            Rotation velocity (Radians Per Second)
      */
     public void setModules(double xVelocityMps, double yVelocityMps, double rotationVelocityRps, double speedMode) {
          // TODO oriented to object on field
          final double slowFactor = 8;
          double speedDivisor = 1 * (1-speedMode) + slowFactor * speedMode;
                    
          xVelocityMps /= speedDivisor;
          yVelocityMps /= speedDivisor;
          rotationVelocityRps /= speedDivisor;

          double xVelocityMpsFieldOriented = getVelocityFieldOriented_X(xVelocityMps, yVelocityMps);
          double yVelocityMpsFieldOriented = getVelocityFieldOriented_Y(xVelocityMps, yVelocityMps);

          boolean correctAngle = true;
          if(Math.abs(xVelocityMps) > 0 || Math.abs(yVelocityMps) > 0 || Math.abs(rotationVelocityRps) > 0){
               if (correctAngle) {
                    m_targetAngle += Units.radiansToDegrees(rotationVelocityRps)*1.5;
                    this.m_swerveSpeeds = new ChassisSpeeds(xVelocityMpsFieldOriented, yVelocityMpsFieldOriented,
                              Math.abs(this.getGyroAngleInRotation2d().getDegrees() - m_targetAngle) > Drive.PID.kThreshold ? -m_pidController.calculate(this.getGyroAngleInRotation2d().getDegrees()) : 0);
                    m_pidController.setSetpoint(m_targetAngle-90);
               } else {
                    this.m_swerveSpeeds = new ChassisSpeeds(xVelocityMpsFieldOriented, yVelocityMpsFieldOriented,
                              -rotationVelocityRps * 1.2);
               }
          } else {
               this.m_swerveSpeeds = new ChassisSpeeds(0, 0, 0);
          }
          // m_targetAngle = getGyroAngleInRotation2d().getDegrees();

          SwerveModuleState[] target_states = Drive.Stats.kinematics.toSwerveModuleStates(this.m_swerveSpeeds);
          setModulesStates(target_states);
     }

     public void setAllModulesToZero() {
          SwerveModuleState[] zeroStates = new SwerveModuleState[4];

          zeroStates[0] = new SwerveModuleState(0,
                    Rotation2d.fromDegrees(-Drive.Stats.kFrontLeftModuleOffsetInDegrees));
          zeroStates[1] = new SwerveModuleState(0,
                    Rotation2d.fromDegrees(-Drive.Stats.kFrontRightModuleOffsetInDegrees));
          zeroStates[2] = new SwerveModuleState(0, Rotation2d.fromDegrees(-Drive.Stats.kBackLeftModuleOffsetInDegrees));
          zeroStates[3] = new SwerveModuleState(0,
                    Rotation2d.fromDegrees(-Drive.Stats.kBackRightModuleOffsetInDegrees));

          setModulesStates(zeroStates);
     }

     public double getTargetAngleInDegrees() {
          return m_targetAngle;
     }

     /**
      * gets the angle of the navx
      */
     public Rotation2d getGyroAngleInRotation2d() {
          return Rotation2d.fromDegrees(getHeading());
     }

     public double getHeading() {
          double angleWithOffset = (double) m_navX.getFusedHeading() + m_navXoffset;
          // Bigger than 360: angleWithOffset - 360
          // Smaller than 0: angleWithOffset + 360
          return (angleWithOffset > 360) ? angleWithOffset - 360
                    : (angleWithOffset < 0) ? angleWithOffset + 360 : angleWithOffset;
     }

     /**
      * overrides the addVisionMeasurement method from the implemented VisionObserver
     */

     /**
      * overrides the getCurrentPosition method from the implemented VisionObserver
      */

     @Override
     public void periodic() {
          m_poseEstimator.update(getGyroAngleInRotation2d(), m_modulePositions);

          m_modulePositions = new SwerveModulePosition[] {
                    new SwerveModulePosition(m_frontLeftModule.getDriveDistance(), m_frontLeftModule.getSteerAngle()),
                    new SwerveModulePosition(m_frontRightModule.getDriveDistance(), m_frontRightModule.getSteerAngle()),
                    new SwerveModulePosition(m_backLeftModule.getDriveDistance(), m_backLeftModule.getSteerAngle()),
                    new SwerveModulePosition(m_backRightModule.getDriveDistance(), m_backRightModule.getSteerAngle())
          };

          m_pidController.setP(SmartDashboard.getNumber("Turn To angle P", Drive.PID.kP));
          m_pidController.setI(SmartDashboard.getNumber("Turn To angle I", Drive.PID.kI));


          SmartDashboard.putNumber("absolute compass headeing", m_navX.getCompassHeading());
          // m_frontLeftModule.setModuleState(states[0]);
          // m_frontRightModule.setModuleState(states[1]);
          // m_backLeftModule.setModuleState(states[2]);
          // m_backRightModule.setModuleState(states[3]);

     }

     @Override
     public void simulationPeriodic() {
          // This method will be called once per scheduler run during simulation
     }
}
