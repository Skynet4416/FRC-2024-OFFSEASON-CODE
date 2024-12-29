// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.Drive;
     
import java.util.Optional;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

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
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.Drive;
import frc.robot.Constants.Swerve.PID.ApriltagDriveX;
import frc.robot.Constants.Swerve.PID.ApriltagDriveY;
import frc.robot.subsystems.Drive.Swerve.*;
import frc.robot.subsystems.Vision.Limelight.LimelightSubsystem;

public class DriveSubsystem extends SubsystemBase {
     // https://github.com/CrossTheRoadElec/Phoenix6-Examples/tree/main/java/SwerveWithPathPlanner
     // https://github.com/CrossTheRoadElec/SwerveDriveExample/blob/main/src/main/java/frc/robot/CTRSwerve/CTRSwerveModule.java
     private final SwerveModule frontLeftModule;
     private final SwerveModule frontRightModule;
     private final SwerveModule backLeftModule;
     private final SwerveModule backRightModule;
     private final Pigeon2 pigeon;
     private SwerveModulePosition[] modulePositions;
     private final SwerveDriveOdometry odometry;
     private final PIDController pidController;
     private static PIDController PIDControllerArriveX;
     private static PIDController PIDControllerArriveY;
     private ChassisSpeeds swerveSpeeds;
     private Pose2d lastPose;
     private Pose2d currentPose;
     private SwerveDrivePoseEstimator poseEstimator;
     private double targetAngle;
     private final Field2d field2d = new Field2d();

     public SwerveModule get_fl() {
          return frontLeftModule;
     }

     public SwerveModule get_fr() {
          return frontRightModule;
     }

     public SwerveModule get_bl() {
          return backLeftModule;
     }

     public SwerveModule get_br() {
          return backRightModule;
     }

     

     public DriveSubsystem() {
          this.frontLeftModule = new SwerveModule(
                    Drive.Motors.kFrontLeftDriveFalconCANID,
                    Drive.Motors.kFrontLeftSteerFalconCANID,
                    Drive.Encoders.kFrontLeftSteerEncoderCANID,
                    Drive.Stats.kFrontLeftModuleOffsetInDegrees, false);
          this.frontRightModule = new SwerveModule(
                    Drive.Motors.kFrontRightDriveFalconCANID,
                    Drive.Motors.kFrontRightSteerFalconCANID,
                    Drive.Encoders.kFrontRightSteerEncoderCANID,
                    Drive.Stats.kFrontRightModuleOffsetInDegrees, true);
          this.backLeftModule = new SwerveModule(
                    Drive.Motors.kBackLeftDriveFalconCANID,
                    Drive.Motors.kBackLeftSteerFalconCANID,
                    Drive.Encoders.kBackLeftSteerEncoderCANID,
                    Drive.Stats.kBackLeftModuleOffsetInDegrees, false);
          this.backRightModule = new SwerveModule(
                    Drive.Motors.kBackRightDriveFalconCANID,
                    Drive.Motors.kBackRightSteerFalconCANID,
                    Drive.Encoders.kBackRightSteerEncoderCANID,
                    Drive.Stats.kBackRightModuleOffsetInDegrees, false);

          
          pigeon = new Pigeon2(Constants.Swerve.pigeonID);
          pigeon.getConfigurator().apply(new Pigeon2Configuration());
          pigeon.setYaw(0);

          modulePositions = new SwerveModulePosition[] {
                    new SwerveModulePosition(frontLeftModule.getVelocityMetersPerSecond(),
                              frontLeftModule.getSteerAngle()),
                    new SwerveModulePosition(frontRightModule.getVelocityMetersPerSecond(),
                              frontRightModule.getSteerAngle()),
                    new SwerveModulePosition(backLeftModule.getVelocityMetersPerSecond(),
                              backLeftModule.getSteerAngle()),
                    new SwerveModulePosition(backRightModule.getVelocityMetersPerSecond(),
                              backRightModule.getSteerAngle())
          };

          odometry = new SwerveDriveOdometry(Drive.Stats.kinematics, Rotation2d.fromDegrees(getHeading()),
                    modulePositions);

          swerveSpeeds = new ChassisSpeeds(0, 0, 0);

          currentPose = odometry.getPoseMeters(); // todo needs to take the position from vision
          pidController = new PIDController(Drive.PID.kP, Drive.PID.kI, Drive.PID.kD);
          pidController.enableContinuousInput(0, 360);
          poseEstimator = new SwerveDrivePoseEstimator(Drive.Stats.kinematics, getGyroAngleInRotation2d(),
                    modulePositions, currentPose);
          lastPose = poseEstimator.getEstimatedPosition();
          targetAngle = 90;

          SmartDashboard.putNumber("Turn To angle I", Drive.PID.kI);
          SmartDashboard.putNumber("Turn To angle P", Drive.PID.kP);
          
          PIDControllerArriveX = new PIDController(ApriltagDriveX.kP ,ApriltagDriveX.kI ,ApriltagDriveX.kP);
          PIDControllerArriveY = new PIDController(ApriltagDriveY.kP ,ApriltagDriveY.kI ,ApriltagDriveY.kP);
     }
 
     /**
      * Sets the state of all of the swerve modules
      * 
      * @param moduleState
      *                    WPILib's SwerveModuleState library
      */
     public void setModulesStates(SwerveModuleState[] moduleState) {
          frontLeftModule.setModuleState(moduleState[0]);
          frontRightModule.setModuleState(moduleState[1]);
          backLeftModule.setModuleState(moduleState[2]);
          backRightModule.setModuleState(moduleState[3]);
     }

     /**
      * this is a function to send to the auto, if we integrate the proper vision
      * stuff from Yotam's branch it would replace this
      * 
      * @return
      */
     public SwerveDrivePoseEstimator getPoseEstimator() {
          return this.poseEstimator;
     }

     /**
      * returns the odometry of the drive subsystem
      * 
      * @return
      */
     public SwerveDriveOdometry getOdometry() {
          return odometry;
     }

     /**
      * this is a function that recives the current position from the auto and
      * updates the odometry
      */
     public void resetOdometry(Pose2d currentPose) {
          odometry.resetPosition(pigeon.getRotation2d(), modulePositions, currentPose);
     }

     /**
      * this is a function to send to the auto, if we integrate the proper vision
      * stuff from Yotam's branch it would replace this
      */
     public Pose2d getCurrentPose() {
          return currentPose;
     }

     /**
      * this is a function to send to the auto, if we integrate the proper vision
      * stuff from Yotam's branch it would replace this
      */
     public ChassisSpeeds getSwerveSpeeds() {
          return swerveSpeeds;
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
                    targetAngle += Units.radiansToDegrees(rotationVelocityRps)*1.5;
                    this.swerveSpeeds = new ChassisSpeeds(xVelocityMpsFieldOriented, yVelocityMpsFieldOriented,
                              Math.abs(this.getGyroAngleInRotation2d().getDegrees() - targetAngle) > Drive.PID.kThreshold ? -pidController.calculate(this.getGyroAngleInRotation2d().getDegrees()) : 0);
                    pidController.setSetpoint(targetAngle-90);
               } else {
                    this.swerveSpeeds = new ChassisSpeeds(xVelocityMpsFieldOriented, yVelocityMpsFieldOriented,
                              -rotationVelocityRps * 1.2);
               }
          } else {
               this.swerveSpeeds = new ChassisSpeeds(0, 0, 0);
          }
          // m_targetAngle = getGyroAngleInRotation2d().getDegrees();

          SwerveModuleState[] target_states = Drive.Stats.kinematics.toSwerveModuleStates(this.swerveSpeeds);
          setModulesStates(target_states);
          System.out.println("xVelocity-"+xVelocityMps);
          System.out.println("yVelocity-"+yVelocityMps);
          System.out.println("rotation-"+rotationVelocityRps);
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
          return targetAngle;
     }

     /**
      * gets the angle of the pigeon
      */
     public Rotation2d getGyroAngleInRotation2d() {
          return Rotation2d.fromDegrees(-getHeading());
     }

     public double getHeading() {
          return pigeon.getYaw().getValueAsDouble();
     }

     public void resetGyroOffset(){
          pigeon.setYaw(0);
     }

     /**
      * overrides the addVisionMeasurement method from the implemented VisionObserver
     */

     /**
      * overrides the getCurrentPosition method from the implemented VisionObserver
      */

     @Override
     public void periodic() {
          poseEstimator.update(getGyroAngleInRotation2d(), modulePositions);

          modulePositions = new SwerveModulePosition[] {
                    new SwerveModulePosition(frontLeftModule.getDriveDistance(), frontLeftModule.getSteerAngle()),
                    new SwerveModulePosition(frontRightModule.getDriveDistance(), frontRightModule.getSteerAngle()),
                    new SwerveModulePosition(backLeftModule.getDriveDistance(), backLeftModule.getSteerAngle()),
                    new SwerveModulePosition(backRightModule.getDriveDistance(), backRightModule.getSteerAngle())
          };

          pidController.setP(SmartDashboard.getNumber("Turn To angle P", Drive.PID.kP));
          pidController.setI(SmartDashboard.getNumber("Turn To angle I", Drive.PID.kI));


          SmartDashboard.putNumber("absolute compass headeing", pigeon.getYaw().getValueAsDouble());
          // m_frontLeftModule.setModuleState(states[0]);
          // m_frontRightModule.setModuleState(states[1]);
          // m_backLeftModule.setModuleState(states[2]);
          // m_backRightModule.setModuleState(states[3]);

     
     }
     public void setSetpointArrivalX(double distanceX)
     {
          this.PIDControllerArriveX.setSetpoint(distanceX);
     }
     public void setSetpointArrivalY(double distanceY)
     {
          this.PIDControllerArriveY.setSetpoint(distanceY);
     }
     public double calculateArrivalSpeedWithXPID(double distanceX){
          return PIDControllerArriveX.calculate(distanceX);
     }
     public double calculateArrivalSpeedWithYPID(double distanceY){
          return PIDControllerArriveY.calculate(distanceYY);
     }

     @Override
     public void simulationPeriodic() {
          // This method will be called once per scheduler run during simulation
     }
}