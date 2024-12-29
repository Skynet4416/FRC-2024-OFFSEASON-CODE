package frc.robot.subsystems.Drive.Swerve;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.VelocityVoltage;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.Drive;
import frc.robot.Constants.Swerve;

public class SwerveModule extends SubsystemBase {
    // https://pro.docs.ctr-electronics.com/en/latest/docs/migration/migration-guide/closed-loop-guide.html
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/trapezoidal-profiles.html

    private final CANSparkFlex driveMotor;
    private final TalonFX steerMotor;
    private final CANcoder steerEncoder;
    private final PositionVoltage voltagePosition;

    private SwerveModuleState moduleState; // current state of the module without steer offset
    private SwerveModuleState targetState;
    private double moduleOffset;
    private boolean isReversed;

    private double targetRotorVelocity = 0;

    /**
     * @param driveMotorCANID
     *                              CANID of the Drive Motor (The Falcon motor that
     *                              controls the wheel)
     * @param steerMotorCANID
     *                              CANID of the Steer Motor (The Falcon motor that
     *                              controls turning)
     * @param steerEncoderCANID
     *                              CANID of the Steer Encoder (on-axis)
     * @param moduleOffsetInDegrees
     *                              The Offset of the module (Relative to the robot)
     */
    public SwerveModule(int driveMotorCANID, int steerMotorCANID, int steerEncoderCANID, double moduleOffsetInDegrees,
            boolean isReversed) {
        this.driveMotor = new CANSparkFlex(driveMotorCANID, CANSparkLowLevel.MotorType.kBrushless);
        this.steerMotor = new TalonFX(steerMotorCANID);
        this.steerEncoder = new CANcoder(steerEncoderCANID);

        this.voltagePosition = new PositionVoltage(0, 0, false, 0, 0, false, false, false);
        this.moduleState = new SwerveModuleState(0, Rotation2d.fromDegrees(0));

        this.moduleOffset = moduleOffsetInDegrees;
        this.isReversed = isReversed;

        configMotors(steerEncoderCANID);
        setBreak();
    }

    /**
     * Adds voltage limits (safety), and true ratios to the motors
     * 
     * @param steerEncoderCANID
     *                          The CANID of the encoder that will replace the
     *                          included encoder inside the Falcon 500
     */
    private void configMotors(int steerEncoderCANID) {

        VoltageConfigs voltageConfigs = new VoltageConfigs()
                .withPeakForwardVoltage(Swerve.Stats.kMaxVoltage)
                .withPeakReverseVoltage(-Swerve.Stats.kMaxVoltage);

        CurrentLimitsConfigs statorConfigs = new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(40)
                .withSupplyCurrentLimit(40);

        FeedbackConfigs steerFeedbackConfigs = new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
                .withFeedbackRemoteSensorID(steerEncoderCANID)
                .withRotorToSensorRatio(Swerve.Stats.kRotorToSensorRatioSteer);

        FeedbackConfigs driveFeedbackConfigs = new FeedbackConfigs()
                .withSensorToMechanismRatio(Swerve.Stats.kRotorToSensorRatioDrive);

        Slot0Configs slot0DriveConfigs = new Slot0Configs()
                .withKA(Swerve.PID.Drive.kA) // Acceleration
                .withKS(Swerve.PID.Drive.kS) // Static Friction Offset
                .withKV(Swerve.PID.Drive.kV) // Velocity Feedforward
                // P I D
                .withKP(Swerve.PID.Drive.kP) // Proportional tuning - error
                .withKI(Swerve.PID.Drive.kI) // Integral tuning - learning
                .withKD(Swerve.PID.Drive.kD); // Derivative tuning - overshoot

        Slot0Configs slot0SteerConfigs = new Slot0Configs()
                .withKA(Swerve.PID.Steer.kA) // Acceleration
                .withKS(Swerve.PID.Steer.kS) // Static Friction Offset
                .withKV(Swerve.PID.Steer.kV) // Velocity Feedforward
                // P I D
                .withKP(Swerve.PID.Steer.kP) // Proportional tuning - error
                .withKI(Swerve.PID.Steer.kI) // Integral tuning - learning
                .withKD(Swerve.PID.Steer.kD); // Derivative tuning - overshoot

        MagnetSensorConfigs sensorConfigs = new MagnetSensorConfigs()
                .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1);

        ClosedLoopGeneralConfigs talonConfigs = new ClosedLoopGeneralConfigs();
        talonConfigs.ContinuousWrap = true;

        this.driveMotor.restoreFactoryDefaults();

        this.driveMotor.getPIDController().setP(Swerve.PID.Drive.kP);
        this.driveMotor.getPIDController().setI(Swerve.PID.Drive.kI);
        this.driveMotor.getPIDController().setD(Swerve.PID.Drive.kD);
        this.driveMotor.setInverted(isReversed);

        this.driveMotor.getPIDController().setSmartMotionMaxVelocity(Drive.Stats.kMaxDriveMotorRPM*Drive.Stats.kDriveEfficiency, 0);
        this.driveMotor.getPIDController().setSmartMotionMaxAccel(Drive.Stats.kMaxDriveAccelRPM,0);

        this.driveMotor.setSmartCurrentLimit(50);
        
        // current theory is that sparkFlex doesn't need configs. but that just a
        // theory.
        // this.m_driveMotor.config();

        // this.m_driveMotor.getConfigurator().apply(voltageConfigs);
        // this.m_driveMotor.getConfigurator().apply(statorConfigs);
        // this.m_driveMotor.getConfigurator().apply(slot0DriveConfigs);
        // this.m_driveMotor.getConfigurator().apply(driveFeedbackConfigs);

        this.steerMotor.getConfigurator().apply(voltageConfigs);
        this.steerMotor.getConfigurator().apply(statorConfigs);
        this.steerMotor.getConfigurator().apply(steerFeedbackConfigs);
        this.steerMotor.getConfigurator().apply(slot0SteerConfigs);
        this.steerMotor.getConfigurator().apply(talonConfigs);
        this.steerMotor.setInverted(true);

        this.steerEncoder.getConfigurator().apply(sensorConfigs);

    }

    public double getTargetRotorVelocityRPM() {
        return this.targetRotorVelocity;
    }
    /**
     * @param target_velocity
     *                        The target velocity in meters per second
     */
    public void setModuleVelocity(double target_velocity) {
        this.targetRotorVelocity = mpsToRps(target_velocity) * 60 * Swerve.Stats.kRotorToSensorRatioDrive;
        this.driveMotor.getPIDController().setReference(targetRotorVelocity, ControlType.kSmartVelocity);
    }

    /**
     * @param target_degrees
     *                       The target angle in degrees of the module
     */
    public void setModuleAngle(double target_degrees) {
        this.steerMotor.setControl(this.voltagePosition.withPosition((this.moduleOffset + target_degrees) / 360));
    }

    public SwerveModuleState getTargetState() {
        return targetState;
    }

    /**
     * @param state
     *              The state of the Module (Affects both the Drive and Steer Motor)
     */
    public void setModuleState(SwerveModuleState state) {
        // state = SwerveModuleState.optimize(state,
        //         Rotation2d.fromDegrees(this.getSteerAngle().getDegrees()));
        targetState = state;
        setModuleVelocity(state.speedMetersPerSecond);
        this.steerMotor
                .setControl(this.voltagePosition.withPosition((moduleOffset + state.angle.getDegrees()) / 360));
    }

    /**
     * Convertion from Rounds per minute to meters per second
     * 
     * @param value
     *              Rounds per minute
     */
    public double rpmToMps(double rpmValue) {
        return (60 / (2 * Math.PI * Swerve.Stats.wheelRadiusMeters)) * rpmValue;
    }

    /**
     * 
     * @param distance
     * @return
     */
    public double roundsToMeters(double distance) {
        return distance * 2 * Math.PI * Swerve.Stats.wheelRadiusMeters;
    }

    /**
     * Convertion from Meters per second to rounds per minute
     * 
     * @param value
     *              Meters per second
     */
    public double mpsToRpm(double mpsValue) {
        return (mpsValue / (2 * Math.PI * Swerve.Stats.wheelRadiusMeters)) * 60;
    }


    /**
     * Convertion from Meters per second to rounds per second
     * 
     * @param value
     *              Meters per second
     */
    public double mpsToRps(double mpsValue) {
        return (mpsValue / (2 * Math.PI * Swerve.Stats.wheelRadiusMeters));
    }

    public Rotation2d getSteerAngle() {
        return this.moduleState.angle;
    }

    public void setCoast() {
        driveMotor.setIdleMode(IdleMode.kCoast);
        steerMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    public void setBreak() {
        driveMotor.setIdleMode(IdleMode.kBrake);
        steerMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public CANSparkFlex getDriveMotor() {
        return this.driveMotor;
    }

    public TalonFX getSteerMotor() {
        return this.steerMotor;
    }

    public double getDriveDistance() {
        return roundsToMeters((Double) this.driveMotor.getEncoder().getPosition());
    }

    public CANcoder getSteerEncoder() {
        return this.steerEncoder;
    }

    public double getVelocityMetersPerSecond() {
        return this.moduleState.speedMetersPerSecond;
    }

    public SwerveModuleState getModuleState() {
        return this.moduleState;
    }

    /**
     *
     * @param speed The speed to set (Percentage). Value should be between -1.0 and
     *              1.0.
     */
    public void setDriveMotor(double speed) {
        this.driveMotor.set(speed);
    }

    /**
     *
     * @param speed The speed to set (Percentage). Value should be between -1.0 and
     *              1.0.
     */
    public void setSteerMotor(double speed) {
        this.steerMotor.set(speed);
    }

    @Override
    public void periodic() { // todo logs needed - ShuffleBoard
        // System.out.println(this.m_moduleState.angle);
        this.moduleState.angle = Rotation2d
                .fromDegrees(this.steerEncoder.getAbsolutePosition().getValue() * 360 - moduleOffset);
        // the function get() returns the speed in percentages, this is kinda ugly but
        // it might work
        this.moduleState.speedMetersPerSecond = rpmToMps((Double) this.driveMotor.get() * 60);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
