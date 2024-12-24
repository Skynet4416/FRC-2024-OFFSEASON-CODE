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

    private final CANSparkFlex m_driveMotor;
    private final TalonFX m_steerMotor;
    private final CANcoder m_steerEncoder;
    private final PositionVoltage m_voltagePosition;

    private SwerveModuleState m_moduleState; // current state of the module without steer offset
    private SwerveModuleState m_targetState;
    private double m_moduleOffset;
    private boolean m_isReversed;

    private double m_targetRotorVelocity = 0;

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
        this.m_driveMotor = new CANSparkFlex(driveMotorCANID, CANSparkLowLevel.MotorType.kBrushless);
        this.m_steerMotor = new TalonFX(steerMotorCANID);
        this.m_steerEncoder = new CANcoder(steerEncoderCANID);

        this.m_voltagePosition = new PositionVoltage(0, 0, false, 0, 0, false, false, false);
        this.m_moduleState = new SwerveModuleState(0, Rotation2d.fromDegrees(0));

        this.m_moduleOffset = moduleOffsetInDegrees;
        this.m_isReversed = isReversed;

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

        this.m_driveMotor.restoreFactoryDefaults();

        this.m_driveMotor.getPIDController().setP(Swerve.PID.Drive.kP);
        this.m_driveMotor.getPIDController().setI(Swerve.PID.Drive.kI);
        this.m_driveMotor.getPIDController().setD(Swerve.PID.Drive.kD);
        this.m_driveMotor.setInverted(m_isReversed);

        this.m_driveMotor.getPIDController().setSmartMotionMaxVelocity(Drive.Stats.kMaxDriveMotorRPM*Drive.Stats.kDriveEfficiency, 0);
        this.m_driveMotor.getPIDController().setSmartMotionMaxAccel(Drive.Stats.kMaxDriveAccelRPM,0);

        this.m_driveMotor.setSmartCurrentLimit(50);
        
        // current theory is that sparkFlex doesn't need configs. but that just a
        // theory.
        // this.m_driveMotor.config();

        // this.m_driveMotor.getConfigurator().apply(voltageConfigs);
        // this.m_driveMotor.getConfigurator().apply(statorConfigs);
        // this.m_driveMotor.getConfigurator().apply(slot0DriveConfigs);
        // this.m_driveMotor.getConfigurator().apply(driveFeedbackConfigs);

        this.m_steerMotor.getConfigurator().apply(voltageConfigs);
        this.m_steerMotor.getConfigurator().apply(statorConfigs);
        this.m_steerMotor.getConfigurator().apply(steerFeedbackConfigs);
        this.m_steerMotor.getConfigurator().apply(slot0SteerConfigs);
        this.m_steerMotor.getConfigurator().apply(talonConfigs);
        this.m_steerMotor.setInverted(true);

        this.m_steerEncoder.getConfigurator().apply(sensorConfigs);

    }

    public double getTargetRotorVelocityRPM() {
        return this.m_targetRotorVelocity;
    }

    /**
     * @param target_velocity
     *                        The target velocity in meters per second
     */
    public void setModuleVelocity(double target_velocity) {
        this.m_targetRotorVelocity = mpsToRps(target_velocity) * 60 * Swerve.Stats.kRotorToSensorRatioDrive;
        this.m_driveMotor.getPIDController().setReference(m_targetRotorVelocity, ControlType.kSmartVelocity);
    }

    /**
     * @param target_degrees
     *                       The target angle in degrees of the module
     */
    public void setModuleAngle(double target_degrees) {
        this.m_steerMotor.setControl(this.m_voltagePosition.withPosition((this.m_moduleOffset + target_degrees) / 360));
    }

    public SwerveModuleState getTargetState() {
        return m_targetState;
    }

    /**
     * @param state
     *              The state of the Module (Affects both the Drive and Steer Motor)
     */
    public void setModuleState(SwerveModuleState state) {
        // state = SwerveModuleState.optimize(state,
        //         Rotation2d.fromDegrees(this.getSteerAngle().getDegrees()));
        m_targetState = state;
        setModuleVelocity(state.speedMetersPerSecond);
        this.m_steerMotor
                .setControl(this.m_voltagePosition.withPosition((m_moduleOffset + state.angle.getDegrees()) / 360));
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
        return this.m_moduleState.angle;
    }

    public void setCoast() {
        m_driveMotor.setIdleMode(IdleMode.kCoast);
        m_steerMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    public void setBreak() {
        m_driveMotor.setIdleMode(IdleMode.kBrake);
        m_steerMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public CANSparkFlex getDriveMotor() {
        return this.m_driveMotor;
    }

    public TalonFX getSteerMotor() {
        return this.m_steerMotor;
    }

    public double getDriveDistance() {
        return roundsToMeters((Double) this.m_driveMotor.getEncoder().getPosition());
    }

    public CANcoder getSteerEncoder() {
        return this.m_steerEncoder;
    }

    public double getVelocityMetersPerSecond() {
        return this.m_moduleState.speedMetersPerSecond;
    }

    public SwerveModuleState getModuleState() {
        return this.m_moduleState;
    }

    /**
     *
     * @param speed The speed to set (Percentage). Value should be between -1.0 and
     *              1.0.
     */
    public void setDriveMotor(double speed) {
        this.m_driveMotor.set(speed);
    }

    /**
     *
     * @param speed The speed to set (Percentage). Value should be between -1.0 and
     *              1.0.
     */
    public void setSteerMotor(double speed) {
        this.m_steerMotor.set(speed);
    }

    @Override
    public void periodic() { // todo logs needed - ShuffleBoard
        // System.out.println(this.m_moduleState.angle);
        this.m_moduleState.angle = Rotation2d
                .fromDegrees(this.m_steerEncoder.getAbsolutePosition().getValue() * 360 - m_moduleOffset);
        // the function get() returns the speed in percentages, this is kinda ugly but
        // it might work
        this.m_moduleState.speedMetersPerSecond = rpmToMps((Double) this.m_driveMotor.get() * 60);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
