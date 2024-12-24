// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class Arm {
        public static class Motors {
            public static final int kLeftMotorID = 22;
            public static final int kRightMotorID = 21;
        }

        public static class Stats {
            public static final double kLimitAngle = 90;
            //todo put actual angles and then it will do the thing
            public static final double encoderOffset = 225;
            public static final double gearRatio = 1 / 25.0;
            public static final double ampAngle = 70;
            public static final double speakerAngle = 5;
            public static final double driveAngle = 40;
            public static final double kThreashold = 0;
            public static final double kIntakeAngle = 5;
            public static final double climbAngle = 0;

        }

        public static class Encoders {
            public static final int kLeftEncoderID = 0;
            public static final int kRightEncoderID = 0;
        }

        public static class Pid {
            public static final double kP = 0.1;
            public static final double kI = 0;
            public static final double kD = 0.1;
        }

    }

    public static class Swerve {

        public static class PID {
            // ! --- DO NOT USE THESE PID k VARIABLES IN PRODUCTION! I DID NOT TEST THEM YET
            // ------------------
            public static class Drive {
                /**
                 * Static Friction Offset (to overcome the friction of the system)
                 */
                public static final double kS = 0.0;
                /**
                 * Velocity Feedforward (to continue the current speed)
                 */
                public static final double kV = 0.0;
                /**
                 * the voltage needed to reach a certain acceleration (i have no idea what
                 * number to put)
                 */
                public static final double kA = 0.0;

                /**
                 * Proportional tuning - error
                 * Lower the kP
                 */
                public static final double kP = 0.00035;
                /**
                 * 
                 * Integral tuning - learning
                 */
                public static final double kI = 0.000002;
                /**
                 * Derivative tuning - overshoot
                 */
                public static final double kD = 0.0;
            }

            public static class Steer {
                /**
                 * Static Friction Offset (to overcome the friction of the system)
                 */
                public static final double kS = 0.0;
                /**
                 * Velocity Feedforward (to continue the current speed)
                 */
                public static final double kV = 0.0;
                /**
                 * the voltage needed to reach a certain acceleration (i have no idea what
                 * number to put)
                 */
                public static final double kA = 0.0;

                /**
                 * Proportional tuning - error
                 */
                public static final double kP = 10.0;
                /**
                 * Integral tuning - learning
                 */
                public static final double kI = 1.0;
                /**
                 * Derivative tuning - overshoot
                 */
                public static final double kD = 0.0;
            }

        }

        public static class Stats {
            // todo: change to the actual ratio of the neo vortex
            public static final double kVoltsPerRPM = 0;

            public static final double kMaxVoltage = 12.0;
            public static final double kStatorCurrentLimit = 35.0;
            public static final double kSupplyCurrentLimit = 35.0;

            /**
             * Distance between the center of the right wheels to the center of the left
             * wheels (Meters)
             */
            public static final double kTrackWidthMeters = 85.5;

            /**
             * +
             * Distance between the center of the back wheels to the center of the front
             * wheels (Meters)
             */
            public static final double kWheelbaseMeters = 85.5;

            /**
             * The ratio between the Motor and the center wheel of the Swerve module (which
             * the CANcoder lies on)
             */
            public static final double kRotorToSensorRatioDrive = 8.14;
            public static final double kRotorToSensorRatioSteer = 150 / 7;

            public static final double kDriveWheelRadiusInches = 2;
            public static final double wheelRadiusMeters = Units.inchesToMeters(kDriveWheelRadiusInches);

        }
        public static final int pigeonID = 26;
    }

    public static class Intake {
        public static class Motors {
            // also the id if there is only one motor
            public static final int kUpperMotorLeftID = 31;
            public static final int kUpperMotorRightID = 34;
        }

        public static class Stats {
            // todo: set the speed needed, and everything in constants honestly
            public static final double kIntakeSpeed = -0.4;
            public static final double kIntakeReverseSpeed = 0.4
            ;
            public static final double kPushingNodeInRounds = 1;
            public static final double kShooterSpeed = -0.05;
        }
    }

    public static class Shooter {
        public static class Motors {
            public static final int ShooterMotorLeftID = 32;
            public static final int ShooterMotorRightID = 33;

        }

        public static class Stats {
            // todo: set the speed needed
            public static final double kIShooterSpeed = 0.0;
            public static final double kPutInAmpSpeed = 0.0;
        }

        public static class PID {
            public static final double kP = 0.1;
            public static final double kI = 0.000;
            public static final double kD = 0;
        }
    }

    public static class Climber {
        public static class PID {
            // todo: set actual numbers
            public static final double kP = 0;
            public static final double kI = 0;
            public static final double kD = 0;
        }

        public static class Motors {
            public static final int kRightHookMotorID = 0;
            public static final int kLeftHookMotorID = 0;
        }

        public static class Stats {
            // the meters the climber gains when "open"
            public static final double kHeightChangeInMeters = 0.0;
            public static final double kMaxHeightInMeters = 0.0;
            // todo: change to actual numbers of stuff
            // the length we want the climber to climb (i think this is the addition to the
            // peropened telescope)
            public static final double kDesiredLength = 0.0;
            public static final double kSpoolCircumference = 0.0;
            // timsoret
            public static final double kGearRatio = 0.0;
            // the rounds the motor needs to do = length we want the climb to go/the
            // circumference of the spool*gear ratio
            public static final double kExtensionTurnsInRounds = kDesiredLength / kSpoolCircumference * kGearRatio;
            // the time it takes the telescop to extend at the chosen speed
            public static final long kExtendTimeMS = 0;
            // the time it takes the telescop to retract at the chosen speed
            public static final long kRetractTimeMS = 0;
            public static final double kThreashold = 0;
            public static final double kRetractInRounds = 4;
        }
    }

    public static class Drive {

        public static class Stats {
            public static final double fieldHeadingOffset = 0;

            /**
             * Distance between the center of the right wheels to the center of the left
             * wheels (Meters)
             */
            public static final double kTrackWidthMeters = 85.5;

            /**
             * Distance between the center of the back wheels to the center of the front
             * wheels (Meters)
             */
            public static final double kWheelbaseMeters = 85.5;

            /**
             * the distance of each module (assuming the drivetrain is a square) from the
             * center of the drivetrain. i made this for the auto. yes this pythagoram is
             * ugly, i know
             */
            public static final double kDriveBaseRadius = Math
                    .sqrt(Math.pow(kTrackWidthMeters / 2.0, 2.0) + Math.pow(kWheelbaseMeters / 2.0, 2.0));

                    // FL (0.957764 * 360) - 180
                    // FR (0.951904 * 360) - 180
                    // BL (0.055908 * 360) - 180
                    // BR (0.201172 * 360) - 180
                    

            /**
             * The current degree of the steer mechanism (At what degree does the drive
             * wheel start)
             */
            public static final double kFrontLeftModuleOffsetInDegrees = (0.459473 * 360) - 180;
            /**
             * The current degree of the steer mechanism (At what degree does the drive
             * wheel start)
             */
            public static final double kFrontRightModuleOffsetInDegrees = (0.957275 * 360) - 180;
            /**
             * The current degree of the steer mechanism (At what degree does the drive
             * wheel start)
             */
            public static final double kBackLeftModuleOffsetInDegrees = (0.566162 * 360) - 180;
            /**
             * The current degree of the steer mechanism (At what degree does the drive
             * wheel start)
             */

            public static final double kBackRightModuleOffsetInDegrees = (0.194824 * 360) - 180;
            /*            BL:20.0
            BR:-166.0
            FL:102.0
            FR:156.0 
            */
            public static final double kMaxDriveAccelRPM = 9000;
            public static final double kDriveEfficiency = 0.8;
            public static final double kMaxDriveMotorRPM = 6784.0;
            public static final double kMaxVelocityMetersPerSecond = (kMaxDriveMotorRPM*kDriveEfficiency)*0.319/60/ Swerve.Stats.kRotorToSensorRatioDrive;
            public static final double kMaxAngularVelocityRadiansPerSecond = kMaxVelocityMetersPerSecond /
                    Math.hypot(kTrackWidthMeters / 2.0, kWheelbaseMeters / 2.0);

            public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                    // TODO needs to be configured with diffrent constants that has the modules
                    // position relative to the middle of the robot
                    new Translation2d(kTrackWidthMeters / 2.0, kWheelbaseMeters / 2.0), // ++
                    new Translation2d(kTrackWidthMeters / 2.0, -kWheelbaseMeters / 2.0), // +-
                    new Translation2d(-kTrackWidthMeters / 2.0, kWheelbaseMeters / 2.0), // -+
                    new Translation2d(-kTrackWidthMeters / 2.0, -kWheelbaseMeters / 2.0) // --
            );

        }

        // what are these PID's for? like just general?
        public static class PID {
            public static final double kP = 0.0005;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final double kThreshold = 5; 
        }

        public static class Motors {
            public static final int kFrontLeftDriveFalconCANID = 17;
            public static final int kFrontLeftSteerFalconCANID = 9;

            public static final int kFrontRightDriveFalconCANID = 20;
            public static final int kFrontRightSteerFalconCANID = 12;

            public static final int kBackLeftDriveFalconCANID = 14;
            public static final int kBackLeftSteerFalconCANID = 18;

            public static final int kBackRightDriveFalconCANID = 11;
            public static final int kBackRightSteerFalconCANID = 15;

        }

        public static class Encoders {
            // ? Only the steer encoder exists (seperate from the encoder inside of the
            // Falcon 500 because of ratio problems between the wheels of the swerve
            // modules)
            public static final int kFrontLeftSteerEncoderCANID = 10;
            public static final int kFrontRightSteerEncoderCANID = 13;
            public static final int kBackLeftSteerEncoderCANID = 19;
            public static final int kBackRightSteerEncoderCANID = 16;
        }

    }

    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

    public static class OI {
        public static final int kXboxControllerPort = 0;
        public static final double kXboxcontrollerDrift = 0.1;
    }

    public static class Field {
        /**
         * The field's length in Meters (Found in page 21 of the 2024 game manual)
         */
        public static final double fieldLength = Units.inchesToMeters(651.25);
        /**
         * The field's width in Meters (Found in page 21 of the 2024 game manual)
         */
        public static final double fieldWidth = Units.inchesToMeters(323.25);
        /**
         * The width (and length) in meters of the AprilTags (Found in page 35 of the
         * 2024 game manual)
         */
        public static final double aprilTagWidth = Units.inchesToMeters(8.125);
    }

    // cool it's still here while the vision isn't.
    public static class AllRobot {
            public static final int kAllMotorsLimitInAmpr = 30; // 
    }
}