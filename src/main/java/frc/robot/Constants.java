package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {
    public static class AdvantageKit {

        public static final Mode simMode = Mode.SIM;
        public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

        public static enum Mode {
            /** Running on a real robot. */
            REAL,

            /** Running a physics simulator. */
            SIM,

            /** Replaying from a log file. */
            REPLAY
        }
    }

    public static class CAN {

    }

    public static class Drive {
        private static final Slot0Configs steerGains = new Slot0Configs()
                .withKP(100)
                .withKI(0)
                .withKD(0.5)
                .withKS(0.1)
                .withKV(1.91)
                .withKA(0)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

        private static final Slot0Configs driveGains = new Slot0Configs().withKP(0.1).withKI(0).withKD(0).withKS(0)
                .withKV(0.124);

        private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
        private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

        private static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
        private static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;

        private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

        private static final Current kSlipCurrent = Amps.of(120.0);

        private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
        private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                .withStatorCurrentLimit(Amps.of(60))
                                .withStatorCurrentLimitEnable(true));
        private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
        private static final Pigeon2Configuration pigeonConfigs = null;

        public static final CANBus kCANBus = new CANBus("canivore", "./logs/example.hoot");

        public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(4.69);

        private static final double kCoupleRatio = 3.8181818181818183;

        private static final double kDriveGearRatio = 7.363636363636365;
        private static final double kSteerGearRatio = 15.42857142857143;
        private static final Distance kWheelRadius = Inches.of(2.167);

        private static final boolean kInvertLeftSide = false;
        private static final boolean kInvertRightSide = true;

        private static final int kPigeonId = 1;

        // These are only used for simulation
        private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.004);
        private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.025);
        // Simulated voltage necessary to overcome friction
        private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
        private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

        public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
                .withCANBusName(kCANBus.getName())
                .withPigeon2Id(kPigeonId)
                .withPigeon2Configs(pigeonConfigs);

        private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator = new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                .withDriveMotorGearRatio(kDriveGearRatio)
                .withSteerMotorGearRatio(kSteerGearRatio)
                .withCouplingGearRatio(kCoupleRatio)
                .withWheelRadius(kWheelRadius)
                .withSteerMotorGains(steerGains)
                .withDriveMotorGains(driveGains)
                .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
                .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
                .withSlipCurrent(kSlipCurrent)
                .withSpeedAt12Volts(kSpeedAt12Volts)
                .withDriveMotorType(kDriveMotorType)
                .withSteerMotorType(kSteerMotorType)
                .withFeedbackSource(kSteerFeedbackType)
                .withDriveMotorInitialConfigs(driveInitialConfigs)
                .withSteerMotorInitialConfigs(steerInitialConfigs)
                .withEncoderInitialConfigs(encoderInitialConfigs)
                .withSteerInertia(kSteerInertia)
                .withDriveInertia(kDriveInertia)
                .withSteerFrictionVoltage(kSteerFrictionVoltage)
                .withDriveFrictionVoltage(kDriveFrictionVoltage);

        // Front Left
        private static final int kFrontLeftDriveMotorId = 3;
        private static final int kFrontLeftSteerMotorId = 2;
        private static final int kFrontLeftEncoderId = 1;
        private static final Angle kFrontLeftEncoderOffset = Rotations.of(0.15234375);
        private static final boolean kFrontLeftSteerMotorInverted = true;
        private static final boolean kFrontLeftEncoderInverted = false;

        private static final Distance kFrontLeftXPos = Inches.of(10);
        private static final Distance kFrontLeftYPos = Inches.of(10);

        // Front Right
        private static final int kFrontRightDriveMotorId = 1;
        private static final int kFrontRightSteerMotorId = 0;
        private static final int kFrontRightEncoderId = 0;
        private static final Angle kFrontRightEncoderOffset = Rotations.of(-0.4873046875);
        private static final boolean kFrontRightSteerMotorInverted = true;
        private static final boolean kFrontRightEncoderInverted = false;

        private static final Distance kFrontRightXPos = Inches.of(10);
        private static final Distance kFrontRightYPos = Inches.of(-10);

        // Back Left
        private static final int kBackLeftDriveMotorId = 7;
        private static final int kBackLeftSteerMotorId = 6;
        private static final int kBackLeftEncoderId = 3;
        private static final Angle kBackLeftEncoderOffset = Rotations.of(-0.219482421875);
        private static final boolean kBackLeftSteerMotorInverted = true;
        private static final boolean kBackLeftEncoderInverted = false;

        private static final Distance kBackLeftXPos = Inches.of(-10);
        private static final Distance kBackLeftYPos = Inches.of(10);

        // Back Right
        private static final int kBackRightDriveMotorId = 5;
        private static final int kBackRightSteerMotorId = 4;
        private static final int kBackRightEncoderId = 2;
        private static final Angle kBackRightEncoderOffset = Rotations.of(0.17236328125);
        private static final boolean kBackRightSteerMotorInverted = true;
        private static final boolean kBackRightEncoderInverted = false;

        private static final Distance kBackRightXPos = Inches.of(-10);
        private static final Distance kBackRightYPos = Inches.of(-10);

        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeft = ConstantCreator
                .createModuleConstants(
                        kFrontLeftSteerMotorId,
                        kFrontLeftDriveMotorId,
                        kFrontLeftEncoderId,
                        kFrontLeftEncoderOffset,
                        kFrontLeftXPos,
                        kFrontLeftYPos,
                        kInvertLeftSide,
                        kFrontLeftSteerMotorInverted,
                        kFrontLeftEncoderInverted);
        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight = ConstantCreator
                .createModuleConstants(
                        kFrontRightSteerMotorId,
                        kFrontRightDriveMotorId,
                        kFrontRightEncoderId,
                        kFrontRightEncoderOffset,
                        kFrontRightXPos,
                        kFrontRightYPos,
                        kInvertRightSide,
                        kFrontRightSteerMotorInverted,
                        kFrontRightEncoderInverted);
        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft = ConstantCreator
                .createModuleConstants(
                        kBackLeftSteerMotorId,
                        kBackLeftDriveMotorId,
                        kBackLeftEncoderId,
                        kBackLeftEncoderOffset,
                        kBackLeftXPos,
                        kBackLeftYPos,
                        kInvertLeftSide,
                        kBackLeftSteerMotorInverted,
                        kBackLeftEncoderInverted);
        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight = ConstantCreator
                .createModuleConstants(
                        kBackRightSteerMotorId,
                        kBackRightDriveMotorId,
                        kBackRightEncoderId,
                        kBackRightEncoderOffset,
                        kBackRightXPos,
                        kBackRightYPos,
                        kInvertRightSide,
                        kBackRightSteerMotorInverted,
                        kBackRightEncoderInverted);

        public static class TunerSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
            public TunerSwerveDrivetrain(
                    SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
                super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConstants, modules);
            }

            public TunerSwerveDrivetrain(
                    SwerveDrivetrainConstants drivetrainConstants,
                    double odometryUpdateFrequency,
                    SwerveModuleConstants<?, ?, ?>... modules) {
                super(
                        TalonFX::new,
                        TalonFX::new,
                        CANcoder::new,
                        drivetrainConstants,
                        odometryUpdateFrequency,
                        modules);
            }

            public TunerSwerveDrivetrain(
                    SwerveDrivetrainConstants drivetrainConstants,
                    double odometryUpdateFrequency,
                    Matrix<N3, N1> odometryStandardDeviation,
                    Matrix<N3, N1> visionStandardDeviation,
                    SwerveModuleConstants<?, ?, ?>... modules) {
                super(
                        TalonFX::new,
                        TalonFX::new,
                        CANcoder::new,
                        drivetrainConstants,
                        odometryUpdateFrequency,
                        odometryStandardDeviation,
                        visionStandardDeviation,
                        modules);
            }
        }
    }
}
