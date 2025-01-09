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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;

public final class Constants 
{
    public static class AdvantageKit 
    {

        public static final Mode SIM_MODE = Mode.SIM;
        public static final Mode CURRENT_MODE = RobotBase.isReal() ? Mode.REAL : SIM_MODE;

        public static enum Mode 
        {
            /** Running on a real robot. */
            REAL,

            /** Running a physics simulator. */
            SIM,

            /** Replaying from a log file. */
            REPLAY
        }
    }

    public static class AIO
    {
        public static final int FL_ENCODER = 0;
        public static final int FR_ENCODER = 1;
        public static final int BL_ENCODER = 2;
        public static final int BR_ENCODER = 3;
    }

    public static class CAN 
    {
        public static final int FL_DRIVE = 1;
        public static final int FL_TURN = 2;
        public static final int FR_DRIVE = 3;
        public static final int FR_TURN = 4;
        public static final int BL_DRIVE = 5;
        public static final int BL_TURN = 6;
        public static final int BR_DRIVE = 7;
        public static final int BR_TURN = 8;
    }

    public static class Drive 
    {
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
        private static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated; //FIXME: THIS IS WRONG!!!!

        private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder; //FIXME: THIS IS WRONG!!!!

        private static final Current kSlipCurrent = Amps.of(120.0);

        private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
        private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                .withStatorCurrentLimit(Amps.of(60))
                                .withStatorCurrentLimitEnable(true));
        private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();

        public static final CANBus kCANBus = new CANBus("canivore", "./logs/example.hoot");

        public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(4.69); //Is this just max speed?

        public static final double kCoupleRatio = 3.8181818181818183;

        public static final double TRACK_WIDTH_X = Units.inchesToMeters(20);
        public static final double TRACK_WIDTH_Y = Units.inchesToMeters(28.5);
        public static final double DRIVE_BASE_RADIUS = Math.hypot(TRACK_WIDTH_X/2, TRACK_WIDTH_Y/2); //FIXME: SET CONSTANT LATER
        public static final double DRIVE_GEAR_RATIO = 7.363636363636365;
        public static final double TURN_GEAR_RATIO = 15.42857142857143;
        public static final double WHEEL_RADIUS = 2.167; //FIXME: SET THESE CONSTANTS
        public static final double WHEEL_COF = 1.2;
        public static final double MAX_LINEAR_SPEED = 20; //FIXME: TEST THIS
        public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS; //FIXME: TEST THIS
        public static final double DRIVE_MAX_CURRENT = 50;
        public static final double TURN_MAX_CURRENT = 20;


        private static final boolean INVERT_LEFT = false;
        private static final boolean INVERT_RIGHT = true;

        // These are only used for simulation
        private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.004);
        private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.025);
        // Simulated voltage necessary to overcome friction
        private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
        private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

        public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
                .withCANBusName(kCANBus.getName());
                // .withPigeon2Id(kPigeonId)            //We don't use pigeon, swap to what gyro we do use?
                //.withPigeon2Configs(pigeonConfigs);

        private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator = new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
                .withSteerMotorGearRatio(TURN_GEAR_RATIO)
                .withCouplingGearRatio(kCoupleRatio)
                .withWheelRadius(WHEEL_RADIUS)
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
        private static final Angle FL_OFFSET = Rotations.of(0.15234375);
        private static final boolean FL_MOTOR_INVERTED = true;
        private static final boolean FL_ENCODER_INVERTED = false;

        private static final Distance FL_X = Inches.of(10);
        private static final Distance FL_Y = Inches.of(10);

        // Front Right
        private static final Angle FR_OFFSET = Rotations.of(-0.4873046875);
        private static final boolean FR_MOTOR_INVERTED = true;
        private static final boolean FR_ENCODER_INVERTED = false;

        private static final Distance FR_X = Inches.of(10);
        private static final Distance FR_Y = Inches.of(-10);

        // Back Left
        private static final Angle BL_OFFSET = Rotations.of(-0.219482421875);
        private static final boolean BL_MOTOR_INVERTED = true;
        private static final boolean BL_ENCODER_INVERTED = false;

        private static final Distance BL_X = Inches.of(-10);
        private static final Distance BL_Y = Inches.of(10);

        // Back Right
        private static final Angle BR_OFFSET = Rotations.of(0.17236328125);
        private static final boolean BR_MOTOR_INVERTED = true;
        private static final boolean BR_ENCODER_INVERTED = false;

        private static final Distance BR_X = Inches.of(-10);
        private static final Distance BR_Y = Inches.of(-10);

        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeft = ConstantCreator
                .createModuleConstants(
                        CAN.FL_TURN,
                        CAN.FL_DRIVE,
                        AIO.FL_ENCODER,
                        FL_OFFSET,
                        FL_X,
                        FL_Y,
                        INVERT_LEFT,
                        FL_MOTOR_INVERTED,
                        FL_ENCODER_INVERTED);
        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight = ConstantCreator
                .createModuleConstants(
                        CAN.FR_TURN,
                        CAN.FR_DRIVE,
                        AIO.FR_ENCODER,
                        FR_OFFSET,
                        FR_X,
                        FR_Y,
                        INVERT_RIGHT,
                        FR_MOTOR_INVERTED,
                        FR_ENCODER_INVERTED);
        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft = ConstantCreator
                .createModuleConstants(
                        CAN.BL_TURN,
                        CAN.BL_DRIVE,
                        AIO.BL_ENCODER,
                        BL_OFFSET,
                        BL_X,
                        BL_Y,
                        INVERT_LEFT,
                        BL_MOTOR_INVERTED,
                        BL_ENCODER_INVERTED);
        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight = ConstantCreator
                .createModuleConstants(
                        CAN.BR_TURN,
                        CAN.BR_DRIVE,
                        AIO.BR_ENCODER,
                        BR_OFFSET,
                        BR_X,
                        BR_Y,
                        INVERT_RIGHT,
                        BR_MOTOR_INVERTED,
                        BR_ENCODER_INVERTED);

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

    public static class General
    {
        public static final double LOOP_PERIOD_SECS = 0.02;
        public static final double ROBOT_MOI = 7; //FIXME: SET CONSTANTS LATER
        public static final double ROBOT_MASS = 74.088; //FIXME: SET CONSTANTS LATER
        public static final double MOTOR_VOLTAGE = 12.0;
        public static final double MAX_NEO_SPEED = 5874;
        public static final double MAX_KRAKEN_SPEED = 6000; //FIXME: CONFIRM MOTOR SPEEDS

    }
}
