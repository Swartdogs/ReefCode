package frc.robot;

import static edu.wpi.first.units.Units.Amps;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.RobotBase;

public final class Constants
{
    public static class AdvantageKit
    {
        public static final Mode SIM_MODE     = Mode.SIM;
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
        public static final int FL_ENCODER    = 0;
        public static final int FR_ENCODER    = 1;
        public static final int BL_ENCODER    = 2;
        public static final int BR_ENCODER    = 3;
        public static final int EXTENSION_POT = 4;
    }

    public static class CAN
    {
        public static final int FL_DRIVE          = 1;
        public static final int FR_DRIVE          = 3;
        public static final int BL_DRIVE          = 5;
        public static final int BR_DRIVE          = 7;
        public static final int FL_TURN           = 2;
        public static final int FR_TURN           = 4;
        public static final int BL_TURN           = 6;
        public static final int BR_TURN           = 8;
        public static final int LEAD_ELEVATOR     = 9;
        public static final int FOLLOWER_ELEVATOR = 10;
        public static final int MANIPULATOR_LEFT  = 11;
        public static final int MANIPULATOR_RIGHT = 12;
        public static final int FUNNEL_SOLENOID   = 13;
    }

    // eden is the fun police
    public static class Dashboard
    {
        public static final double LOW_BATTERY_VOLTAGE      = 11.5;
        public static final double CAN_ERROR_TIME_THRESHOLD = 2.0; // HI COLLIN!TIME TO WAIT BEFORE REMOVING CAN ERROR.
    }

    public static class DIO
    {
        public static final int MANIPULATOR_LIGHT_SENSOR = 1;
    }

    public static class Drive
    {
        public static final double          TRACK_WIDTH                   = Units.inchesToMeters(26.5);
        public static final double          WHEEL_BASE                    = Units.inchesToMeters(26.5);
        public static final double          DRIVE_BASE_RADIUS             = Math.hypot(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0);
        public static final Translation2d[] MODULE_TRANSLATIONS           = new Translation2d[] { new Translation2d(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0), new Translation2d(TRACK_WIDTH / 2.0, -WHEEL_BASE / 2.0),
                new Translation2d(-TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0), new Translation2d(-TRACK_WIDTH / 2.0, -WHEEL_BASE / 2.0), };
        public static final Rotation2d      FL_ZERO_ROTATION              = new Rotation2d(0.0);
        public static final Rotation2d      FR_ZERO_ROTATION              = new Rotation2d(0.0);
        public static final Rotation2d      BL_ZERO_ROTATION              = new Rotation2d(0.0);
        public static final Rotation2d      BR_ZERO_ROTATION              = new Rotation2d(0.0);
        public static final Current         DRIVE_MOTOR_CURRENT_LIMIT     = Amps.of(50);
        public static final double          WHEEL_RADIUS                  = Units.inchesToMeters(1.5);
        public static final double          DRIVE_MOTOR_REDUCTION         = 5.67;
        public static final boolean         DRIVE_INVERTED                = false;
        public static final DCMotor         DRIVE_GEARBOX                 = DCMotor.getKrakenX60(1);
        public static final double          DRIVE_ENCODER_POSITION_FACTOR = 2 * Math.PI / DRIVE_MOTOR_REDUCTION;
        public static final double          DRIVE_ENCODER_VELOCITY_FACTOR = 2 * Math.PI / 60.0 / DRIVE_MOTOR_REDUCTION;
        public static final double          DRIVE_KP                      = 0.0;
        public static final double          DRIVE_KD                      = 0.0;
        public static final double          DRIVE_KS                      = 0.0;
        public static final double          DRIVE_KV                      = 0.1;
        public static final double          DRIVE_SIM_P                   = 0.05;
        public static final double          DRIVE_SIM_D                   = 0.0;
        public static final double          DRIVE_SIM_KS                  = 0.0;
        public static final double          DRIVE_SIM_KV                  = 0.0789;
        public static final boolean         TURN_INVERTED                 = false;
        public static final int             TURN_MOTOR_CURRENT_LIMIT      = 20;
        public static final double          TURN_MOTOR_REDUCTION          = 12.0;
        public static final DCMotor         TURN_GEARBOX                  = DCMotor.getNEO(1);
        public static final boolean         TURN_ENCODER_INVERTED         = true;
        public static final double          TURN_ENCODER_POSITION_FACTOR  = 2 * Math.PI;
        public static final double          TURN_ENCODER_VELOCITY_FACTOR  = 2 * Math.PI / 60.0;
        public static final double          TURN_KP                       = 2.0;
        public static final double          TURN_KD                       = 0.0;
        public static final double          TURN_SIM_P                    = 8.0;
        public static final double          TURN_SIM_D                    = 0.0;
        public static final double          TURN_PID_MIN_INPUT            = 0;
        public static final double          TURN_PID_MAX_INPUT            = 2 * Math.PI;
        public static final double          WHEEL_COF                     = 1.2;
        public static final double          ODOMETRY_FREQUENCY            = 100.0; // ms
        public static final ModuleConfig    MODULE_CONFIG                 = new ModuleConfig(WHEEL_RADIUS, General.MAX_LINEAR_SPEED, WHEEL_COF, DRIVE_GEARBOX, DRIVE_MOTOR_REDUCTION, TURN_MOTOR_CURRENT_LIMIT, 1);
        public static final RobotConfig     PP_CONFIG                     = new RobotConfig(
                General.ROBOT_MASS, General.ROBOT_MOI, new ModuleConfig(WHEEL_RADIUS, General.MAX_LINEAR_SPEED, WHEEL_COF, DRIVE_GEARBOX.withReduction(DRIVE_MOTOR_REDUCTION), DRIVE_MOTOR_CURRENT_LIMIT.magnitude(), 1), MODULE_TRANSLATIONS
        );
    }

    public static class Elevator
    {
        public static final double  EXTENSION_KP              = 0.4;
        public static final double  EXTENSION_KI              = 0.7;
        public static final double  EXTENSION_KD              = 0.15; // anything above 0.18 causes "shake"
        public static final double  MAX_EXTENSION             = 68.0;
        public static final double  EXTENSION_TOLERANCE       = 0.5;
        public static final double  STOW_HEIGHT               = 0.0;
        public static final double  L1_HEIGHT                 = 18.0 - 12.7625; // 12.7625 account for bottom of baseplate to bottom of carrige
        public static final double  L2_HEIGHT                 = 31.875 - 12.7625; // 19.11
        public static final double  L3_HEIGHT                 = 47.625 - 12.7625; // 34.86
        public static final double  L4_HEIGHT                 = 72.0 - 12.7625; // 59.29
        public static final double  EXTENSION_SCALE           = 1.757 * Math.PI;
        public static final double  EXTENSION_MOTOR_REDUCTION = 5.0;
        public static final DCMotor ELEVATOR_GEARBOX          = DCMotor.getNeoVortex(2);
        public static final double  EXTENSION_OFFSET          = 0.0;
        public static final double  ELEVATOR_MASS             = 15.875;
        public static final double  ELEVATOR_DRUM_RADIUS      = 0.0223139;
    }

    public static class Funnel
    {
        public static final double FUNNEL_VOLTS   = 6.0;
        public static final double DROP_TIME_SECS = 2.0;
    }

    public static class General
    {
        public static final double LOOP_PERIOD_SECS = 0.02;
        public static final double MAX_LINEAR_SPEED = 4.8; // m/s
        public static final double ROBOT_MASS       = 74.088;
        public static final double ROBOT_MOI        = 6.883;
        public static final double MOTOR_VOLTAGE    = 12.0;
    }

    public static class Manipulator
    {
        public static final DCMotor MANIPULATOR_MOTOR = DCMotor.getVex775Pro(1);
        public static final double  MOTOR_REDUCTION   = 5.0;
        public static final double  INTAKE_VOLTS      = 12.0;
        public static final double  OUTPUT_VOLTS      = 12.0;
    }
}
