package frc.robot;

import static edu.wpi.first.units.Units.Amps;

import java.util.HashMap;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;

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

    public static class Dashboard
    {
        public static final double LOW_BATTERY_VOLTAGE      = 11.5;
        public static final double CAN_ERROR_TIME_THRESHOLD = 2.0;
    }

    public static class DIO
    {
        public static final int MANIPULATOR_LIGHT_SENSOR_END   = 9;
        public static final int MANIPULATOR_LIGHT_SENSOR_START = 8;
    }

    public static class Drive
    {
        public static final double          TRACK_WIDTH                   = Units.inchesToMeters(22);
        public static final double          WHEEL_BASE                    = Units.inchesToMeters(26);
        public static final double          DRIVE_BASE_RADIUS             = Math.hypot(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0);
        public static final Translation2d[] MODULE_TRANSLATIONS           = new Translation2d[] { new Translation2d(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0), new Translation2d(TRACK_WIDTH / 2.0, -WHEEL_BASE / 2.0),
                new Translation2d(-TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0), new Translation2d(-TRACK_WIDTH / 2.0, -WHEEL_BASE / 2.0), };
        public static final Rotation2d      FL_ZERO_ROTATION              = Rotation2d.fromDegrees(57.45 + 45.0); // Raw reading - 1/8 for the 45 degree offset
        public static final Rotation2d      FR_ZERO_ROTATION              = Rotation2d.fromDegrees(11.29 - 45.0);
        public static final Rotation2d      BL_ZERO_ROTATION              = Rotation2d.fromDegrees(65.26 - 45.0);
        public static final Rotation2d      BR_ZERO_ROTATION              = Rotation2d.fromDegrees(114.1 + 45.0);
        public static final Current         DRIVE_MOTOR_CURRENT_LIMIT     = Amps.of(50);
        public static final double          WHEEL_RADIUS                  = Units.inchesToMeters(2);
        public static final double          DRIVE_MOTOR_REDUCTION         = 5.67;
        public static final boolean         DRIVE_INVERTED                = false;
        public static final DCMotor         DRIVE_GEARBOX                 = DCMotor.getKrakenX60(1);
        public static final double          DRIVE_ENCODER_POSITION_FACTOR = 2 * Math.PI / DRIVE_MOTOR_REDUCTION;
        public static final double          DRIVE_ENCODER_VELOCITY_FACTOR = 2 * Math.PI / 60.0 / DRIVE_MOTOR_REDUCTION;
        public static final double          DRIVE_KP                      = 5.0;
        public static final double          DRIVE_KD                      = 0.0;
        public static final double          DRIVE_KS                      = 0.0;
        public static final double          DRIVE_KV                      = 0.1;
        public static final double          DRIVE_SIM_P                   = 0.05;
        public static final double          DRIVE_SIM_D                   = 0.0;
        public static final double          DRIVE_SIM_KS                  = 0.0;
        public static final double          DRIVE_SIM_KV                  = 0.0789;
        public static final boolean         TURN_INVERTED                 = true;
        public static final int             TURN_MOTOR_CURRENT_LIMIT      = 80;
        public static final double          TURN_MOTOR_REDUCTION          = 12.1;
        public static final DCMotor         TURN_GEARBOX                  = DCMotor.getNEO(1);
        public static final double          TURN_ENCODER_POSITION_FACTOR  = 1.0 / TURN_MOTOR_REDUCTION;
        public static final double          TURN_ENCODER_VELOCITY_FACTOR  = 1 / TURN_MOTOR_REDUCTION;
        public static final double          TURN_KP                       = 7.0;
        public static final double          TURN_KD                       = 0.0;
        public static final double          TURN_SIM_P                    = 8.0;
        public static final double          TURN_SIM_D                    = 0.0;
        public static final double          TURN_PID_MIN_INPUT            = 0;
        public static final double          TURN_PID_MAX_INPUT            = 1;
        public static final double          WHEEL_COF                     = 1.2;
        public static final double          ODOMETRY_FREQUENCY            = 100.0; // ms
        public static final ModuleConfig    MODULE_CONFIG                 = new ModuleConfig(WHEEL_RADIUS, General.MAX_LINEAR_SPEED, WHEEL_COF, DRIVE_GEARBOX, DRIVE_MOTOR_REDUCTION, TURN_MOTOR_CURRENT_LIMIT, 1);
        public static final RobotConfig     PP_CONFIG                     = new RobotConfig(
                General.ROBOT_MASS, General.ROBOT_MOI, new ModuleConfig(WHEEL_RADIUS, General.MAX_LINEAR_SPEED, WHEEL_COF, DRIVE_GEARBOX.withReduction(DRIVE_MOTOR_REDUCTION), DRIVE_MOTOR_CURRENT_LIMIT.magnitude(), 1), MODULE_TRANSLATIONS
        );
        public static final double          MAX_SPEED_ELEVATOR            = 1;
        public static final double          MIN_SPEED_ELEVATOR            = 0.2;
        public static final double          MAX_SPEED_ELEVATOR_HEIGHT     = Constants.Elevator.L2_HEIGHT;
        public static final double          MIN_SPEED_ELEVATOR_HEIGHT     = Constants.Elevator.L3_HEIGHT;
        public static final double          SPEED_ELEVATOR_M              = (MAX_SPEED_ELEVATOR - MIN_SPEED_ELEVATOR) / (MIN_SPEED_ELEVATOR_HEIGHT - MAX_SPEED_ELEVATOR_HEIGHT);
        public static final double          SPEED_ELEVATOR_B              = MAX_SPEED_ELEVATOR - SPEED_ELEVATOR_M * MAX_SPEED_ELEVATOR_HEIGHT;
        public static final double          SPEED_MOTION_THRESHOLD        = 0.02 * Constants.General.MAX_LINEAR_SPEED;
        public static final double          ROTATION_MOTION_THRESHOLD     = 0.02 * Constants.General.MAX_ANGULAR_SPEED;
    }

    public static class Field
    {
        public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT   = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        public static final Rotation2d          BLUE_REEF_ANGLE_ONE      = getTagAngle(18);
        public static final Rotation2d          BLUE_REEF_ANGLE_TWO      = getTagAngle(17);
        public static final Rotation2d          BLUE_REEF_ANGLE_THREE    = getTagAngle(22);
        public static final Rotation2d          BLUE_REEF_ANGLE_FOUR     = getTagAngle(21);
        public static final Rotation2d          BLUE_REEF_ANGLE_FIVE     = getTagAngle(20);
        public static final Rotation2d          BLUE_REEF_ANGLE_SIX      = getTagAngle(19);
        public static final Rotation2d          RED_REEF_ANGLE_ONE       = getTagAngle(7);
        public static final Rotation2d          RED_REEF_ANGLE_TWO       = getTagAngle(8);
        public static final Rotation2d          RED_REEF_ANGLE_THREE     = getTagAngle(9);
        public static final Rotation2d          RED_REEF_ANGLE_FOUR      = getTagAngle(10);
        public static final Rotation2d          RED_REEF_ANGLE_FIVE      = getTagAngle(11);
        public static final Rotation2d          RED_REEF_ANGLE_SIX       = getTagAngle(6);
        public static final Rotation2d          BLUE_LEFT_STATION_ANGLE  = getTagAngle(13);
        public static final Rotation2d          BLUE_RIGHT_STATION_ANGLE = getTagAngle(12);
        public static final Rotation2d          RED_LEFT_STATION_ANGLE   = getTagAngle(1);
        public static final Rotation2d          RED_RIGHT_STATION_ANGLE  = getTagAngle(2);
        public static final Rotation2d          BLUE_PROCESSOR_ANGLE     = getTagAngle(16);
        public static final Rotation2d          RED_PROCESSOR_ANGLE      = getTagAngle(3);

        private static Rotation2d getTagAngle(int tagID)
        {
            return APRIL_TAG_FIELD_LAYOUT.getTagPose(tagID).get().getRotation().toRotation2d().rotateBy(new Rotation2d(Math.PI));
        }
    }

    public static class Elevator
    {
        public static final double  RAW_SENSOR_MIN               = 0.989;
        public static final double  RAW_SENSOR_MAX               = 0.631;
        public static final double  SCALED_MIN                   = 15.5;
        public static final double  SCALED_MAX                   = 74.75;
        public static final double  EXTENSION_KP                 = 0.1;
        public static final double  EXTENSION_KI                 = 0.0;
        public static final double  EXTENSION_KD                 = 0.0; // anything above 0.18 causes "shake"
        public static final double  MAX_EXTENSION                = SCALED_MAX - SCALED_MIN;
        public static final double  EXTENSION_TOLERANCE          = 0.5;
        public static final double  STOW_HEIGHT                  = 15.6;
        public static final double  L1_HEIGHT                    = 23.8; // 12.7625 account for bottom of baseplate to bottom of carrige
        public static final double  L2_HEIGHT                    = 32.1; // 19.11
        public static final double  L3_HEIGHT                    = 47.9; // 34.86
        public static final double  L4_HEIGHT                    = 73.3; // 59.29
        public static final double  HANG_HEIGHT                  = L3_HEIGHT; // 19.11
        public static final double  EXTENSION_SCALE              = (SCALED_MAX - SCALED_MIN) / (RAW_SENSOR_MAX - RAW_SENSOR_MIN);
        public static final double  EXTENSION_MOTOR_REDUCTION    = 5.0;
        public static final DCMotor ELEVATOR_GEARBOX             = DCMotor.getNeoVortex(2);
        public static final double  EXTENSION_OFFSET             = SCALED_MIN - EXTENSION_SCALE * RAW_SENSOR_MIN;
        public static final double  ELEVATOR_MASS                = 15.875;
        public static final double  ELEVATOR_DRUM_RADIUS         = 0.0223139;
        public static final double  ELEVATOR_FEED_FORWARD        = 0.645;
        public static final double  ELEVATOR_MODIFICATION_HEIGHT = 0.5;
    }

    public static class Funnel
    {
        public static final double FUNNEL_VOLTS   = 6.0;
        public static final double DROP_TIME_SECS = 2.0;
        public static final double DEFAULT_VOLTS  = -4.5;
    }

    public static class LED
    {
        public static final double FLASH_TIME_SECS = 1.0;
        public static final int    NUM_LEDS        = 14;
        public static final Color  GREEN           = new Color(0, 115, 0);
        public static final Color  YELLOW          = new Color(255, 115, 0);
        public static final Color  RED             = new Color(255, 0, 0);
        public static final Color  ORANGE          = new Color(255, 50, 0);
        public static final Color  BLUE            = new Color(0, 0, 255);
        public static final Color  PINK            = new Color(255, 46, 204);
        public static final Color  PURPLE          = new Color(127, 0, 255);
    }

    public static class General
    {
        public static final double LOOP_PERIOD_SECS  = 0.02;
        public static final double MAX_LINEAR_SPEED  = 4.8; // m/s
        public static final double ROBOT_MASS        = 74.088;
        public static final double ROBOT_MOI         = 6.883;
        public static final double MOTOR_VOLTAGE     = 12.0;
        public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / Constants.Drive.DRIVE_BASE_RADIUS;
    }

    public static class Lookups
    {
        public static final HashMap<String, Command> _lookup = new HashMap<String, Command>() {
            {
                put("LeftIKL", AutoBuilder.buildAuto("Left_IKL"));
                put("LeftILK", AutoBuilder.buildAuto("Left_ILK"));
                put("LeftJKL", AutoBuilder.buildAuto("Left_JKL"));
                put("LeftJLK", AutoBuilder.buildAuto("Left_JLK"));
                put("MiddleGAB", AutoBuilder.buildAuto("Middle_GAB"));
                put("MiddleGBA", AutoBuilder.buildAuto("Middle_GBA"));
                put("MiddleHAB", AutoBuilder.buildAuto("Middle_HAB"));
                put("MiddleHBA", AutoBuilder.buildAuto("Middle_HBA"));
                put("RightECD", AutoBuilder.buildAuto("Right_ECD"));
                put("RightEDC", AutoBuilder.buildAuto("Right_EDC"));
                put("RightFCD", AutoBuilder.buildAuto("Right_FCD"));
                put("RightFDC", AutoBuilder.buildAuto("Right_FDC"));
            }
        };
    }

    public static class Manipulator
    {
        public static final DCMotor MANIPULATOR_MOTOR = DCMotor.getVex775Pro(1);
        public static final double  MOTOR_REDUCTION   = 5.0;
        public static final double  INTAKE_VOLTS      = 3.50;
        public static final double  OUTPUT_VOLTS      = 9.0;
    }
}
