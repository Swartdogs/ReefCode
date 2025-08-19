package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.Utilities;

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

    public static class Choreo
    {
        public static final double DRIVE_KP = 5.0;
        public static final double DRIVE_KD = 0.1;
        public static final double TURN_KP  = 3.5;
        public static final double TURN_KD  = 0.5;
    }

    public static class Controls
    {
        public static final double JOYSTICK_DEADBAND = 0.1;
    }

    public static class Dashboard
    {
        public static final double LOW_BATTERY_VOLTAGE        = 11.5;
        public static final double LOW_BATTERY_TIME_THRESHOLD = 5.0; // seconds
    }

    public static class DIO
    {
        public static final int MANIPULATOR_LIGHT_SENSOR_END   = 9;
        public static final int MANIPULATOR_LIGHT_SENSOR_START = 8;
    }

    public static class Drive
    {
        public static final double          TRACK_WIDTH                         = Units.inchesToMeters(22);
        public static final double          WHEEL_BASE                          = Units.inchesToMeters(26);
        public static final double          DRIVE_BASE_RADIUS                   = Math.hypot(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0);
        public static final Translation2d[] MODULE_TRANSLATIONS                 = new Translation2d[] { new Translation2d(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0), new Translation2d(TRACK_WIDTH / 2.0, -WHEEL_BASE / 2.0),
                new Translation2d(-TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0), new Translation2d(-TRACK_WIDTH / 2.0, -WHEEL_BASE / 2.0) };
        public static final Rotation2d      FL_ZERO_ROTATION                    = Rotation2d.fromDegrees(104.73682672005505); // Rotation2d.fromRadians(0.944 + Math.PI / 4);
        public static final Rotation2d      FR_ZERO_ROTATION                    = Rotation2d.fromDegrees(-28.896014907341144); // Rotation2d.fromRadians(-2.942 + 3 * Math.PI / 4);
        public static final Rotation2d      BL_ZERO_ROTATION                    = Rotation2d.fromDegrees(20.22286182699888); // Rotation2d.fromRadians(1.12 - Math.PI / 4);
        public static final Rotation2d      BR_ZERO_ROTATION                    = Rotation2d.fromDegrees(159.06889771665584); // Rotation2d.fromRadians(-1.150 - 3 * Math.PI / 4);
        public static final int             DRIVE_MOTOR_CURRENT_LIMIT           = 120;
        public static final double          WHEEL_RADIUS                        = Units.inchesToMeters(2.0);
        public static final double          DRIVE_MOTOR_REDUCTION               = 5.67;
        public static final boolean         DRIVE_INVERTED                      = false;
        public static final DCMotor         DRIVE_GEARBOX                       = DCMotor.getKrakenX60(1);
        public static final double          DRIVE_KP                            = 0.079014; // Swerve Module Driving
        public static final double          DRIVE_KD                            = 0.0;
        public static final double          DRIVE_KS                            = 0.21614;
        public static final double          DRIVE_KV                            = 0.10404;
        public static final double          DRIVE_KA                            = 0.0039067;
        public static final double          DRIVE_SIM_KP                        = 0.2;
        public static final double          DRIVE_SIM_KD                        = 0.0;
        public static final double          DRIVE_SIM_KS                        = 0.0;
        public static final double          DRIVE_SIM_KV                        = 0.0789;
        public static final boolean         TURN_INVERTED                       = true;
        public static final int             TURN_MOTOR_CURRENT_LIMIT            = 80;
        public static final double          TURN_MOTOR_REDUCTION                = 12.1;
        public static final DCMotor         TURN_GEARBOX                        = DCMotor.getNEO(1);
        public static final double          TURN_KP                             = 3.0;  // Swerve Module Rotation
        public static final double          TURN_KD                             = 0.0;
        public static final double          TURN_SIM_KP                         = 8.0;
        public static final double          TURN_SIM_KD                         = 0.0;
        public static final double          ODOMETRY_FREQUENCY                  = 100.0; // ms
        public static final double          ROTATE_KP                           = 2.1; // Snap To Angle
        public static final double          ROTATE_KD                           = 0.1;
        public static final double          MAX_SPEED_ELEVATOR_MULTIPLIER       = 1;
        public static final double          MIN_SPEED_ELEVATOR_MULTIPLIER       = 0.3;
        public static final double          MAX_SPEED_ELEVATOR_HEIGHT           = Constants.Elevator.L1_HEIGHT;
        public static final double          MIN_SPEED_ELEVATOR_HEIGHT           = Constants.Elevator.L3_HEIGHT;
        public static final double          SPEED_ELEVATOR_M                    = (MAX_SPEED_ELEVATOR_MULTIPLIER - MIN_SPEED_ELEVATOR_MULTIPLIER) / (MAX_SPEED_ELEVATOR_HEIGHT - MIN_SPEED_ELEVATOR_HEIGHT);
        public static final double          SPEED_ELEVATOR_B                    = MAX_SPEED_ELEVATOR_MULTIPLIER - SPEED_ELEVATOR_M * MAX_SPEED_ELEVATOR_HEIGHT;
        public static final double          MAX_LINEAR_SPEED                    = 4.8; // m/s
        public static final double          MAX_ANGULAR_SPEED                   = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;
        public static final double          SPEED_MOTION_THRESHOLD              = 0.02 * MAX_LINEAR_SPEED;
        public static final double          ROTATION_MOTION_THRESHOLD           = 0.01 * MAX_ANGULAR_SPEED;
        public static final double          MAX_SNAP_SPEED_PERCENTAGE           = 0.7;
        public static final double          MAX_AUTO_TRANSLATE_SPEED_PERCENTAGE = 0.2;
        public static final double          DEADBAND                            = 0.1;
        public static final double          TRANSLATE_KP                        = 10.0; // Auto-Align
        public static final double          TRANSLATE_KI                        = 0.2;
        public static final double          TRANSLATE_KD                        = 0.3;
        public static final double          ANGLE_KP                            = 5.0;
        public static final double          ANGLE_KD                            = 0.4;
        public static final double          ANGLE_MAX_VELOCITY                  = 8.0;
        public static final double          ANGLE_MAX_ACCELERATION              = 20.0;
        public static final double          FF_START_DELAY                      = 2.0; // Secs
        public static final double          FF_RAMP_RATE                        = 0.1; // Volts/Sec
        public static final double          WHEEL_RADIUS_MAX_VELOCITY           = 0.25; // Rad/Sec
        public static final double          WHEEL_RADIUS_RAMP_RATE              = 0.05; // Rad/Sec^2
        public static final double          ACCELEROMETER_COLLISION_THRESHOLD   = 0.8;
    }

    public static class Elevator
    {
        public static final double  RAW_SENSOR_MIN               = 0.625;
        public static final double  RAW_SENSOR_MAX               = 0.3829;
        public static final double  MIN_EXTENSION                = 14.75;
        public static final double  MAX_EXTENSION                = 77.0;
        public static final double  EXTENSION_KP                 = 0.5;
        public static final double  EXTENSION_KD                 = 0.02;
        public static final double  EXTENSION_TOLERANCE          = 1.0;
        public static final double  STOW_HEIGHT                  = MIN_EXTENSION + 2.0;
        public static final double  COAST_HEIGHT                 = MIN_EXTENSION + 10.0;
        public static final double  L1_HEIGHT                    = MIN_EXTENSION + 17.453;
        public static final double  L2_HEIGHT                    = MIN_EXTENSION + 19.665;
        public static final double  L3_HEIGHT                    = MIN_EXTENSION + 32.452;
        public static final double  L4_HEIGHT                    = MIN_EXTENSION + 59.883;
        public static final double  LOW_ALGAE_HEIGHT             = MIN_EXTENSION + 9.557;
        public static final double  HIGH_ALGAE_HEIGHT            = MIN_EXTENSION + 24.344;
        public static final double  HANG_HEIGHT                  = MIN_EXTENSION + 23.549;
        public static final double  MAX_ASCENT_SPEED             = 0.8;
        public static final double  MAX_DESCENT_SPEED            = 0.2;
        public static final double  EXTENSION_SCALE              = (MAX_EXTENSION - MIN_EXTENSION) / (RAW_SENSOR_MAX - RAW_SENSOR_MIN);
        public static final double  EXTENSION_MOTOR_REDUCTION    = 5.0;
        public static final DCMotor ELEVATOR_GEARBOX             = DCMotor.getNeoVortex(2);
        public static final double  EXTENSION_OFFSET             = MIN_EXTENSION - EXTENSION_SCALE * RAW_SENSOR_MIN;
        public static final double  ELEVATOR_MASS                = 15.875;
        public static final double  ELEVATOR_DRUM_RADIUS         = 0.0223139;
        public static final double  ELEVATOR_FEED_FORWARD        = 0.992;
        public static final double  ELEVATOR_MODIFICATION_HEIGHT = 0.5;
        public static final double  HANG_SPEED                   = 1.0;
        public static final double  WAIT_TIME                    = 0.5;
    }

    public static class Field
    {
        public static final AprilTagFieldLayout       APRIL_TAG_FIELD_LAYOUT   = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
        private static final Map<Integer, Rotation2d> APRIL_TAG_ANGLES         = new HashMap<Integer, Rotation2d>() {
                                                                                   {
                                                                                       put(1, getFieldTagAngle(1).plus(Rotation2d.fromDegrees(180)));
                                                                                       put(2, getFieldTagAngle(2).plus(Rotation2d.fromDegrees(180)));
                                                                                       put(3, getFieldTagAngle(3));
                                                                                       put(4, getFieldTagAngle(4).plus(Rotation2d.fromDegrees(180)));
                                                                                       put(5, getFieldTagAngle(5).plus(Rotation2d.fromDegrees(180)));
                                                                                       put(6, getFieldTagAngle(6));
                                                                                       put(7, getFieldTagAngle(7));
                                                                                       put(8, getFieldTagAngle(8));
                                                                                       put(9, getFieldTagAngle(9));
                                                                                       put(10, getFieldTagAngle(10));
                                                                                       put(11, getFieldTagAngle(11));
                                                                                       put(12, getFieldTagAngle(12).plus(Rotation2d.fromDegrees(180)));
                                                                                       put(13, getFieldTagAngle(13).plus(Rotation2d.fromDegrees(180)));
                                                                                       put(14, getFieldTagAngle(14).plus(Rotation2d.fromDegrees(180)));
                                                                                       put(15, getFieldTagAngle(15).plus(Rotation2d.fromDegrees(180)));
                                                                                       put(16, getFieldTagAngle(16));
                                                                                       put(17, getFieldTagAngle(17));
                                                                                       put(18, getFieldTagAngle(18));
                                                                                       put(19, getFieldTagAngle(19));
                                                                                       put(20, getFieldTagAngle(20));
                                                                                       put(21, getFieldTagAngle(21));
                                                                                       put(22, getFieldTagAngle(22));
                                                                                   }
                                                                               };
        public static final Rotation2d                BLUE_REEF_ANGLE_ONE      = getTagAngle(18);
        public static final Rotation2d                BLUE_REEF_ANGLE_TWO      = getTagAngle(17);
        public static final Rotation2d                BLUE_REEF_ANGLE_THREE    = getTagAngle(22);
        public static final Rotation2d                BLUE_REEF_ANGLE_FOUR     = getTagAngle(21);
        public static final Rotation2d                BLUE_REEF_ANGLE_FIVE     = getTagAngle(20);
        public static final Rotation2d                BLUE_REEF_ANGLE_SIX      = getTagAngle(19);
        public static final Rotation2d                RED_REEF_ANGLE_ONE       = getTagAngle(7);
        public static final Rotation2d                RED_REEF_ANGLE_TWO       = getTagAngle(8);
        public static final Rotation2d                RED_REEF_ANGLE_THREE     = getTagAngle(9);
        public static final Rotation2d                RED_REEF_ANGLE_FOUR      = getTagAngle(10);
        public static final Rotation2d                RED_REEF_ANGLE_FIVE      = getTagAngle(11);
        public static final Rotation2d                RED_REEF_ANGLE_SIX       = getTagAngle(6);
        public static final Rotation2d                BLUE_LEFT_STATION_ANGLE  = getTagAngle(13);
        public static final Rotation2d                BLUE_RIGHT_STATION_ANGLE = getTagAngle(12);
        public static final Rotation2d                RED_LEFT_STATION_ANGLE   = getTagAngle(1);
        public static final Rotation2d                RED_RIGHT_STATION_ANGLE  = getTagAngle(2);
        public static final Rotation2d                BLUE_PROCESSOR_ANGLE     = getTagAngle(16);
        public static final Rotation2d                RED_PROCESSOR_ANGLE      = getTagAngle(3);

        public static enum Branch
        {
            A(18, 7, Vision.LEFT_REFERENCE), B(18, 7, Vision.RIGHT_REFERENCE), C(17, 8, Vision.LEFT_REFERENCE), D(17, 8, Vision.RIGHT_REFERENCE), E(22, 9, Vision.LEFT_REFERENCE), F(22, 9, Vision.RIGHT_REFERENCE),
            G(21, 10, Vision.LEFT_REFERENCE), H(21, 10, Vision.RIGHT_REFERENCE), I(20, 11, Vision.LEFT_REFERENCE), J(20, 11, Vision.RIGHT_REFERENCE), K(19, 6, Vision.LEFT_REFERENCE), L(19, 6, Vision.RIGHT_REFERENCE);

            private int           blueID;
            private int           redID;
            private Translation2d reference;

            private Branch(int blueID, int redID, Translation2d reference)
            {
                this.blueID    = blueID;
                this.redID     = redID;
                this.reference = reference;
            }

            public int getID()
            {
                return Utilities.isBlueAlliance() ? this.blueID : this.redID;
            }

            public Translation2d getReference()
            {
                return this.reference;
            }
        }

        private static Rotation2d getFieldTagAngle(int tagID)
        {
            return APRIL_TAG_FIELD_LAYOUT.getTagPose(tagID).get().getRotation().toRotation2d().rotateBy(new Rotation2d(Math.PI));
        }

        public static Rotation2d getTagAngle(int tagID)
        {
            return APRIL_TAG_ANGLES.get(tagID);
        }
    }

    public static class Funnel
    {
        public static final double RETRACT_SPEED  = 1.0;
        public static final double DROP_TIME_SECS = 2.0;
    }

    public static class General
    {
        public static final double LOOP_PERIOD_SECS = 0.02;
        public static final double MOTOR_VOLTAGE    = 12.0;
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

    public static class Manipulator
    {
        public static final DCMotor MANIPULATOR_MOTOR   = DCMotor.getNeo550(2);
        public static final double  MOTOR_REDUCTION     = 5.0;
        public static final double  CORAL_INTAKE_SPEED  = 3.5 / Constants.General.MOTOR_VOLTAGE;
        public static final double  CORAL_OUTPUT_SPEED  = 10.0 / Constants.General.MOTOR_VOLTAGE;
        public static final double  ALGAE_INTAKE_SPEED  = -8.0 / Constants.General.MOTOR_VOLTAGE;
        public static final double  ALGAE_OUTPUT_SPEED  = 10.0 / Constants.General.MOTOR_VOLTAGE;
        public static final double  L1_SPEED_MULTIPLIER = 1.0;
        public static final double  DEBOUNCE_TIMER      = 0.2;
        public static final double  INDEX_SPEED         = 1.2 / Constants.General.MOTOR_VOLTAGE;
        public static final double  INDEX_TIME          = 0.35;
    }

    public static class Vision
    {
        public static final double        VISION_STD_DEV_BASE_XY     = 2.0; // meters
        public static final double        VISION_STD_DEV_BASE_THETA  = Units.degreesToRadians(40);
        public static final double        VISION_STD_DEV_MULTI_XY    = 0.1;
        public static final double        VISION_STD_DEV_MULTI_THETA = Units.degreesToRadians(10);
        public static final double        VISION_DISTANCE_SCALE      = 1.5; // How much to increase uncertainty per meter
        public static final double        MAX_DETECTION_RANGE        = Units.inchesToMeters(120);
        public static final double        DRIVE_KP                   = 0.3;
        public static final double        DRIVE_KD                   = 0.0;
        public static final double        AUTO_ALIGN_KP              = 0.005;
        public static final double        AUTO_ALIGN_KD              = 0.0;
        public static final Translation2d LEFT_REFERENCE             = new Translation2d(Units.inchesToMeters(-18.5), Units.inchesToMeters(6.5)); // set values
        public static final Translation2d RIGHT_REFERENCE            = new Translation2d(Units.inchesToMeters(-18.5), Units.inchesToMeters(-6.5)); // set values
    }

    public static class Demo
    {
        public static final double       WHEEL_RADIUS  = Units.inchesToMeters(1.5);
        public static final double       WHEEL_COF     = 1.2;
        public static final double       ROBOT_MASS    = 63.0947;
        public static final double       ROBOT_MOI     = 1.8;
        public static final ModuleConfig MODULE_CONFIG = new ModuleConfig(WHEEL_RADIUS, Drive.MAX_LINEAR_SPEED, WHEEL_COF, Drive.DRIVE_GEARBOX, Drive.DRIVE_MOTOR_REDUCTION, Drive.TURN_MOTOR_CURRENT_LIMIT, 1);
        public static final RobotConfig  PP_CONFIG     = new RobotConfig(
                ROBOT_MASS, ROBOT_MOI, new ModuleConfig(WHEEL_RADIUS, Drive.MAX_LINEAR_SPEED, WHEEL_COF, Drive.DRIVE_GEARBOX.withReduction(Drive.DRIVE_MOTOR_REDUCTION), Drive.DRIVE_MOTOR_CURRENT_LIMIT, 1), Drive.MODULE_TRANSLATIONS
        );
    }
}
