package frc.robot;

import static edu.wpi.first.units.Units.Amps;

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
        public static final int FL_ENCODER = 0;
        public static final int FR_ENCODER = 1;
        public static final int BL_ENCODER = 2;
        public static final int BR_ENCODER = 3;
    }

    public static class CAN
    {
        public static final int FL_DRIVE = 1;
        public static final int FR_DRIVE = 3;
        public static final int BL_DRIVE = 5;
        public static final int BR_DRIVE = 7;
        public static final int FL_TURN  = 2;
        public static final int FR_TURN  = 4;
        public static final int BL_TURN  = 6;
        public static final int BR_TURN  = 8;
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

    public static class Field
    {
        public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        public static final Rotation2d          BLUE_REEF_ANGLE_ONE    = getTagAngle(18);
        public static final Rotation2d          BLUE_REEF_ANGLE_TWO    = getTagAngle(17);
        public static final Rotation2d          BLUE_REEF_ANGLE_THREE  = getTagAngle(22);
        public static final Rotation2d          BLUE_REEF_ANGLE_FOUR   = getTagAngle(21);
        public static final Rotation2d          BLUE_REEF_ANGLE_FIVE   = getTagAngle(20);
        public static final Rotation2d          BLUE_REEF_ANGLE_SIX    = getTagAngle(19);
        public static final Rotation2d          RED_REEF_ANGLE_ONE     = getTagAngle(7);
        public static final Rotation2d          RED_REEF_ANGLE_TWO     = getTagAngle(8);
        public static final Rotation2d          RED_REEF_ANGLE_THREE   = getTagAngle(9);
        public static final Rotation2d          RED_REEF_ANGLE_FOUR    = getTagAngle(10);
        public static final Rotation2d          RED_REEF_ANGLE_FIVE    = getTagAngle(11);
        public static final Rotation2d          RED_REEF_ANGLE_SIX     = getTagAngle(6);
        public static final Rotation2d          BLUE_LEFT_STATION      = getTagAngle(13);
        public static final Rotation2d          BLUE_RIGHT_STATION     = getTagAngle(12);
        public static final Rotation2d          RED_LEFT_STATION       = getTagAngle(1);
        public static final Rotation2d          RED_RIGHT_STATION      = getTagAngle(2);
        public static final Rotation2d          BLUE_PROCESSOR         = getTagAngle(16);
        public static final Rotation2d          RED_PROCESSOR          = getTagAngle(3);

        private static Rotation2d getTagAngle(int tagID)
        {
            return APRIL_TAG_FIELD_LAYOUT.getTagPose(tagID).get().getRotation().toRotation2d().rotateBy(new Rotation2d(Math.PI));
        }
    }

    public static class General
    {
        public static final double MAX_LINEAR_SPEED = 4.8; // m/s
        public static final double ROBOT_MASS       = 74.088;
        public static final double ROBOT_MOI        = 6.883;
    }
}
