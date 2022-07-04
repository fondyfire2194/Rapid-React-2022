/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

   public static final double inchToMetersConversionFactor = 0.0254;
   public static final double MINIMUM_TURN_SPEED = 0;

   public static final class CANConstants {

      public static final int PDP = 1;

      public static final int DRIVETRAIN_LEFT_MASTER = 2;
      public static final int DRIVETRAIN_LEFT_FOLLOWER = 3;

      public static final int DRIVETRAIN_RIGHT_MASTER = 4;
      public static final int DRIVETRAIN_RIGHT_FOLLOWER = 5;

      public static final int LEFT_SHOOTER_MOTOR = 6;
      public static final int RIGHT_SHOOTER_MOTOR = 7;

      public static final int TURRET_ROTATE_MOTOR = 8;// turret

      public static final int TILT_MOTOR = 9;

      public static final int TURRET_CANCODER = 31;

      // talons

      public static final int REAR_INTAKE_MOTOR = 14;
      public static final int FRONT_INTAKE_MOTOR = 13;

      public static final int TOP_ROLLER = 10;
      public static final int LOWER_ROLLER = 12;

      public static final int CLIMB_MOTOR = 15;

   }

   public static class SolenoidConstants {
      public static final int CLIMBER_ARM_1 = 4;
      public static final int CLIMBER_ARM_2 = 5;
      public static final int CLIMBER_RATCHET_1 = 6;
      public static final int CLIMBER_RATCHET_2 = 7;

      public static final int FRONT_INTAKE_1 = 3;
      public static final int FRONT_INTAKE_2 = 2;

      public static final int REAR_INTAKE_1 = 1;
      public static final int REAR_INTAKE_2 = 0;

   }

   public static final class PDPConstants {

      // PDP slots
      public static final int DRIVETRAIN_LEFT_MOTOR_A_PDP = 1;
      public static final int DRIVETRAIN_LEFT_MOTOR_B_PDP = 2;
      public static final int DRIVETRAIN_RIGHT_MOTOR_A_PDP = 3;
      public static final int DRIVETRAIN_RIGHT_MOTOR_B_PDP = 4;
      public static final int ELEVATOR_MOTOR_A_PDP = 5;
      public static final int ELEVATOR_MOTOR_B_PDP = 6;
      public static final int SHOOTER_MOTOR_A_PDP = 2;
      public static final int SHOOTER_MOTOR_B_PDP = 3;

   }

   public static final class DriveConstants {

      /**
       * Neo brushless 4096 counts per rev Gearing 10 to 1 6" diameter wheels Neo
       * reports in revs so multiply rev by 4096 to get counts
       * 
       * 
       */
      // DIMENSIONS IN METERS
      // max speed = .4788 *(5700 /(60 *10.2 )) = 4.5 mps (14.5 fps)
      public static double WHEEL_DIAMETER = .1524;// 6"
      public static double WHEEL_CIRCUMFERENCE = .4788;// meters
      public static double METERS_PER_MOTOR_REV = 0.0454;// pi * diameter /gear ratio
      // .0467 *5700/60 = 4.4 mps
      public static double NEO550_COUNTS_PER_REV = 4096;// not used
      public static double DRIVE_GEAR_RATIO = 10.25;

      public static double ksVolts = .227;

      public static double kvVoltSecondsPerMeter = 2.629;//22.63;// 27;//2;

      public static double kaVoltSecondsSquaredPerMeter = .401;// 3.48;//.408;

      public final static double WHEELBASE_WIDTH = .68;

      public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
            WHEELBASE_WIDTH);

      public static final boolean kGyroReversed = true;
      /**
       * 
       * Robot turn constants
       */

      public static double kMaxTurnRateDegPerS = 5.;
      public static double kMaxTurnAccelerationDegPerSSquared = 15.;
      public static double kTurnRateToleranceDegPerS = 2.;
      public static double kTurnToleranceDeg = 3.;

      /**
       * 
       * Position constants
       */
      public static double kPositionRateToleranceMetersPerS = 0.2;
      public static double kPositionToleranceMeters = 0.2;
      public static double kPositionI = 0.01;
      public static double kPositionP = .4;
      public static double kPositionD = .5;
//       public static final double kMaxSpeedMetersPerSecond = 1.0;
//       public static double kMaxPositionAccelerationMetersPerSSquared = 4;

      /**
       * 
       * Trajectory constants
       */

      public static double kMaxTrajectoryMetersPerSecond = 2.5;

      public static double kMaxTrajectoryAccelerationMetersPerSquared = 2.5;

      public static double kPDriveVel = .000001;

      

      public static double kPickupSpeedMetersPerSecond = 1;

      public static final double kvVoltSecondsPerRadian = 3.0;

      public static final double kaVoltSecondsSquaredPerRadian = 0.3;

      public static final LinearSystem<N2, N2, N2> kDrivetrainPlant = LinearSystemId.identifyDrivetrainSystem(
            kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter, kvVoltSecondsPerRadian, kaVoltSecondsSquaredPerRadian);

      public static final DCMotor kDriveGearbox = DCMotor.getNEO(2);

   }

   public static final class ClimberConstants {

   }

   public static class ShooterConstants {

      public static final double MAX_SHOOTER_RPM = 5700;
      public static double kVVoltSecondsPerRotation;
      public static double kA = 0.0061592;
      public static double kV = 0.13622;

      public static double kS = 0.38817;

   }

   /**
    * Tilt axis is a leadscrew driven through a 14T to 28T or 2:1 belt and a 10:1
    * gearbox = 20:1
    * 
    *
    * 
    * The calculated angle will be used for position display and closed loop
    * purposes. The shoot angle counts down as the hood moves up. The angle is
    * preset to max and the encoder is set to 0 on the bottom switch. Then the
    * encoder is subtracted from max so as it increases the angle decreases.
    * 
    * It is pretty much linear with turns.
    * 
    * Leadscrew has 122/20 or 6.1 turns from bottom limit switch
    * 
    * 
    */

   public static class TiltConstants {

      public static final double TILT_MIN_ANGLE = -1;

      public static final double TILT_MAX_ANGLE = 18;

      public static double leadScrewPitch = Units.metersToInches(0.004);

      public static double tiltArmLength = 12.375;
      // lsp/tal = .15748/12.375 = .01272
      public static double motorGearRatio = 20;
      // atan .1272 = .7288
      public static double anglePerScrewRev = Units.radiansToDegrees(Math.atan(leadScrewPitch / tiltArmLength));
      // .03645
      public static double tiltDegreesPerRev = anglePerScrewRev / motorGearRatio;

      public static final double kVVoltSecondsPerRotation = 0.0774;

      public static final double kA = 0.00003;
   }

   public static class TurretConstants {
      /**
       * 21 revs of turret motor turns an 18 tooth pinion one time There are 222 teeth
       * in 360 degrees, so 1 tooth = 360/222 = 1.6216 degrees So 18 teeth = 18 *
       * 1.6216
       * = 28.189 degrees and one motor rev is 1.38996 degrees
       * 
       * At 10,000 motor rpm = 170 rps the turret rotates at 1.421 * 170 = 242 deg per
       * sec
       * ff = 1/ max deg per sec = 1/242 = .004
       * 
       * With end to end travel of 100 degrees = 100/142 = .7 second
       * 
       * 
       * 
       */

      public static final double TURRET_MAX_ANGLE = 35;

      public static final double TURRET_MIN_ANGLE = -35;

      public static final double TURRET_DEG_PER_MOTOR_REV = 1.38996;// max rps = 150 so 208 deg per sec

      public static final double kVVoltSecondsPerRotation = 0.0774;

      public static final double kA = 0.00003;

    public static final double ABSZEROVALUE = 0;

   }

   public static final class AutoConstants {

      public static final double kMaxAccelerationMetersPerSecondSquared = 10;

      // Reasonable baseline values for a RAMSETE follower in units of meters and
      // seconds
      public static final double kRamseteB = 2;
      public static final double kRamseteZeta = 0.7;
   }

   public static final class OIConstants {
      public static final int kDriverControllerPort = 0;
      public static final int kCoDriverControllerPort = 1;
      public static final int kSetupControllerPort = 3;
      public static final int kShootBoxControllerPort = 2;
   }

   public static final class FieldConstants {

      public static final double fieldWidth = Units.inchesToMeters(323);

      public static final double fieldLength = Units.inchesToMeters(626.25);

      public static final double hubTargetHeight = 105.5 / 12.;

      public static final double CAMERA_HEIGHT = 42. / 12;

      public static final double heightDifference = hubTargetHeight - CAMERA_HEIGHT;

      public static final double CAMERA_ANGLE = 128.5 - 90.;

      public static final double CAMERA_TO_FRONT_BUMPER = 1.33;

   }

   public static class ShooterRangeConstants {
      public static final int range1 = 5; // front bumper to hub fenders in feet
      public static final int range2 = 8;
      public static final int range3 = 14;
      public static final int range4 = 20;

      public static final int tiltRange1 = 4;// degrees
      public static final int tiltRange2 = 11;
      public static final int tiltRange3 = 14;
      public static final int tiltRange4 = 17;

   }

   public static class PipelinesConstants {

      public static final int noZoom960720 = 1;

      public static final int x2Zoom320240 = 2;

      public static final int x3ZoomPipeline = 3;

      public static final int ledsOffPipeline = 8;

      public static final double noZoomMinBoundingBoxHeight = 3;

      public static final double noZoomMaxBoundingBoxHeight = 10;

      public static final double x2ZoomMinBoundingBoxHeight = 8;

   }

   public static double kRobotPeriod = .02;
}
