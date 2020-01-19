/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class RobotMap {
        public static class CAN {
            public static final int LEFT_FRONT_MOTOR = 37;
            public static final int LEFT_BACK_MOTOR = 46;
            public static final int RIGHT_FRONT_MOTOR = 34;
            public static final int RIGHT_BACK_MOTOR = 35;
    
            public static final int SHOOTER_1 = 0;
            public static final int SHOOTER_2 = 1;
        }
    
        public static class DIO {

        }
    
        public static class PCM {
    
        }
    
        public static class XBOX {
            // Buttons
            public static final int BUTTON_A = 1;
            public static final int BUTTON_B = 2;
            public static final int BUTTON_X = 3;
            public static final int BUTTON_Y = 4;
            public static final int BUMPER_LEFT = 5;
            public static final int BUMPER_RIGHT = 6;
            public static final int BUTTON_BACK = 7;
            public static final int BUTTON_START = 8;
            public static final int STICK_LEFT = 9;
            public static final int STICK_RIGHT = 10;
    
            // Axes
            public static final int STICK_L_X_AXIS = 0;
            public static final int STICK_L_Y_AXIS = 1;
            public static final int STICK_R_X_AXIS = 4;
            public static final int STICK_R_Y_AXIS = 5;
            public static final int TRIGGER_L_AXIS = 2;
            public static final int TRIGGER_R_AXIS = 3;
        }
    }

    public static class DriveConstants {
        public static final double GEARING = 408.0/2640.0;
        public static final double WHEEL_DIAMETER_INCHES = 4;
        public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_INCHES * .0254 * Math.PI;
        public static final boolean GYRO_REVERSED = false;
		public static final double ksVolts = 0.201;
		public static final double kvVoltSecondsPerMeter = 2.48;
        public static final double kaVoltSecondsSquaredPerMeter = 0.17;
        public static final double kTrackwidthMeters = 0.66;
		public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
		public static final double kPDriveVel = 6.64;
    }

    public static class AutoConstants {
		public static final double kRamseteB = 2;
		public static final double kRamseteZeta = 0.7;

    }
}
