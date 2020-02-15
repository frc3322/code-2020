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

    public enum whichBot {
        TEST_BENCH,
        P1,
        P2
    }
    
    public static class RobotMap {
        public static class CAN {

            public final int LEFT_FRONT_MOTOR;
            public final int LEFT_BACK_MOTOR;
            public final int RIGHT_FRONT_MOTOR;
            public final int RIGHT_BACK_MOTOR;

            public final int LEFT_HOPPER_MOTOR;
            public final int RIGHT_HOPPER_MOTOR;
    
            public final int INTAKE_RIGHT;
            public final int INTAKE_LEFT;

            public final int FEEDER_1;
            public final int FEEDER_2;

            public final int CLIMBER_RAISE;
            public final int CLIMBER_CLIMB;

            public final int SHOOTER_1;
            public final int SHOOTER_2;

            public final int SPINNER_MOTOR;

            public CAN() {
                // TODO: switch the part of the enum to match robot currently being deployed to (TEST_BENCH, P1, or P2)
                switch(whichBot.TEST_BENCH){
                    case TEST_BENCH:

                        LEFT_FRONT_MOTOR = 36;
                        LEFT_BACK_MOTOR = 40;
                        RIGHT_FRONT_MOTOR = 45;
                        RIGHT_BACK_MOTOR = 8;
        
                        // TODO: give these actual IDs
                        FEEDER_1 = 20;
                        FEEDER_2 = 21;

                        CLIMBER_RAISE = 0;
                        CLIMBER_CLIMB = 1;

                        LEFT_HOPPER_MOTOR = 5;
                        RIGHT_HOPPER_MOTOR = 4;

                        INTAKE_RIGHT = 6;
                        INTAKE_LEFT = 7;
                    
                        SHOOTER_1 = 33;
                        SHOOTER_2 = 45;
                        
                        SPINNER_MOTOR = 2;

                        break;
                    case P1:

                        LEFT_FRONT_MOTOR = 4;
                        LEFT_BACK_MOTOR = 3;
                        RIGHT_FRONT_MOTOR = 5;
                        RIGHT_BACK_MOTOR = 6;

                        FEEDER_1 = 9;
                        FEEDER_2 = 14;

                        CLIMBER_RAISE = 0;
                        CLIMBER_CLIMB = 0;
        
                        LEFT_HOPPER_MOTOR = 12;
                        RIGHT_HOPPER_MOTOR = 13;
            
                        INTAKE_RIGHT = 0;
                        INTAKE_LEFT = 0;
                    
                        SHOOTER_1 = 10;
                        SHOOTER_2 = 11;

                        SPINNER_MOTOR = 0;

                        break;
                    case P2:

                        // TODO: set these to be P2 CAN IDs
                        LEFT_FRONT_MOTOR = 34;
                        LEFT_BACK_MOTOR = 46;
                        RIGHT_FRONT_MOTOR = 37;
                        RIGHT_BACK_MOTOR = 35;

                        FEEDER_1 = 20;
                        FEEDER_2 = 21;

                        CLIMBER_RAISE = 0;
                        CLIMBER_CLIMB = 1;
        
                        LEFT_HOPPER_MOTOR = 0;
                        RIGHT_HOPPER_MOTOR = 0;
            
                        INTAKE_RIGHT = 0;
                        INTAKE_LEFT = 0;
                    
                        SHOOTER_1 = 0;
                        SHOOTER_2 = 1;

                        SPINNER_MOTOR = 0;

                        break;
                    default:
                        // default IDs are currently Test Bench ones.
                        LEFT_FRONT_MOTOR = 37;
                        LEFT_BACK_MOTOR = 46;
                        RIGHT_FRONT_MOTOR = 34;
                        RIGHT_BACK_MOTOR = 35;
    
                        // TODO: give these actual IDs
                        FEEDER_1 = 20;
                        FEEDER_2 = 21;

                        CLIMBER_RAISE = 0;
                        CLIMBER_CLIMB = 1;

                        LEFT_HOPPER_MOTOR = 0;
                        RIGHT_HOPPER_MOTOR = 0;
        
                        INTAKE_RIGHT = 0;
                        INTAKE_LEFT = 0;
                
                        SHOOTER_1 = 0;
                        SHOOTER_2 = 1;

                        SPINNER_MOTOR = 0;

                        break;
                }
            }
        }
    
        public static class DIO {
            public static final int IR_ID = 0;
        }

        // no real purpose
        public static class JOJO {

        }
    
        public static class PCM {
              
            //TODO: need to give these values 
            public static final int PCM_ID = 0;
            public static final int INTAKE_EXTEND = 2;
            public static final int INTAKE_RETRACT = 3;
            public static final int SPINNER_UP = 0;
            public static final int SPINNER_DOWN = 1;    
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
        public static final double GEARING = 242.0/2480.0;
        public static final double WHEEL_DIAMETER_INCHES = 6;
        public static final double WHEEL_DIAMETER_METERS = WHEEL_DIAMETER_INCHES * 0.0254;
        public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
        public static final boolean GYRO_REVERSED = false;
		public static final double ksVolts = 0.146;
		public static final double kvVoltSecondsPerMeter = 2.65; //2.65 2.64
        public static final double kaVoltSecondsSquaredPerMeter = 0.298; //0.339
        public static final double kTrackwidthMeters = 0.66; //0.66
		public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
		public static final double kPDriveVel = 0; //0.4
    }

    public static class AutoConstants {
		public static final double kRamseteB = 2;
		public static final double kRamseteZeta = 0.7;
		public static double kMaxSpeedMetersPerSecond = 5;
		public static double kMaxAccelerationMetersPerSecondSquared = 2.5;
    }

    public static class ShooterConstants {
        public static final double SHOOTER_TOLERANCE = 30.0;
    }
}
