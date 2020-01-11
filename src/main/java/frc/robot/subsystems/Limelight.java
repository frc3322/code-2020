/**
 *  _____    _____     _____     _____   
 * |___  \  |___  \   /  _  \   /  _  \
 *  ___|  |  ___|  | |__| |  | |__| |  |
 * |___   | |___   |     /  /      /  /
 *  ___|  |  ___|  |   /  /__    /  /__
 * |_____/  |_____/   |______|  |______|
 *
 */

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.*;

public class Limelight extends Subsystem {

    public Limelight() {
        SmartDashboard.putNumber("Limelight Angle", 35);
        SmartDashboard.putNumber("Limelight Height", 1/2);
        SmartDashboard.putString("Limelight Lights On", "f");
    }

    @Override
    protected void initDefaultCommand() {
        
    }

    private static NetworkTableInstance table = null;

    /**
     * Light modes for Limelight.
     *
     * @author Dan Waxman
     */
    public static enum LightMode {
        eOn, eOff, eBlink
    }

    /**
     * Camera modes for Limelight.
     *
     * @author Dan Waxman
     */
    public static enum CameraMode {
        eVision, eDriver
    }

    /**
     * Gets whether a target is detected by the Limelight.
     *
     * @return true if a target is detected, false otherwise.
     */
    public boolean isTarget() {
        return getValue("tv").getDouble(0) == 1;
    }

    /**
     * Horizontal offset from crosshair to target (-27 degrees to 27 degrees).
     *
     * @return tx as reported by the Limelight.
     */
    public double getTx() {
        return getValue("tx").getDouble(0.00);
    }

    /**
     * Vertical offset from crosshair to target (-20.5 degrees to 20.5 degrees).
     *
     * @return ty as reported by the Limelight.
     */
    public double getTy() {
        return getValue("ty").getDouble(0.00);
    }

    /**
     * Area that the detected target takes up in total camera FOV (0% to 100%).
     *
     * @return Area of target.
     */
    public double getTa() {
        return getValue("ta").getDouble(0.00);
    }

    /**
     * Gets target skew or rotation (-90 degrees to 0 degrees).
     *
     * @return Target skew.
     */
    public double getTs() {
        return getValue("ts").getDouble(0.00);
    }

    /**
     * Gets target latency (ms).
     *
     * @return Target latency.
     */
    public static double getTl() {
        return getValue("tl").getDouble(0.00);
    }

    /**
     * Sets LED mode of Limelight.
     *
     * @param mode Light mode for Limelight.
     */
    public static void setLedMode(LightMode mode) {
        getValue("ledMode").setNumber(mode.ordinal());
    }

    /**
     * Sets camera mode for Limelight.
     *
     * @param mode Camera mode for Limelight.
     */
    public static void setCameraMode(CameraMode mode) {
        getValue("camMode").setNumber(mode.ordinal());
    }

    public double getDistance() {
        double limelightAngle = 24;
        double targetAngle = getTy();
        double limelightHeight = 1/6;
        double targetHeight = 7 + (5/6);

        return ((targetHeight-limelightHeight)/(Math.tan((limelightAngle + targetAngle) * Math.PI/180)));
    }

    /**
     * Sets pipeline number (0-9 value).
     *
     * @param number Pipeline number (0-9).
     */
    public static void setPipeline(int number) {
        getValue("pipeline").setNumber(number);
    }

    /**
     * Helper method to get an entry from the Limelight NetworkTable.
     *
     * @param key Key for entry.
     * @return NetworkTableEntry of given entry.
     */
    private static NetworkTableEntry getValue(String key) {
        if (table == null) {
            table = NetworkTableInstance.getDefault();
        }

        return table.getTable("limelight").getEntry(key);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Distance to Target", getDistance());
        
        if(SmartDashboard.getString("Limelight Lights On", "f").equals("t")){
            Limelight.setLedMode(LightMode.eOn);
        } else{
            Limelight.setLedMode(LightMode.eOff);
        }
    }
}