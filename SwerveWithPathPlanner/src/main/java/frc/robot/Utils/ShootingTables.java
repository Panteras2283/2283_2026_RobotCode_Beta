package frc.robot.Utils;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShootingTables {

    // Target RPM for the Main (Bottom) Flywheel
    public static InterpolatingDoubleTreeMap bottomFlywheelMap = new InterpolatingDoubleTreeMap();
    
    // Target RPM for the Top Flywheel (Slower = More Backspin = More Lift)
    // If you only have one flywheel + hood, ignore this.
    public static InterpolatingDoubleTreeMap topFlywheelMap = new InterpolatingDoubleTreeMap();

    // Hood Angle
    public static InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();

    // Predicted Exit Velocity (For shooting-while-moving math)
    public static InterpolatingDoubleTreeMap exitVelocityMap = new InterpolatingDoubleTreeMap();

    static {
        // DATA POINT 1: Fender Shot (Close)
        // High angle, low speed, minimal spin needed
        bottomFlywheelMap.put(1.5, 3000.0);
        topFlywheelMap.put(1.5, 2000.0); 
        hoodAngleMap.put(1.5, 65.0);
        exitVelocityMap.put(1.5, 12.0);

        // DATA POINT 2: Initiation Line (Mid)
        // Need more lift (backspin) to keep the trajectory flat
        bottomFlywheelMap.put(3.5, 4500.0);
        topFlywheelMap.put(3.5, 2500.0); // Big difference = Big Spin
        hoodAngleMap.put(3.5, 45.0);
        exitVelocityMap.put(3.5, 20.0);

        // DATA POINT 3: Long Range
        // Max power.
        bottomFlywheelMap.put(6.0, 6000.0);
        topFlywheelMap.put(6.0, 4000.0);
        hoodAngleMap.put(6.0, 28.0);
        exitVelocityMap.put(6.0, 26.0);
    }
}