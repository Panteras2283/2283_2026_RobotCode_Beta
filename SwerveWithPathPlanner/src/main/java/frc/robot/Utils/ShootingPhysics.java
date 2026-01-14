package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * A static utility class to handle all aiming mathematics for moving turrets.
 * Handles Linear Velocity, Angular Velocity (Tangential), and Offset Mounting.
 */
public class ShootingPhysics {

    /**
     * Data record to hold the results of the calculation.
     * This keeps your Subsystem code clean.
     */
    public record AimingSolution(
        Rotation2d turretAngle,     // The angle the turret should face (Robot-Relative)
        double effectiveDistance,   // The distance to use for RPM lookup tables
        Translation2d virtualTarget // The calculated "Ghost" target on the field (Debug)
    ) {}

    /**
     * Calculates the exact aiming solution for a specific turret.
     * * @param robotPose        Current field position of the robot center.
     * @param robotSpeeds      Current velocity of the robot center (Vx, Vy, Omega).
     * @param turretOffset     Where this turret is mounted relative to robot center (e.g., Left/Right).
     * @param targetLocation   The static field location of the goal (Speaker/Funnel).
     * @param exitVelocityMps  The predicted speed of the ball (from your lookup table).
     * @return AimingSolution containing the angle and distance.
     */
    public static AimingSolution calculateAimingSolution(
            Pose2d robotPose,
            ChassisSpeeds robotSpeeds,
            Translation2d turretOffset,
            Translation2d targetLocation,
            double exitVelocityMps) {

        // 1. Calculate the Field Position of the Turret itself
        // We transform the robot's center pose by the turret's offset
        Pose2d turretPose = robotPose.transformBy(new Transform2d(turretOffset, new Rotation2d()));
        Translation2d turretTranslation = turretPose.getTranslation();

        // 2. Calculate the True Velocity of the Turret (Linear + Tangential)
        // If the robot is spinning, the Left Turret has a different velocity than the Right Turret.
        ChassisSpeeds turretVelocity = getTurretFieldVelocity(robotSpeeds, turretOffset, robotPose.getRotation());

        // 3. Calculate Distance to Target (from the turret, not robot center)
        double rawDistance = targetLocation.getDistance(turretTranslation);

        // 4. Calculate Time of Flight (t = d / v)
        // Note: For advanced teams, you might iterate this twice to converge, but 1 pass is usually fine.
        double timeOfFlight = rawDistance / exitVelocityMps;

        // 5. Calculate Drift (How far the robot/turret moves while ball is in air)
        double driftX = turretVelocity.vxMetersPerSecond * timeOfFlight;
        double driftY = turretVelocity.vyMetersPerSecond * timeOfFlight;

        // 6. Calculate Virtual Target (Aim Opposite to Drift)
        Translation2d virtualTarget = new Translation2d(
            targetLocation.getX() - driftX,
            targetLocation.getY() - driftY
        );

        // 7. Calculate Field-Relative Angle to Virtual Target
        double dx = virtualTarget.getX() - turretTranslation.getX();
        double dy = virtualTarget.getY() - turretTranslation.getY();
        Rotation2d fieldRelativeAngle = new Rotation2d(dx, dy);

        // 8. Convert to Robot-Relative Angle (Target - Robot Heading)
        // The turret mount is fixed to the chassis, so we subtract chassis rotation.
        Rotation2d robotRelativeAngle = fieldRelativeAngle.minus(robotPose.getRotation());

        // 9. Calculate Effective Distance (Distance to Virtual Target)
        // Use this distance for your Flywheel RPM lookup table to adjust for moving towards/away.
        double effectiveDistance = virtualTarget.getDistance(turretTranslation);

        return new AimingSolution(robotRelativeAngle, effectiveDistance, virtualTarget);
    }

    /**
     * Helper: Calculates the field-relative velocity of a point (turret) on the robot.
     * Combines the robot's linear speed with the tangential speed from spinning.
     */
    private static ChassisSpeeds getTurretFieldVelocity(
            ChassisSpeeds robotSpeeds, 
            Translation2d offset, 
            Rotation2d robotHeading) {

        // A. Convert Robot-Relative speeds to Field-Relative speeds temporarily
        // (ChassisSpeeds from swerve are usually Robot-Relative)
        double vX_field = robotSpeeds.vxMetersPerSecond * robotHeading.getCos() 
                        - robotSpeeds.vyMetersPerSecond * robotHeading.getSin();
        
        double vY_field = robotSpeeds.vxMetersPerSecond * robotHeading.getSin() 
                        + robotSpeeds.vyMetersPerSecond * robotHeading.getCos();

        // B. Add Tangential Velocity (V = omega * r)
        // Omega is in Radians/Sec. 
        // Cross Product 2D: Tangential X = -omega * rY, Tangential Y = omega * rX
        // Be careful: The 'offset' is robot-relative. We need to rotate the offset to field-relative first
        // to apply tangential logic correctly in field frame, OR clearer:
        
        // Simpler approach: Calculate purely in Robot Relative first, then rotate result.
        
        // 1. Robot Relative Linear Velocity at Center
        double vX_robot = robotSpeeds.vxMetersPerSecond;
        double vY_robot = robotSpeeds.vyMetersPerSecond;
        double omega    = robotSpeeds.omegaRadiansPerSecond;

        // 2. Add Tangential Component (V_tan = omega x r)
        double vX_turret_robot = vX_robot - (omega * offset.getY());
        double vY_turret_robot = vY_robot + (omega * offset.getX());

        // 3. Rotate this combined vector to Field Relative
        Translation2d robotRelVel = new Translation2d(vX_turret_robot, vY_turret_robot);
        Translation2d fieldRelVel = robotRelVel.rotateBy(robotHeading);

        return new ChassisSpeeds(
            fieldRelVel.getX(), 
            fieldRelVel.getY(), 
            omega
        );
    }
}