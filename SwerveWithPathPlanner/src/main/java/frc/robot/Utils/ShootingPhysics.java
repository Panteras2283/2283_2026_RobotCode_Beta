package frc.robot.Utils;

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
     */
    public record AimingSolution(
        Rotation2d turretAngle,     // The angle the turret should face (Robot-Relative)
        double effectiveDistance,   // The distance to use for RPM lookup tables
        Pose2d virtualTarget        // <--- CHANGED: Now a Pose2d for visualization!
    ) {}

    public static AimingSolution calculateAimingSolution(
            Pose2d robotPose,
            ChassisSpeeds robotSpeeds,
            Translation2d turretOffset,
            Translation2d targetLocation,
            double exitVelocityMps) {

        // 1. Calculate the Field Position of the Turret itself
        Pose2d turretPose = robotPose.transformBy(new Transform2d(turretOffset, new Rotation2d()));
        Translation2d turretTranslation = turretPose.getTranslation();

        // 2. Calculate the True Velocity of the Turret (Linear + Tangential)
        ChassisSpeeds turretVelocity = getTurretFieldVelocity(robotSpeeds, turretOffset, robotPose.getRotation());

        // 3. Calculate Distance to Target
        double rawDistance = targetLocation.getDistance(turretTranslation);

        // 4. Calculate Time of Flight (t = d / v)
        double timeOfFlight = rawDistance / exitVelocityMps;

        // 5. Calculate Drift
        double driftX = turretVelocity.vxMetersPerSecond * timeOfFlight;
        double driftY = turretVelocity.vyMetersPerSecond * timeOfFlight;

        // 6. Calculate Virtual Target (Translation)
        Translation2d virtualTargetTrans = new Translation2d(
            targetLocation.getX() - driftX,
            targetLocation.getY() - driftY
        );

        // --- NEW: Convert to Pose2d for Visualization ---
        // We set rotation to 0 because a point target doesn't really have a heading, 
        // but the visualizer needs it.
        Pose2d virtualTargetPose = new Pose2d(virtualTargetTrans, new Rotation2d(0));

        // 7. Calculate Field-Relative Angle
        double dx = virtualTargetTrans.getX() - turretTranslation.getX();
        double dy = virtualTargetTrans.getY() - turretTranslation.getY();
        Rotation2d fieldRelativeAngle = new Rotation2d(dx, dy);

        // 8. Convert to Robot-Relative Angle
        Rotation2d robotRelativeAngle = fieldRelativeAngle.minus(robotPose.getRotation());

        // 9. Calculate Effective Distance
        double effectiveDistance = virtualTargetTrans.getDistance(turretTranslation);

        return new AimingSolution(robotRelativeAngle, effectiveDistance, virtualTargetPose);
    }

    private static ChassisSpeeds getTurretFieldVelocity(
            ChassisSpeeds robotSpeeds, 
            Translation2d offset, 
            Rotation2d robotHeading) {

        double vX_robot = robotSpeeds.vxMetersPerSecond;
        double vY_robot = robotSpeeds.vyMetersPerSecond;
        double omega    = robotSpeeds.omegaRadiansPerSecond;

        double vX_turret_robot = vX_robot - (omega * offset.getY());
        double vY_turret_robot = vY_robot + (omega * offset.getX());

        Translation2d robotRelVel = new Translation2d(vX_turret_robot, vY_turret_robot);
        Translation2d fieldRelVel = robotRelVel.rotateBy(robotHeading);

        return new ChassisSpeeds(
            fieldRelVel.getX(), 
            fieldRelVel.getY(), 
            omega
        );
    }
}