package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.ShootingPhysics;
import frc.robot.Utils.ShootingPhysics.AimingSolution;
import frc.robot.Utils.ShootingTables;

import java.util.function.Supplier;

public class Superstructure extends SubsystemBase {

    private final TurretSubsystem leftTurret;
    private final ShooterSubsystem leftShooter;
    private final TurretSubsystem rightTurret;
    private final ShooterSubsystem rightShooter;

    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<ChassisSpeeds> speedSupplier;

    // Physical Offsets
    private final Translation2d LEFT_TURRET_OFFSET = new Translation2d(0.0, 0.25);
    private final Translation2d RIGHT_TURRET_OFFSET = new Translation2d(0.0, -0.25);
    private final Translation2d FIELD_TARGET = new Translation2d(4.6, 4);

    // Status Flags
    private boolean isLeftLockedOn = false;
    private boolean isRightLockedOn = false;

    public Superstructure(
            TurretSubsystem leftTurret, ShooterSubsystem leftShooter,
            TurretSubsystem rightTurret, ShooterSubsystem rightShooter,
            Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedSupplier) {
        
        this.leftTurret = leftTurret;
        this.leftShooter = leftShooter;
        this.rightTurret = rightTurret;
        this.rightShooter = rightShooter;
        this.poseSupplier = poseSupplier;
        this.speedSupplier = speedSupplier;
    }

    @Override
    public void periodic() {
        Pose2d robotPose = poseSupplier.get();
        ChassisSpeeds robotSpeeds = speedSupplier.get();

        // --- LEFT SIDE ---
        isLeftLockedOn = runAimingLoop(
            leftTurret, leftShooter, 
            robotPose, robotSpeeds, 
            LEFT_TURRET_OFFSET, "Left"
        );

        // --- RIGHT SIDE ---
        isRightLockedOn = runAimingLoop(
            rightTurret, rightShooter, 
            robotPose, robotSpeeds, 
            RIGHT_TURRET_OFFSET, "Right"
        );
    }

    /**
     * Now includes a 'name' parameter for logging!
     */
    private boolean runAimingLoop(
            TurretSubsystem turret, 
            ShooterSubsystem shooter, 
            Pose2d robotPose, 
            ChassisSpeeds robotSpeeds, 
            Translation2d offset,
            String sideName) { // <--- Added Name for Dashboard

        // 1. Initial Distance & Speed Lookup
        double rawDistance = robotPose.getTranslation().getDistance(FIELD_TARGET);
        double estimatedExitVel = ShootingTables.exitVelocityMap.get(rawDistance);

        // 2. Physics Calculation
        AimingSolution solution = ShootingPhysics.calculateAimingSolution(
            robotPose, robotSpeeds, offset, FIELD_TARGET, estimatedExitVel
        );

        // 3. Look up specific RPMs for the Effective Distance
        double targetTopRPM = ShootingTables.topFlywheelMap.get(solution.effectiveDistance());
        double targetBottomRPM = ShootingTables.bottomFlywheelMap.get(solution.effectiveDistance());

        // --- TELEMETRY / DEBUGGING ---
        // This is what you asked for: seeing the math in real-time.
        SmartDashboard.putNumber(sideName + "/Aim/Dist_Effective", solution.effectiveDistance());
        SmartDashboard.putNumber(sideName + "/Aim/Target_Angle", solution.turretAngle().getDegrees());
        SmartDashboard.putNumber(sideName + "/Aim/RPM_Top", targetTopRPM);
        SmartDashboard.putNumber(sideName + "/Aim/RPM_Bot", targetBottomRPM);
        
        // Optional: Visualize the "Virtual Target" vs Real Target
        // Ideally use Field2d widget, but X/Y works for quick checks
        SmartDashboard.putNumber(sideName + "/Aim/Virtual_X", solution.virtualTarget().getX());
        SmartDashboard.putNumber(sideName + "/Aim/Virtual_Y", solution.virtualTarget().getY());

        // 4. Command Subsystems
        turret.setTargetAngle(solution.turretAngle());
        shooter.setTargetDistance(solution.effectiveDistance());

        // 5. Lock Checks
        boolean turretAtTarget = Math.abs(turret.getErrorDegrees()) < 2.0;
        boolean shooterAtSpeed = shooter.isReadyToFire();
        
        boolean locked = turretAtTarget && shooterAtSpeed;
        SmartDashboard.putBoolean(sideName + "/Locked", locked);

        return locked;
    }

    public boolean canLeftShoot() { return isLeftLockedOn; }
    public boolean canRightShoot() { return isRightLockedOn; }
}