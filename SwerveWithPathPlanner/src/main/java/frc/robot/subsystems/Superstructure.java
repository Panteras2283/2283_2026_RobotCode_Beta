package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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

    private final Translation2d LEFT_TURRET_OFFSET = new Translation2d(0.0, 0.25);
    private final Translation2d RIGHT_TURRET_OFFSET = new Translation2d(0.0, -0.25);

    // Default Targets
    private final Translation2d BLUE_TARGET = new Translation2d(4.6, 4);
    private final Translation2d RED_TARGET = new Translation2d(11.9, 4);

    private boolean isLeftLockedOn = false;
    private boolean isRightLockedOn = false;

    // --- NEW: Struct Publishers for Visualization ---
    private final StructPublisher<Pose2d> leftTargetPub;
    private final StructPublisher<Pose2d> rightTargetPub;

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

        // --- Initialize NetworkTables Publishers ---
        // These use the new "Struct" format which Elastic/AdvantageScope handle natively
        var table = NetworkTableInstance.getDefault().getTable("Superstructure");
        leftTargetPub = table.getStructTopic("LeftTarget", Pose2d.struct).publish();
        rightTargetPub = table.getStructTopic("RightTarget", Pose2d.struct).publish();
    }

    @Override
    public void periodic() {
        Pose2d robotPose = poseSupplier.get();
        ChassisSpeeds robotSpeeds = speedSupplier.get();

        // 1. Determine Target based on Alliance
        Translation2d currentTarget = BLUE_TARGET; 
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            currentTarget = RED_TARGET;
        }

        // 2. Run Logic (Passing the specific publisher for each side)
        isLeftLockedOn = runAimingLoop(
            leftTurret, leftShooter, 
            robotPose, robotSpeeds, 
            LEFT_TURRET_OFFSET, currentTarget, 
            "Left", leftTargetPub
        );

        isRightLockedOn = runAimingLoop(
            rightTurret, rightShooter, 
            robotPose, robotSpeeds, 
            RIGHT_TURRET_OFFSET, currentTarget, 
            "Right", rightTargetPub
        );
    }

    private boolean runAimingLoop(
            TurretSubsystem turret, 
            ShooterSubsystem shooter, 
            Pose2d robotPose, 
            ChassisSpeeds robotSpeeds, 
            Translation2d offset,
            Translation2d targetLocation,
            String sideName,
            StructPublisher<Pose2d> publisher) { // <--- Receive Publisher

        // 1. Calculate Physics
        double rawDistance = robotPose.getTranslation().getDistance(targetLocation);
        double estimatedExitVel = ShootingTables.exitVelocityMap.get(rawDistance);

        AimingSolution solution = ShootingPhysics.calculateAimingSolution(
            robotPose, robotSpeeds, offset, targetLocation, estimatedExitVel
        );

        // 2. Publish Ghost Target (The "Green" fix)
        // This puts the data in the exact format your dashboard expects for a "Robot Pose"
        publisher.set(solution.virtualTarget());

        // 3. Command Subsystems
        double targetTopRPM = ShootingTables.topFlywheelMap.get(solution.effectiveDistance());
        double targetBottomRPM = ShootingTables.bottomFlywheelMap.get(solution.effectiveDistance());

        // Standard Debug Numbers
        SmartDashboard.putNumber(sideName + "/Aim/Dist_Effective", solution.effectiveDistance());
        SmartDashboard.putNumber(sideName + "/Aim/Target_Angle", solution.turretAngle().getDegrees());

        turret.setTargetAngle(solution.turretAngle());
        shooter.setTargetDistance(solution.effectiveDistance());

        // 4. Lock Check
        boolean turretAtTarget = Math.abs(turret.getErrorDegrees()) < 2.0;
        boolean shooterAtSpeed = shooter.isReadyToFire();
        
        return turretAtTarget && shooterAtSpeed;
    }

    public boolean canLeftShoot() { return isLeftLockedOn; }
    public boolean canRightShoot() { return isRightLockedOn; }
}