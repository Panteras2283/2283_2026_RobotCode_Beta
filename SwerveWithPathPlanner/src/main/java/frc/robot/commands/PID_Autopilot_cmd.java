// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds; // Import this!
import edu.wpi.first.math.util.Units;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.*;

public class PID_Autopilot_cmd extends Command {
  private final CommandSwerveDrivetrain s_Swerve;
  private final Pose2d targetPose;
  private final PIDController xController;
  private final PIDController yController;
  private final PIDController rotationController;

  private final CommandXboxController driverController;
  private final CommandXboxController operatorController;

  // CHANGED: Use ApplyRobotSpeeds instead of FieldCentric.
  // This allows us to handle the coordinate math ourselves and ignore Driver Perspective.
  private final SwerveRequest.ApplyRobotSpeeds driveRequest = new SwerveRequest.ApplyRobotSpeeds();

  // Get max speeds from TunerConstants
  private final double MAX_SPEED = 4.7;
  private final double MAX_ANGULAR_SPEED = Units.rotationsToRadians(0.55);

  // --- TUNING VALUES ---
  private static final double kPX = 1.5;
  private static final double kIX = 0.5; 
  private static final double kDX = 0.0;

  private static final double kPY = 1.5; 
  private static final double kIY = 0.5;
  private static final double kDY = 0.0; 

  private static final double kPRot = 1.5; 
  private static final double kIRot = 0.2; 
  private static final double kDRot = 0.0; 

  private static final double POSE_TOLERANCE_METERS = 0.05; 
  private static final double ANGLE_TOLERANCE_RADIANS = Units.degreesToRadians(2); 
  // --- END TUNING VALUES ---


  public PID_Autopilot_cmd(CommandSwerveDrivetrain s_Swerve, Pose2d targetPose, CommandXboxController driver, CommandXboxController operator) {
    this.s_Swerve = s_Swerve;
    this.targetPose = targetPose;
    this.driverController = driver;
    this.operatorController = operator;

    xController = new PIDController(kPX, kIX, kDX);
    yController = new PIDController(kPY, kIY, kDY);
    rotationController = new PIDController(kPRot, kIRot, kDRot);

    rotationController.enableContinuousInput(-Math.PI, Math.PI);

    xController.setTolerance(POSE_TOLERANCE_METERS);
    yController.setTolerance(POSE_TOLERANCE_METERS);
    rotationController.setTolerance(ANGLE_TOLERANCE_RADIANS);

    addRequirements(s_Swerve);
  }

  @Override
  public void initialize() {
    xController.reset();
    yController.reset();
    rotationController.reset();
    SmartDashboard.putBoolean("AtTargetPose", false);
  }

  @Override
  public void execute() {
    Pose2d currentPose = s_Swerve.getState().Pose;

    // 1. Calculate Field-Relative Velocities (PID)
    double xOutput = xController.calculate(currentPose.getX(), targetPose.getX());
    double yOutput = yController.calculate(currentPose.getY(), targetPose.getY());
    double rotationOutput = rotationController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    SmartDashboard.putNumber("X error", targetPose.getX() - currentPose.getX());
    SmartDashboard.putNumber("Y error", targetPose.getY() - currentPose.getY());
    SmartDashboard.putNumber("Rot Error", targetPose.getRotation().getDegrees() - currentPose.getRotation().getDegrees());

    // Clamp
    xOutput = Math.max(-MAX_SPEED, Math.min(MAX_SPEED, xOutput));
    yOutput = Math.max(-MAX_SPEED, Math.min(MAX_SPEED, yOutput));
    rotationOutput = Math.max(-MAX_ANGULAR_SPEED, Math.min(MAX_ANGULAR_SPEED, rotationOutput));

    // 2. Convert to Robot-Relative Velocities
    // This uses the robot's current rotation to mathematically rotate the vector.
    // This works correctly regardless of alliance color because the Gyro is absolute.
    ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        xOutput, 
        yOutput, 
        rotationOutput, 
        currentPose.getRotation()
    );

    // 3. Apply Control
    s_Swerve.setControl(driveRequest
        .withSpeeds(robotRelativeSpeeds) // Apply the converted speeds directly
    );

    // Rumble Logic
    if(xController.atSetpoint() && yController.atSetpoint() && rotationController.atSetpoint()) {
      SmartDashboard.putBoolean("AtTargetPose", true);
      driverController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.8);
      operatorController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.8);
    } else {
      SmartDashboard.putBoolean("AtTargetPose", false);
      driverController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
      operatorController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    s_Swerve.setControl(new SwerveRequest.SwerveDriveBrake());
    SmartDashboard.putBoolean("AtTargetPose", false);
    driverController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
    operatorController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}