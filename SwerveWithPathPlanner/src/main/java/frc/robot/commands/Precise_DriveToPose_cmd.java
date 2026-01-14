package frc.robot.commands;

import java.util.Set;
import java.util.function.Supplier; // Import Supplier

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Precise_DriveToPose_cmd extends SequentialCommandGroup {

    /**
     * Drives to a specific Reef location (dynamic or fixed) in two stages:
     * 1. Pathfinds to an "Approach Point" (1 meter away).
     * 2. Uses a PID Controller to drive the final meter precisely.
     * * @param drivetrain The drivetrain subsystem.
     * @param constraints The pathfinding constraints.
     * @param sideSupplier Supplier for the Reef Side (1-6).
     * @param slotSupplier Supplier for the Slot ID (0-2).
     * @param driver The driver controller (for rumble).
     * @param operator The operator controller (for rumble).
     */
    public Precise_DriveToPose_cmd(
            CommandSwerveDrivetrain drivetrain, 
            PathConstraints constraints,
            Supplier<Integer> sideSupplier, // Changed to Supplier
            Supplier<Integer> slotSupplier, // Changed to Supplier
            CommandXboxController driver, 
            CommandXboxController operator) {

        addCommands(
            // --- STEP 1: Pathfind to Approach Point ---
            // DeferredCommand ensures we read the "get()" value only when the command actually starts
            new DeferredCommand(() -> {
                // 1. Resolve the Suppliers to actual integers NOW
                int targetSide = sideSupplier.get();
                int targetSlot = slotSupplier.get();

                // 2. Get the Scoring Pose
                Pose2d scoringPose = Constants.AutopilotConstants.getPose(targetSide, targetSlot);

                // 3. Calculate Approach Pose (0.25 meter back)
                Pose2d approachPose = scoringPose.transformBy(new Transform2d(-0.25, 0.0, new Rotation2d()));

                // 4. Return the Pathfind Command
                return drivetrain.PathfindToPose(approachPose, constraints, 0.0);
                
            }, Set.of(drivetrain)),

            // --- STEP 2: PID Approach for the final meter ---
            new DeferredCommand(() -> {
                // 1. Resolve the Suppliers again (in case they changed, though unlikely in 2 seconds)
                int targetSide = sideSupplier.get();
                int targetSlot = slotSupplier.get();

                // 2. Get the Scoring Pose
                Pose2d scoringPose = Constants.AutopilotConstants.getPose(targetSide, targetSlot);

                // 3. Return the PID Command
                return new PID_Autopilot_cmd(
                    drivetrain,
                    scoringPose,
                    driver,
                    operator
                );
            }, Set.of(drivetrain))
        );
    }
}