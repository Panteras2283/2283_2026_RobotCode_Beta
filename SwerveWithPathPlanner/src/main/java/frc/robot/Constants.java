// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathConstraints; // Import this
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units; // Import this
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Constants {
    public static final class AutopilotConstants {

        // --- PATHFINDING CONSTRAINTS ---
        // Max Velocity (m/s), Max Acceleration (m/s^2), 
        // Max Angular Velocity (rad/s), Max Angular Acceleration (rad/s^2)
        public static final PathConstraints kPathConstraints = new PathConstraints(
            4.0, 
            10.0, 
            Units.degreesToRadians(540), 
            Units.degreesToRadians(720)
        );

       // Field Dimensions (Standard FRC Field)
       public static final double FIELD_LENGTH_METERS = 16.540;
       public static final double FIELD_WIDTH_METERS = 8.070;

       //Pose2d[# of areas][# of targets per area];
       public static final Pose2d[][] POSES = new Pose2d[3][3];

       static {

            Rotation2d one = Rotation2d.fromDegrees(0);
            Rotation2d two = Rotation2d.fromDegrees(-120);
            Rotation2d three = Rotation2d.fromDegrees(-60);

            // --- SIDE 1 (Example: Neutral Zone) ---
            POSES[0][0] = new Pose2d(7, 6.767, Rotation2d.fromDegrees(-45)); // Side 1, Slot 0 (Left)
            POSES[0][1] = new Pose2d(4.0, 3.5, one); // Side 1, Slot 1 (Center)
            POSES[0][2] = new Pose2d(7, 1.127, Rotation2d.fromDegrees(45)); // Side 1, Slot 2 (Right)

            // --- SIDE 2 ---
            POSES[1][0] = new Pose2d(5.0, 2.0, two);
            POSES[1][1] = new Pose2d(5.5, 2.5, two);
            POSES[1][2] = new Pose2d(6.0, 3.0, two);

             // --- SIDE 3 ---
             POSES[2][0] = new Pose2d(5.0, 2.0, three);
             POSES[2][1] = new Pose2d(5.5, 2.5, three);
             POSES[2][2] = new Pose2d(6.0, 3.0, three);
 
       }
       public static Pose2d getPose(int side, int slot) {
           
            int sideIndex = side;

            // Safety Checks
            if (sideIndex < 0 || sideIndex >= POSES.length) {
                System.out.println("[Autopilot] Invalid Side: " + side);
                return new Pose2d();
            }
            if (slot < 0 || slot >= POSES[0].length) {
                System.out.println("[Autopilot] Invalid Slot: " + slot);
                return new Pose2d();
            }

            Pose2d bluePose = POSES[sideIndex][slot];

            // Check Alliance and Flip if Red
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                return flipPoseToRed(bluePose);
            }

            return bluePose;
        }

        /**
         * Flips a Blue Alliance Pose to the Red Alliance side of the field.
         * Assumes the field is mirrored across the center line (standard FRC).
         */
        private static Pose2d flipPoseToRed(Pose2d bluePose) {
            return new Pose2d(
                FIELD_LENGTH_METERS - bluePose.getX(),
                bluePose.getY(), // Usually Y is not flipped in "Mirrored" fields, only X. 
                                 // NOTE: Verify 2025 rules (Point Symmetry vs Reflection).
                                 // If Point Symmetry (rotated 180): Y = FIELD_WIDTH - Y
                new Rotation2d(Math.PI).minus(bluePose.getRotation()) // Rotate 180 - theta
            );
        }
    }
}
