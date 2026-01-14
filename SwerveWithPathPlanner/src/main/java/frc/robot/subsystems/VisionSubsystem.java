package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.proto.Photon;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Transform3d;

import java.io.IOException;
import java.util.Optional;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;


public class VisionSubsystem extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain;
    
    private static final String kLimelightName = "limelight";

    private final PhotonCamera photon1; 
    private final PhotonCamera photon2;
    private PhotonPoseEstimator photonPoseEstimator1;
    private PhotonPoseEstimator photonPoseEstimator2;
    private static final String kPhotonCameraName1 = "arducam_left";
    private static final String kPhotonCameraName2 = "arducam_right";
    private static final Transform3d kRobotToCam1 = new Transform3d(
        new Translation3d (0,0,0), 
        new Rotation3d(0,0,0)
    );
    private static final Transform3d kRobotToCam2 = new Transform3d(
        new Translation3d (0,0,0), 
        new Rotation3d(0,0,0)
    );



    public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        //Initialize cameras 
        photon1 = new PhotonCamera(kPhotonCameraName1);
        photon2 = new PhotonCamera(kPhotonCameraName2);

        //Load field layout and set up pose estimator
        AprilTagFieldLayout fieldLayout = null;
        try {
            fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
        } catch (Exception e) {
            System.out.println("Failed to load AprilTagFieldLayout: " + e.getMessage());
        }
        
        if(fieldLayout != null){
            //Estimator for camera 1
            photonPoseEstimator1 = new PhotonPoseEstimator(
                fieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                kRobotToCam1
            );
            //Estimator for camera 2
            photonPoseEstimator2 = new PhotonPoseEstimator(
                fieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                kRobotToCam2
            );
        }

        
    }


    @Override
    public void periodic() {
        updateLimelight();
        updatePhotonVision(photonPoseEstimator1, photon1);
        updatePhotonVision(photonPoseEstimator2, photon2);
    }

    private void updateLimelight(){
        /* * 1. Send Orientation to Limelight 
         * Required for MegaTag2 to calculate the pose correctly.
         * We retrieve the robot's current heading from the drivetrain (Gyro).
         */
        var driveState = drivetrain.getState();
        double headingDeg = driveState.Pose.getRotation().getDegrees();
        
        LimelightHelpers.SetRobotOrientation(kLimelightName, headingDeg, 0, 0, 0, 0, 0);

        /* * 2. Read Pose Estimate 
         * getBotPoseEstimate_wpiBlue_MegaTag2 uses the orientation sent above 
         * to compute the robot's location on the field (Blue Alliance origin).
         */
        PoseEstimate llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(kLimelightName);

        if (llMeasurement != null && llMeasurement.tagCount > 0) {
            /*
             * 3. Filter and Apply
             * Reject measurements if the robot is spinning too fast to avoid camera blur/latency issues.
             */
            double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);
            if (Math.abs(omegaRps) < 2.0) {
                Matrix<N3, N1> stdDevs = getLimelightStdDevs(llMeasurement);
                drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds, stdDevs);
            }
        }
    }

    private void updatePhotonVision(PhotonPoseEstimator estimator, PhotonCamera camera){
        if (estimator == null || camera == null) return;

        //1. Get latest result from the camera
        PhotonPipelineResult result = camera.getLatestResult();
        //2. Stop if no targets
        if (!result.hasTargets()) return;
        //3. Update the estimator with camera result
        Optional<EstimatedRobotPose> estimatedPose = estimator.update(result);
        //4. Filter and apply 
        if (estimatedPose.isPresent()){
            EstimatedRobotPose camPose = estimatedPose.get();

            Matrix<N3, N1> stdDevs = getPhotonStdDevs(camPose);

            drivetrain.addVisionMeasurement(
                camPose.estimatedPose.toPose2d(),
                camPose.timestampSeconds,
                stdDevs
            );
        }

    }

    /**
     * Calculates standard deviations based on the number of tags seen and their distance.
     * Lower numbers = more trust in the vision measurement.
     */
    private Matrix<N3, N1> getLimelightStdDevs(PoseEstimate estimate) {
        double xyStds;
        double degStds;

        if (estimate.tagCount >= 2) {
            // High trust if multiple tags are visible
            xyStds = 0.5;
            degStds = 6;
        } else {
            // Scale trust based on distance to the single tag
            double dist = estimate.avgTagDist;
            if (dist > 4.0) {
                 xyStds = 3.0; 
                 degStds = 30;
            } else {
                 xyStds = 0.9 + (0.3 * dist);
                 degStds = 10 + (5 * dist);
            }
        }
        return VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds));
    }

    private Matrix<N3, N1> getPhotonStdDevs(EstimatedRobotPose estimate){
        double xyStds;
        double degStds;

        //Calculate average distace to tag for this measurement
        double avgDist = 0;
        int count = 0;
        var targets = estimate.targetsUsed;

        for (var target: targets){
            avgDist += target.getBestCameraToTarget().getTranslation().getNorm();
            count++;
        
        }
        if(count > 0)avgDist /= count;

        if (count >= 2){
            xyStds = 0.5;
            degStds = 6;
        } else {
            if (avgDist > 4.0){
                xyStds = 3.0;
                degStds = 30;
            } else {
                xyStds = 0.9 + (0.3 * avgDist);
                degStds = 10 + (5 * avgDist);
            }
        }
        return VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds));
    }
}