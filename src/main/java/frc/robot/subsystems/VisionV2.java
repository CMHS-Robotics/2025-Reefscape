package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;


// added this??
import edu.wpi.first.math.geometry.Transform3d;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class VisionV2 extends SubsystemBase {

    // Cameras
    private PhotonCamera FrontLeftCamera = new PhotonCamera("FrontLeftCamera");
    private PhotonCamera FrontRightCamera = new PhotonCamera("FrontRightCamera");

    // Stored results
    private PhotonTrackedTarget targetL;
    private PhotonTrackedTarget targetR;
    private int targetIDL = -1;
    private int targetIDR = -1;
    private boolean targetsMatch = false;

    private final AprilTagFieldLayout fieldLayout;

    // Swerve + Driver (kept in case you need later)
    private final CommandSwerveDrivetrain swerve;
    private final CommandXboxController driver;

    public VisionV2(CommandSwerveDrivetrain s, CommandXboxController d) {
        this.swerve = s;
        this.driver = d;

        fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    }

    public int GetCameraTarget() {
        var resultL = FrontLeftCamera.getLatestResult();
        var resultR = FrontRightCamera.getLatestResult();

        if (resultL.hasTargets()) {
            targetL = resultL.getBestTarget();
            targetIDL = targetL.getFiducialId();
        } else {
            targetIDL = -1;
        }

        if (resultR.hasTargets()) {
            targetR = resultR.getBestTarget();
            targetIDR = targetR.getFiducialId();
        } else {
            targetIDR = -1;
        }

        // Match check
        if (targetIDL != -1 && targetIDL == targetIDR) {
            targetsMatch = true;
        } else {
            targetsMatch = false;
        }

        return targetIDL;
    }

    // this causes code to have a fatal error :(

    public Rotation2d CalcRotation(PhotonTrackedTarget target) {
        // If there’s no target passed in, just return 0° rotation error
        if (target == null) {
            return Rotation2d.fromDegrees(0);
        }

        AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    
        int seenTagId = target.getFiducialId();
    
        var tagMap = fieldLayout.getTags();
    
        var tagDefinition = tagMap.get(seenTagId);

        if (tagDefinition == null) {
            return Rotation2d.fromDegrees(0);
        }
        Rotation2d robotHeading = swerve.getState().Pose.getRotation();

        Rotation2d tagHeading = tagDefinition.pose.getRotation().toRotation2d();

        Rotation2d rotationError = tagHeading.minus(robotHeading);
    
        return rotationError;
    }



    public Transform3d getTagTransform() {
        var result = FrontLeftCamera.getLatestResult();
        if (!result.hasTargets()) return null;
    
        PhotonTrackedTarget target = result.getBestTarget();
        return target.getBestCameraToTarget();
    }

//bruh


    @Override
    public void periodic() {
        GetCameraTarget();

        SmartDashboard.putBoolean("HasSameTargetId", targetsMatch);
        SmartDashboard.putNumber("LeftTargetID", targetIDL);
        SmartDashboard.putNumber("RightTargetID", targetIDR);
        SmartDashboard.putNumber("VerifiedTargetID", targetsMatch ? targetIDL : -1);
        SmartDashboard.putNumber("LROTERROR", CalcRotation(targetL).getRadians());

    }
}
