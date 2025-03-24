package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.tools.PID;

public class Vision extends SubsystemBase {
    // PhotonCamera Left = new PhotonCamera("LeftCamera");
    // PhotonCamera Right = new PhotonCamera("RightCamera");
    // PhotonCamera Back = new PhotonCamera("BackCamera");
    PhotonCamera Front = new PhotonCamera("FrontCamera");
    CommandSwerveDrivetrain swerve;

    PhotonPoseEstimator FrontPoseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded),PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d(0.0,0.0,0.0,Rotation3d.kZero));

    // public PhotonCamera[] Cameras = {Front,Right,Back,Left};
    public PhotonCamera[] Cameras = {Front};

    PhotonTrackedTarget FrontTarget;
    PhotonTrackedTarget RightTarget;
    PhotonTrackedTarget BackTarget;
    PhotonTrackedTarget LeftTarget;
    List<PhotonTrackedTarget> Targets = new ArrayList<PhotonTrackedTarget>(4);
    List<PhotonPipelineResult> FrontResults;
    List<PhotonPipelineResult> RightResults;
    List<PhotonPipelineResult> BackResults;
    List<PhotonPipelineResult> LeftResults;
    List<List<PhotonPipelineResult>> ResultsList = new ArrayList<List<PhotonPipelineResult>>(4);
    boolean targetVisible;

    public PID turnTrackingPID;

    public enum CAMERA{
        FRONT(0),
        RIGHT(1),
        BACK(2),
        LEFT(3);
            
        private int cameraId;
        CAMERA(int id){
            cameraId = id;
        }

        public int getId(){
            return cameraId;
        }
    }

    public Vision(CommandSwerveDrivetrain s){
        swerve = s;
        turnTrackingPID = new PID(0.5,0,0.1);
        FrontResults = Front.getAllUnreadResults();
        ResultsList.add(FrontResults);
        ResultsList.add(RightResults);
        ResultsList.add(BackResults);
        ResultsList.add(LeftResults);
        Targets.add(FrontTarget);
        Targets.add(RightTarget);
        Targets.add(BackTarget);
        Targets.add(LeftTarget);
    }

    @Override
    public void periodic(){

        var visionEst = estimatePose();

        var fr = Front.getAllUnreadResults();
        if(!fr.isEmpty()){
            FrontResults=fr;
            visionEst = FrontPoseEstimator.update(fr.get(0));
        }
        // RightResults = Right.getAllUnreadResults();
        // BackResults = Back.getAllUnreadResults();
        // LeftResults = Left.getAllUnreadResults();

        ResultsList.set(0, FrontResults);
        ResultsList.set(1, RightResults);
        ResultsList.set(2, BackResults);
        ResultsList.set(3, LeftResults);

        SmartDashboard.putBoolean("Vision Estimate Present",visionEst.isPresent());
        

        visionEst.ifPresent(
            est -> {
            swerve.addVisionMeasurement(
                    est.estimatedPose.toPose2d(), est.timestampSeconds);
        
        });
    
    }

    public Optional<EstimatedRobotPose> estimatePose(){
        Optional<EstimatedRobotPose> est = Optional.empty();

        for(var result : ResultsList.get(0)){
            est = FrontPoseEstimator.update(result);
        }

        return est;
    }
    
    public PhotonTrackedTarget getTarget(CAMERA cam){
        return Targets.get(cam.getId());
    }
    
    public List<PhotonPipelineResult> getResults(CAMERA cam){
         return ResultsList.get(cam.getId());
    }

    public boolean hasTarget(){
        for (int i = 0; i<3; i ++){
            var results = ResultsList.get(i);
            if(!results.isEmpty()){
                var result = results.get(results.size()-1);
                if(result.hasTargets()){
                    targetVisible = true;
                    Targets.set(i,result.getBestTarget());
                    return true;
                }    
            }
        }
        return false;
    }    
    
    public boolean hasTarget(CAMERA camera){
        var results = ResultsList.get(camera.getId());
        if(!results.isEmpty()){
            var result = results.get(results.size()-1);
            if(result.hasTargets()){
                targetVisible = true;
                Targets.set(camera.getId(),result.getBestTarget());
                return true;
            }    
        }
        return false;
    }

    public boolean hasTarget(int id){
        for (int i = 0; i<3;i ++){
            var results = ResultsList.get(i);
            if(!results.isEmpty()){
                var result = results.get(results.size()-1);
                if(result.hasTargets()){
                    for(var target : results.get(results.size()-1).getTargets()){
                        if(target.getFiducialId() == id){
                            Targets.set(i,target);
                            targetVisible = true;
                            return true;
                        }
                    }
                }
            }
        }
        return false;
    }

    public boolean hasTarget(CAMERA camera, int id){
        var results = ResultsList.get(camera.getId());
        if(!results.isEmpty()){
            var result = results.get(results.size()-1);
            if(result.hasTargets()){
                for(var target : results.get(results.size()-1).getTargets()){
                    if(target.getFiducialId() == id){
                        Targets.set(camera.getId(),target);
                        targetVisible = true;
                        return true;
                    }
                }
            }
        }
        return false;
    }

}
