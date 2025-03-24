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
    PhotonCamera Left = new PhotonCamera("LeftCamera");
    PhotonCamera Right = new PhotonCamera("RightCamera");
    PhotonCamera Back = new PhotonCamera("BackCamera");
    PhotonCamera Front = new PhotonCamera("FrontCamera");
    CommandSwerveDrivetrain swerve;

    PhotonPoseEstimator FrontPoseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded),PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d(0.2,0.0,0.0,Rotation3d.kZero));
    PhotonPoseEstimator RightPoseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded),PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d(0.0,0.0,0.0,new Rotation3d(Rotation2d.kCW_90deg)));
    PhotonPoseEstimator BackPoseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded),PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d(0.0,0.0,0.0,new Rotation3d(Rotation2d.k180deg)));
    PhotonPoseEstimator LeftPoseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded),PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d(0.0,0.0,0.0,new Rotation3d(Rotation2d.kCCW_90deg)));

    public PhotonCamera[] Cameras = {Front,Right,Back,Left};

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

        var FrontVisionEst = estimatePose(CAMERA.FRONT);

        var fr = Front.getAllUnreadResults();
        if(!fr.isEmpty()){
            FrontResults=fr;
            FrontVisionEst = FrontPoseEstimator.update(fr.get(0));
        }

        //var RightVisionEst = estimatePose(CAMERA.RIGHT);
        
        var rr = Right.getAllUnreadResults();
        if(!rr.isEmpty()){
            RightResults=rr;
            //RightVisionEst = RightPoseEstimator.update(rr.get(0));
        }

        //var BackVisionEst = estimatePose(CAMERA.BACK);
        
        var br = Back.getAllUnreadResults();
        if(!br.isEmpty()){
            BackResults=br;
            //BackVisionEst = BackPoseEstimator.update(br.get(0));
        }

        //var LeftVisionEst = estimatePose(CAMERA.LEFT);
        
        var lr = Right.getAllUnreadResults();
        if(!lr.isEmpty()){
            LeftResults=lr;
            //LeftVisionEst = LeftPoseEstimator.update(lr.get(0));
        }

        ResultsList.set(0, FrontResults);
        ResultsList.set(1, RightResults);
        ResultsList.set(2, BackResults);
        ResultsList.set(3, LeftResults);

        FrontVisionEst.ifPresent(
            est -> {
            swerve.addVisionMeasurement(
                    est.estimatedPose.toPose2d(), est.timestampSeconds);
        
        });

        // RightVisionEst.ifPresent(
        //     est -> {
        //     swerve.addVisionMeasurement(
        //             est.estimatedPose.toPose2d(), est.timestampSeconds);
        
        // });

        // BackVisionEst.ifPresent(
        //     est -> {
        //     swerve.addVisionMeasurement(
        //             est.estimatedPose.toPose2d(), est.timestampSeconds);
        
        // });

        // LeftVisionEst.ifPresent(
        //     est -> {
        //     swerve.addVisionMeasurement(
        //             est.estimatedPose.toPose2d(), est.timestampSeconds);
        
        // });
    
    }

    public Optional<EstimatedRobotPose> estimatePose(CAMERA cam){
        Optional<EstimatedRobotPose> est = Optional.empty();

        for(var result : ResultsList.get(cam.getId())){
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
