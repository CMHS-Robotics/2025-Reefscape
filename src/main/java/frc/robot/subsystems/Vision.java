package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.tools.PID;

public class Vision extends SubsystemBase {
    PhotonCamera Left = new PhotonCamera("LeftCamera");
    PhotonCamera Right = new PhotonCamera("RightCamera");
    PhotonCamera Back = new PhotonCamera("BackCamera");
    PhotonCamera Front = new PhotonCamera("FrontCamera");

    PhotonCamera[] Cameras = {Front,Right,Back,Left};
    
    PhotonTrackedTarget target;
    List<PhotonPipelineResult> results;
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


    public Vision(){
        turnTrackingPID = new PID(0.5,0,0.1);
    }

    
    public PhotonTrackedTarget getTarget(){
        return target;
    }

    public List<PhotonPipelineResult> getResults(){
        return results;
    }

    public boolean getTargetVisible(){
        hasTarget();
        return targetVisible;
    }
    
    public boolean getTargetVisible(int id){
        hasTarget(id);
        return targetVisible;
    }
    public boolean getTargetVisible(CAMERA cam, int id){
        hasTarget(cam,id);
        return targetVisible;
    }


    public boolean hasTarget(){
        for (int i = 0; i<3; i ++){
            var results = Cameras[i].getAllUnreadResults();
            if(!results.isEmpty()){
                var result = results.get(results.size()-1);
                if(result.hasTargets()){
                    targetVisible = true;
                    this.results = results;
                    return true;
                }    
            }
        }
                return false;
    }    
    
    public boolean hasTarget(CAMERA camera){
            var results = Cameras[camera.getId()].getAllUnreadResults();
            if(!results.isEmpty()){
                var result = results.get(results.size()-1);
                if(result.hasTargets()){
                    targetVisible = true;
                    this.results = results;
                    return true;
                }    
            }
                return false;
    }

    public boolean hasTarget(int id){
        for (int i = 0; i<3;i ++){
            var results = Cameras[i].getAllUnreadResults();
            if(!results.isEmpty()){
                var result = results.get(results.size()-1);
                if(result.hasTargets()){
                    for(var target : results.get(results.size()-1).getTargets()){
                        if(target.getFiducialId() == id){
                            this.target = target;
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
        var results = Cameras[camera.getId()].getAllUnreadResults();
        if(!results.isEmpty()){
            var result = results.get(results.size()-1);
            if(result.hasTargets()){
                for(var target : results.get(results.size()-1).getTargets()){
                    if(target.getFiducialId() == id){
                        this.target = target;
                        targetVisible = true;
                        return true;
                    }
                }
            }
        }
                return false;
    }

}
