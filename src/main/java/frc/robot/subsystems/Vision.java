package frc.robot.subsystems;

import java.lang.reflect.Array;

import static edu.wpi.first.units.Units.Rotation;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SetVisionPIDToTargetRotationAndCenterCommand;
import frc.robot.commands.SetVisionPIDToTargetRotationCommand;
import frc.robot.commands.SetVisionPIDToTargetYawCommand;
import frc.robot.tools.PID;

public class Vision extends SubsystemBase {
    PhotonCamera Left = new PhotonCamera("LeftCamera");
    PhotonCamera Right = new PhotonCamera("RightCamera");
    PhotonCamera Top = new PhotonCamera("TopCamera");
    PhotonCamera Front = new PhotonCamera("FrontCamera");
    CommandSwerveDrivetrain swerve;

    //create the pose estimators
    //the transforms need to be accurate to exactly where the cameras are on the robot
    PhotonPoseEstimator FrontPoseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded),PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d(0.2,0.0,0.1,Rotation3d.kZero));
    PhotonPoseEstimator RightPoseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded),PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d(0.2,-0.1,0.1,new Rotation3d(Rotation2d.fromDegrees(30))));
    PhotonPoseEstimator TopPoseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded),PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d(0.0,0.0,0.5,new Rotation3d(Rotation2d.k180deg)));
    PhotonPoseEstimator LeftPoseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded),PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d(0.2,0.1,0.1,new Rotation3d(Rotation2d.fromDegrees(-30))));

    public PhotonCamera[] Cameras = {Front,Right,Top,Left};

    //create lists for targets and results for each of the cameras
    public PhotonTrackedTarget FrontTarget;
    PhotonTrackedTarget RightTarget;
    PhotonTrackedTarget TopTarget;
    PhotonTrackedTarget LeftTarget;
    public List<PhotonTrackedTarget> TargetsList = new ArrayList<>(4);
    public List<PhotonPipelineResult> FrontResults;
    List<PhotonPipelineResult> RightResults;
    List<PhotonPipelineResult> TopResults;
    List<PhotonPipelineResult> LeftResults;
    public List<List<PhotonPipelineResult>> ResultsList = new ArrayList<>(4);

    //pid for the turn locking and center locking
    public PID turnTrackingPID;
    public PID xTranslatePID;
    public PID yTranslatePID;

    CommandXboxController driver;

    public MODE currentMode = MODE.LOCKVECTOR;

    //enum allows different cameras to just be defined by a word
    public enum CAMERA{
        FRONT(0),
        RIGHT(1),
        TOP(2),
        LEFT(3);
            
        private int cameraId;
        CAMERA(int id){
            cameraId = id;
        }

        public int getId(){
            return cameraId;
        }
    }

    public Vision(CommandSwerveDrivetrain s, CommandXboxController d){
        swerve = s;
        driver = d;
        turnTrackingPID = new PID(0.03,0,0.01);
        turnTrackingPID.setMinInput(-180);
        turnTrackingPID.setMaxInput(180);
        xTranslatePID = new PID(0.03,0,0.01);
        yTranslatePID = new PID(0.03,0,0.01);
        
        FrontResults = Front.getAllUnreadResults();
        RightResults = Right.getAllUnreadResults();
        // TopResults = Top.getAllUnreadResults();
        LeftResults = Left.getAllUnreadResults();
        ResultsList.add(FrontResults);
        ResultsList.add(RightResults);
        ResultsList.add(TopResults);
        ResultsList.add(LeftResults);
        TargetsList.add(FrontTarget);
        TargetsList.add(RightTarget);
        TargetsList.add(TopTarget);
        TargetsList.add(LeftTarget);

        SetVisionPIDToTargetYawCommand setPIDFromYaw = new SetVisionPIDToTargetYawCommand(this);
        SetVisionPIDToTargetRotationCommand setPIDFromRot = new SetVisionPIDToTargetRotationCommand(this);
        SetVisionPIDToTargetRotationAndCenterCommand setPIDFromRotAndCenter = new SetVisionPIDToTargetRotationAndCenterCommand(this);

        driver.rightBumper().and(() -> currentMode.equals(MODE.LOCKVECTOR)).whileTrue(setPIDFromRotAndCenter);

        driver.rightBumper().and(() -> currentMode.equals(MODE.LOCKANGLE)).whileTrue(setPIDFromRot);

        driver.rightBumper().and(() -> currentMode.equals(MODE.YAWTARGET)).whileTrue(setPIDFromYaw);
        //Note:
        //map the lock on function to another button:X
        //create new or expand mode enum
        //Get Coords When Locked, Tell It To Center There, May Need Offset
        //Set State Upon Moved To ReadyLRAct
        //Restrict All Movement
        //When Mode=ReadyLRAct, Left Bumper, move offset to left, vice versa.
        //Lock All Input Except X, Until Moved To Target, Set State: ReadyElv-unlock elevator input.
        //May need to move back slightly... test to confirm
        //Configure Coral Wrist For Better Angles, The Robot Needs To Be Flush On Surface For This:easier for driver if done manual
        //Maybe use X to Enter And Exit Lock Mode, Auto Exit Upon Completion.
        //Make Maximum Distance For Lock On, 5 meters:code will already grab best target.

    }

    public enum MODE{
        YAWTARGET(2),
        LOCKANGLE(1),
        LOCKVECTOR(0);

        private int modeid;
        MODE(int id){
            modeid = id;
        }
        private int getId(){
            return modeid;
        }

        static public MODE getModeOfId(int i){
            return switch (i){
            case 0 -> LOCKVECTOR;
               case 1 -> LOCKANGLE;
               case 2 -> YAWTARGET;
               default-> YAWTARGET;
            };
        }
    }

    public enum CVState{
        MOVETOTARG(4),//moving to target
        ATTARG(3),//At Target
        TARGL(2),//At Target Left
        TARGR(1),//At Target Right
        FREE(0);//Free State, Can Freely Move, Default State

        private int cvid;
        CVState(int id) {
            cvid = id;
        }
        private int getId(){
            return cvid;
        }

        static public CVState getStateOfId(int i){
            return switch(i){
                case 0 -> FREE;
                case 1 -> TARGR;
                case 2 -> TARGL;
                case 3 -> ATTARG;
                case 4 -> MOVETOTARG;
                default -> FREE;
            };
        }
        
    
    }
    public Pose2d getRobotPose(){
        SmartDashboard.putNumber("Robot Rotation",swerve.getState().Pose.getRotation().getDegrees());
        return swerve.getState().Pose;
        }

    public Pose3d getTargetPose(PhotonTrackedTarget target){
        return AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getTags().get(target.getFiducialId()).pose;
    }

    public MODE getCurrentMode(){
        return currentMode;
    }

    public void setCurrentMode(MODE mode){
        currentMode = mode;
    }
    public void setCurrentMode(int modeid){
        currentMode = MODE.getModeOfId(modeid);
    }

    public Rotation2d CalcRotation(PhotonTrackedTarget target){
        Rotation2d robotHeading = swerve.getState().Pose.getRotation();
        Rotation2d targHeading = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getTags().get(target.getFiducialId()).pose.getRotation().toRotation2d();

        Rotation2d RotError = targHeading.minus(robotHeading);

        return RotError;
    }

    public Translation2d CalcTranslation(){
        Pose2d robotpose = getRobotPose();
        Pose3d targpose = getTargetPose(FrontTarget);
        
        Translation2d PoseError = targpose.getTranslation().toTranslation2d().minus(robotpose.getTranslation());

        return PoseError;
    }

    @Override
    public void periodic(){

        //update all the results list with new results

        var FrontVisionEst = estimatePose(CAMERA.FRONT);

        var fr = Front.getAllUnreadResults();
        if(!fr.isEmpty()){
            FrontResults=fr;
            FrontVisionEst = FrontPoseEstimator.update(fr.get(0));
        }

        var RightVisionEst = estimatePose(CAMERA.RIGHT);
        
        var rr = Right.getAllUnreadResults();
        if(!rr.isEmpty()){
            RightResults=rr;
            RightVisionEst = RightPoseEstimator.update(rr.get(0));
        }

        //var TopVisionEst = estimatePose(CAMERA.Top);
        
        var br = Top.getAllUnreadResults();
        if(!br.isEmpty()){
            TopResults=br;
            //TopVisionEst = TopPoseEstimator.update(br.get(0));
        }

        var LeftVisionEst = estimatePose(CAMERA.LEFT);
        
        var lr = Right.getAllUnreadResults();
        if(!lr.isEmpty()){
            LeftResults=lr;
            LeftVisionEst = LeftPoseEstimator.update(lr.get(0));
        }

        ResultsList.set(0, FrontResults);
        ResultsList.set(1, RightResults);
        ResultsList.set(2, TopResults);
        ResultsList.set(3, LeftResults);

        //do all the vision estimates

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

        // TopVisionEst.ifPresent(
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
    
    //getters for targets and results
    public PhotonTrackedTarget getTarget(CAMERA cam){
        return TargetsList.get(cam.getId());
    }
    
    public List<PhotonPipelineResult> getResults(CAMERA cam){
         return ResultsList.get(cam.getId());
    }

    //bunch of overloaded methods for different circumstances that check for targets
        //if a camera isnt specified, it traverses the whole list of results
        //if an id is specified, it will update the target list with the target with that id
        //if an id is not specified, it will update the target list with getBestTarget
    public boolean hasTarget(){
        for (int i = 0; i<3; i ++){
            var results = ResultsList.get(i);
            if(!results.isEmpty()){
                var result = results.get(results.size()-1);
                if(result.hasTargets()){
                    TargetsList.set(i,result.getBestTarget());
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
                TargetsList.set(camera.getId(),result.getBestTarget());
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
                            TargetsList.set(i,target);
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
                        TargetsList.set(camera.getId(),target);
                        return true;
                    }
                }
            }
        }
        return false;
    }

}
