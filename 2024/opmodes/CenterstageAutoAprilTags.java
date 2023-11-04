
package opmodes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;

import team25core.ObjectDetectionNewTask;
import team25core.Robot;
import team25core.RobotEvent;

@Autonomous(name = "AprilTagAuto")
public class CenterstageAutoAprilTags extends Robot {
    private ObjectDetectionNewTask objDetectionTask;
    private final static String TAG = "Prop";

    public String position = "right";

    private int desireTagID;

    @Override
    public void handleEvent(RobotEvent e)
    {
        /*
         * Every time we complete a segment drop a note in the robot log.
         */
        //if (e instanceof DeadReckonTask.DeadReckonEvent) {
        //  RobotLog.i("Completed path segment %d", ((DeadReckonTask.DeadReckonEvent)e).segment_num);
        }

    public void findAprilTag()
    {
        RobotLog.ii(TAG, "Setup findAprilTag");
        objDetectionTask = new ObjectDetectionNewTask(this, telemetry, ObjectDetectionNewTask.DetectionKind.APRILTAG_DETECTED){
            @Override
            public void handleEvent(RobotEvent e) {
                ObjectDetectionEvent event = (ObjectDetectionEvent) e;
                switch (event.kind) {
                    case OBJECTS_DETECTED:
                        int numDetection = getNumAprilTags();
                        for (int i = 0; i<numDetection; i++){
                            getAprilTagDetections(i);

                        }
                        RobotLog.ii(TAG, "Object detected");
                        break;
                    }
                }
            };
            objDetectionTask.init(telemetry, hardwareMap);
            objDetectionTask.rateLimit(1000); // currently calling objDetectionTask every second
            objDetectionTask.start();
            objDetectionTask.resumeStreaming();
            addTask(objDetectionTask);
        }
        public void findDesireID()
        {
            if(position.equals("right")){
                desireTagID = 3;
            }
            else if(position.equals("left")){
                desireTagID = 1;
            }
            else
            {
                desireTagID = 2;
            }

        }
    @Override
    public void init(){

    }
    @Override
    public void start(){
        findAprilTag();
    }
}