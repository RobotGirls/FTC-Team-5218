//package opmodes;
//import com.qualcomm.robotcore.util.RobotLog;
//
//import team25core.ObjectDetectionNewTask;
//import team25core.Robot;
//import team25core.RobotEvent;
//
//
//public class CenterStageAuto extends Robot {
//    private ObjectDetectionNewTask objDetectionTask;
//    private final static String TAG = "Prop";
//    ObjectDetectionNewTask objDetectionTask;
//
//    public void detectProp()
//    {
//        RobotLog.ii(TAG, "Setup detectProp");
//        objDetectionTask = new ObjectDetectionNewTask(this){
//            @Override
//            public void handleEvent(RobotEvent e) {
//                ObjectDetectionEvent event = (ObjectDetectionEvent) e;
//                switch (event.kind) {
//                    case OBJECTS_DETECTED:
//                        RobotLog.ii(TAG, "Object detected");
//                        break;
//                }
//            }
//        };
//        //FIXME figure out where we want to do the addTask
//        //addTask(new SkystoneDetectionTask(this, purpleColorSensor, purpleDistanceSensor) {
//
//}
//
//
//
//    @Override
//   public void init(){
//
//   }
//   @Override
//   public void start(){
//       objDetectionTask = new ObjectDetectionNewTask();
//
//   }
//
//
//}
