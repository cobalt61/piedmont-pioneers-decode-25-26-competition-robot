package org.firstinspires.ftc.teamcode.hardware;

public class Globals {
    /*
        Wrapper Class for all constants that we use.
        Each Subsystem gets its own subclass to hold constants.
        All fields should be public, static, and final.
     */

    public static class Intake {

        public static final String INTAKE_MOTOR = "bottomFlywheelAndIntake";
        public static final String INDEXER = "indexer";
        public static final int INDEX_SLEEP_TIME = 100;
        public static final String BOTTOM_INTAKE_ROLLER = "topFlywheelAndBottomRoller";


        public static final double POWER_OFF = 0;
        public static final double POWER_ON = 1;

//        public static final int SENSOR_RED_THRESHOLD = 300;
//        public static final int SENSOR_YELLOW_THRESHOLD_RED = 300;
//        public static final int SENSOR_YELLOW_THRESHOLD_GREEN = 300;
//        public static final int SENSOR_BLUE_THRESHOLD = 300;
    }

    public static final class Outtake {
        public static final String BOTTOM_FLYWHEEL = "bottomFlywheelAndIntake";
        public static final String TOP_FLYWHEEL = "topFlywheelAndBottomRoller";
       public static final String TOP_FLYWHEEL_2 = "topFlywheelTopGear";
        public static final String LIMIT_SWITCH = "v";
        public static final String INDEXER = "indexer";
        public static final double INDEXER_SPINNING_POWER = 1;
        public static final double INDEXER_SPINNING_POWER_RESTING = 0;
        public static final double POWER_ON = 1;
        public static final double POWER_OFF = 0;

    }

    public static final class Drive {
        public static final String LEFT_FRONT_DRIVE = "left_front_drive";
        public static final String RIGHT_FRONT_DRIVE = "right_front_drive";
        public static final String LEFT_BACK_DRIVE = "left_back_drive";
        public static final String RIGHT_BACK_DRIVE = "right_back_drive";

        public static final double MAX_SPEED = 0.5;
        public static final double WHEEL_LOCK_MIN = 0.2;
    }
    public static final class Auto{
        public static final String LEFT_FRONT_DRIVE = "left_front_drive";
        public static final String RIGHT_FRONT_DRIVE = "right_front_drive";
        public static final String LEFT_BACK_DRIVE = "left_back_drive";
        public static final String RIGHT_BACK_DRIVE = "right_back_drive";
        public static final double     FORWARD_SPEED = 0.6;
       public static final double     TURN_SPEED    = 0.5;
    }
}
