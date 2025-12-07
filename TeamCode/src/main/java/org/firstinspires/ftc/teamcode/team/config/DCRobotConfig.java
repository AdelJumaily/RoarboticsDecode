package org.firstinspires.ftc.teamcode.team.config;

/**
 * Centralized configuration for DC Robot.
 * All constants, motor powers, and configuration values should be defined here.
 */
public final class DCRobotConfig {
    private DCRobotConfig() {
        // Utility class - prevent instantiation
    }

    /**
     * Motor power configurations
     */
    public static final class MotorPowers {
        private MotorPowers() {}

        // Intake powers
        public static final double INTAKE_POWER = 0.55;
        public static final double OUTTAKE_POWER = -0.65;
        public static final double INTAKE_IDLE = 0.0;

        // Shooter powers
        public static final double SHOOTER_POWER = -0.65;
        public static final double SHOOTER_IDLE = 0.0;
    }

    /**
     * Lift position configurations (in encoder ticks or inches)
     */
    public static final class LiftPositions {
        private LiftPositions() {}

        public static final double LIFT_OUT = 0.0;
        public static final double LIFT_IN = 10.0;
        public static final double LIFT_HOME = 0.0;
    }

    /**
     * Drive speed multipliers
     */
    public static final class DriveSpeed {
        private DriveSpeed() {}

        public static final double NORMAL = 1.0;
        public static final double SLOW = 0.7;
    }

    /**
     * Lift control constants
     */
    public static final class LiftControl {
        private LiftControl() {}

        // Extend PID constants
        public static final double EXTEND_P = 0.4;
        public static final double EXTEND_I = 0.0;
        public static final double EXTEND_D = 0.00;
        public static final double EXTEND_S = 0.05;
        public static final double EXTEND_V = (1 - 0.05) / 15.0;
        public static final double EXTEND_A = 0.0;

        // Retract PID constants
        public static final double RETRACT_P = 0.08;
        public static final double RETRACT_I = 0.0;
        public static final double RETRACT_D = 0.0;
        public static final double RETRACT_S = 0.05;
        public static final double RETRACT_V = 1.0 / 55.0;
        public static final double RETRACT_A = 0.0;

        // Setpoint tolerance
        public static final double SETPOINT_TOLERANCE = 0.25; // 1/4 inch
    }
}

