package frc.robot.subsystems.superstructure;

public final class SuperstructureEnums {
    private SuperstructureEnums() {}

    public enum Goal {
        IDLE,
        INTAKE,
        SHOOT,
        SNOWBLOW,
        OUTTAKE
    }

    public enum State {
        IDLING(false, false),
        INTAKING(false, false),
        SHOOTING_SPINUP(false, true),
        SHOOTING_FEED(true, true),
        SNOWBLOW_SPINUP(false, true),
        SNOWBLOW_FEED(true, true),
        OUTTAKING(false, false),
        CLEARING_KICKER(false, false);

        private final boolean feeding;
        private final boolean shooting;

        private State(boolean feeding, boolean shooting) {
            this.feeding = feeding;
            this.shooting = shooting;
        }

        public boolean isFeeding() {
            return feeding;
        }

        public boolean isShooting() {
            return shooting;
        }
    }

    public enum Mode {
        AUTO,
        MANUAL
    }
}
