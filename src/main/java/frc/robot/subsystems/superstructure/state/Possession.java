package frc.robot.subsystems.superstructure.state;

public final class Possession {

    public static enum PossessionState {
        EMPTY,
        CORAL,
        ALGAE,
        FORBIDDEN // Represents an illegal or impossible possession state
    }

    //Represents the change in possession resulting from a given superstate
    public static class PossessionMap {

        private final PossessionState EmptyMapping;
        private final PossessionState CoralMapping;
        private final PossessionState AlgaeMapping;

        public PossessionMap(PossessionState EmptyMapping, PossessionState CoralMapping, PossessionState AlgaeMapping) {
            this.EmptyMapping = EmptyMapping;
            this.CoralMapping = CoralMapping;
            this.AlgaeMapping = AlgaeMapping;
        }

        public PossessionState map(PossessionState start) {
            switch (start) {
                case EMPTY:
                    return EmptyMapping;
                case CORAL:
                    return CoralMapping;
                case ALGAE:
                    return AlgaeMapping;
                case FORBIDDEN:
                    return PossessionState.FORBIDDEN; // Theoretically should never be called
                default:
                    return PossessionState.FORBIDDEN; // Theoretically should never be called
            }
        }
    }

    public static final PossessionMap kNoChange = new PossessionMap(PossessionState.EMPTY,PossessionState.CORAL,PossessionState.ALGAE);
    public static final PossessionMap kAlgaeIntake = new PossessionMap(PossessionState.ALGAE,PossessionState.FORBIDDEN,PossessionState.FORBIDDEN);
    public static final PossessionMap kCoralIntake = new PossessionMap(PossessionState.CORAL,PossessionState.FORBIDDEN,PossessionState.FORBIDDEN);
    public static final PossessionMap kOuttake = new PossessionMap(PossessionState.EMPTY,PossessionState.EMPTY,PossessionState.EMPTY);

    private Possession() {}
}
