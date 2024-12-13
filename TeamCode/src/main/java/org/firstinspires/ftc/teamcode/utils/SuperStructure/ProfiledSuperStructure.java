package org.firstinspires.ftc.teamcode.utils.SuperStructure;

public class ProfiledSuperStructure {
    private final ProfiledMechanism[] profiledMechanisms;
    public final double[] positionSetPoints;

    public ProfiledSuperStructure(ProfiledMechanism[] profiledMechanisms, double[] initialPositions) {
        assert profiledMechanisms.length == initialPositions.length
                : "Mechanisms and Positions Length Not Match";
        this.profiledMechanisms = profiledMechanisms;
        this.positionSetPoints = initialPositions;
    }

    public void requestPositions(SetPoints setPoints) {
        System.arraycopy(setPoints.positions, 0, positionSetPoints, 0, positionSetPoints.length);
    }

    public void update() {
        for (ProfiledMechanism mechanism:profiledMechanisms)
            mechanism.update();
    }
    public void update(double dt) {
        for (ProfiledMechanism mechanism:profiledMechanisms)
            mechanism.update(dt);
    }

    public boolean atReference() {
        for (ProfiledMechanism mechanism:profiledMechanisms)
            if (!mechanism.atReference()) return false;
        return true;
    }

    public final class SetPoints {
        private final double[] positions;
        public SetPoints(double... positions) {
            assert positions.length == positionSetPoints.length
                    : "Mechanisms and Positions Length Not Match";
            this.positions = positions;
        }
    }
}
