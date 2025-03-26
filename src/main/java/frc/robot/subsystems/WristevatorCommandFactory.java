package frc.robot.subsystems;

import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

public class WristevatorCommandFactory {
    private final ElevatorSubsystem m_elevator;
    private final WristSubsystem m_wrist;
    public WristevatorCommandFactory(ElevatorSubsystem elevator, WristSubsystem wrist) {
        m_elevator = elevator;
        m_wrist = wrist;
    }
}
