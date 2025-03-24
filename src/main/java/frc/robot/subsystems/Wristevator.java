// // Copyright (c) FRC 1076 PiHi Samurai
// // You may use, distribute, and modify this software under the terms of
// // the license found in the root directory of this project

// package frc.robot.subsystems;

// import java.util.function.BooleanSupplier;
// import java.util.function.Supplier;

// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import frc.robot.Constants.SuperstructureConstants.WristevatorState;
// import frc.robot.subsystems.elevator.ElevatorSubsystem;
// import frc.robot.subsystems.wrist.WristSubsystem;

// public class Wristevator {
//     public final ElevatorSubsystem m_elevator;
//     public final WristSubsystem m_wrist;

//     public Wristevator(ElevatorSubsystem elevator, WristSubsystem wrist) {
//         m_elevator = elevator;
//         m_wrist = wrist;
//     }

//     public void setKgConstants(double elevator_kG, double wrist_kG) {
//         m_elevator.setKg(elevator_kG);
//         m_wrist.setKg(wrist_kG);
//     }

//     /**
//      * Sequentially sets wrist to a safe angle, moves elevator to desired state, then sets wrist to desired state
//      */
//     public Command applyStateSafe(WristevatorState state, Supplier<Rotation2d> safeAngleSupplier) {
//         return Commands.sequence(
//             m_wrist.applyAngle(safeAngleSupplier.get()),
//             Commands.race(
//                 m_elevator.applyPosition(state.elevatorHeightMeters),
//                 m_wrist.holdAngle(safeAngleSupplier.get())
//             ),
//             m_wrist.applyAngle(state.wristAngle)
//         );
//     }

//     /** Allows the user to define a deploy safe height, so that the wrist can begin deploying as quickly as possible */
//     public Command applyStateDynamic(WristevatorState state, Supplier<Rotation2d> safeAngle, BooleanSupplier goingUp, double wristDeploySafeHeightMeters) {
//         return Commands.either(
//             Commands.parallel(
//                 m_elevator.applyPosition(state.elevatorHeightMeters),
//                 Commands.waitUntil(() -> m_elevator.getPositionMeters() >= wristDeploySafeHeightMeters)
//                     .andThen(m_wrist.applyAngle(state.wristAngle))
//             ).beforeStarting(m_wrist.applyAngle(safeAngle.get())
//                 .onlyIf(() -> m_elevator.getPositionMeters() < wristDeploySafeHeightMeters)
//             ),
//             Commands.sequence(
//                 Commands.parallel(
//                     m_elevator.applyPosition(wristDeploySafeHeightMeters),
//                     m_wrist.applyAngle(safeAngle.get())
//                 ).onlyIf(() -> m_elevator.getPositionMeters() > wristDeploySafeHeightMeters),
//                 Commands.parallel(
//                     m_elevator.applyPosition(state.elevatorHeightMeters),
//                     m_wrist.applyAngle(state.wristAngle)
//                 )
//             ),
//             goingUp
//         );
//     }
// }
