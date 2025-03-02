// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import lib.extendedcommands.CommandUtils;

/*
 * This class is used to create mechanism visualizations and pass them into IOSim objects
 */
public class SuperstructureVisualizer {
    
    private final Mechanism2d superstructureVis;
    private final MechanismRoot2d elevatorRoot;
    private final MechanismLigament2d elevator;
    private final MechanismLigament2d fixed;
    private final MechanismLigament2d wrist;
    private final Superstructure superstructure;

    public SuperstructureVisualizer(Superstructure superstructure){
        this.superstructure = superstructure;
        // Create the canvas for mechanisms
        // Width is side frame perimeter (30 inches), height is max elevator height + some buffer
        superstructureVis = new Mechanism2d(0.762, 3);
        // 0.381 is center of frame, 0.127 shifts elevator 5 inches to side like with the real robot
        elevatorRoot = superstructureVis.getRoot("Elevator Root", 0.381 + 0.127, 0.23114);
        elevator = elevatorRoot.append(
            new MechanismLigament2d("Elevator", 0, 90, 10, new Color8Bit("#000000"))
        );
        // The fixed part of the wrist that has the pivot on the end for the grabber
        fixed = elevator.append(
            new MechanismLigament2d("Fixed", 0.127, -90, 10, new Color8Bit("#000000"))
        );
        // Technically visualizes the grabber
        wrist = fixed.append(
            new MechanismLigament2d("Wrist", 0.3048, 0, 10, new Color8Bit("#770085"))
        );
        SmartDashboard.putData("Superstructure Visualization", superstructureVis);
        CommandUtils.makePeriodic(this::updateVisualization, true);
    }

    public MechanismLigament2d getElevatorLigament(){
        return elevator;
    }

    public MechanismLigament2d getWristLigament(){
        return wrist;
    }

    private void updateVisualization() {
        elevator.setLength(superstructure.getElevator().getPositionMeters() + 0.01); //can't set to min value or else advantage scope visualization disappears
        wrist.setAngle(superstructure.getWrist().getAngle());
    }

}
