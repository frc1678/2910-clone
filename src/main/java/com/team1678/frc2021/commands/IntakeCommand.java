package com.team1678.frc2021.commands;

import javax.swing.plaf.metal.MetalInternalFrameTitlePane;

import com.team1678.frc2021.subsystems.Indexer;
import com.team1678.frc2021.subsystems.Intake;
import com.team1678.frc2021.subsystems.Superstructure;
import com.team1678.frc2021.subsystems.Indexer.WantedAction;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeCommand extends CommandBase{

    private final Intake mIntake;
    private final Indexer mIndexer;

    public IntakeCommand(Intake intake, Indexer indexer) {
        mIntake = intake;
        mIndexer = indexer;
    }

    @Override
    public void execute() {
        mIntake.setState(Intake.WantedAction.INTAKE);
        mIndexer.setState(Indexer.WantedAction.INDEX);
    }


}
