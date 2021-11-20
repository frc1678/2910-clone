package com.team1678.frc2021.commands;

import javax.swing.plaf.metal.MetalInternalFrameTitlePane;

import com.team1678.frc2021.subsystems.Indexer;
import com.team1678.frc2021.subsystems.Intake;
import com.team1678.frc2021.subsystems.Superstructure;
import com.team1678.frc2021.subsystems.Indexer.WantedAction;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShootCommand extends CommandBase{

    Superstructure mSuperstructure = Superstructure.getInstance();

    public ShootCommand(Superstructure superstructure) {
        mSuperstructure = superstructure;
    }

    @Override
    public void initialize() {
        mSuperstructure.setWantShoot(true);
    }

    @Override
    public void execute() {
        mSuperstructure.setWantShoot(true);
    }

    @Override
    public boolean isFinished() {
        return true;
    }


}
