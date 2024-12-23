package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class PickUpSystem {

    Linkage linkage = new Linkage();
    GlobalUse lift = new GlobalUse();
    BratServo brat = new BratServo();
    int submersibleSearch=0;

    public void StartSubmersibleSearch(int state)
    {
        submersibleSearch=state;
    }

    public void PerformSubmersibleSearch()
    {
        switch(submersibleSearch)
        {
            case 1:
                linkage.setBratTarget(2100);
                submersibleSearch=2;
                break;

            case 2:
                if(!linkage.IsMoving())
                    submersibleSearch=3;
                break;

            case 3:
                lift.SetLiftTarget(lift.ReturnLiftPickUpMax());
                submersibleSearch=4;
                break;

            case 4:
                if(!lift.IsMoving())
                    submersibleSearch=5;

            case 0:
            default:
                break;


        }
    }

}
