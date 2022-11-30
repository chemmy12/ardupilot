//
// Created by chemmy on 11/12/22.
//

#include "AP_MB4_AKS16_Enc.h"

MB4_AKS16_Enc::MB4_AKS16_Enc():
    notYetInit(true)
{
}


void MB4_AKS16_Enc::update()
{

    hal.console->printf("MB4_AKS16_Enc::update(): could not init() MB4.\n");


    if (notYetInit) {    // happens only once at boot
        aks16.init_AKS16();
        aks16.createBackProcess();
        notYetInit = false;
    }

    if (!aks16.test()) {
        if (!aks16.init_AKS16()) {
            hal.console->printf("MB4_AKS16_Enc::update(): could not init() MB4.\n");
            return;
        }
        else {
            hal.console->printf("MB4_AKS16_Enc::update(): MB4 alive test failed.\n");
            return;
        }
    }


}
