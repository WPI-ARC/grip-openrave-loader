/**
 * @file pushDemoTabApp.h
 * @brief Creates application for pushDemoTab
 * @author A. Huaman Q.
 */
#include "GRIPApp.h"
#include "executeFromFileTab.h"

extern wxNotebook* tabView;

class manipulationTabApp : public GRIPApp {
    virtual void AddTabs() {
        tabView->AddPage(new executeFromFileTab(tabView), wxT("executeFromFile"));
    }
};
IMPLEMENT_APP(manipulationTabApp)
