/*
 * Copyright (c) 2012, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 */

#include "GRIPApp.h"
#include "ExampleDynamicSimulationTab.hpp"
#include "tabRipPlanner.h"

extern wxNotebook* tabView;

/**
 * @class RipPlannerTabApp
 */
class RipPlannerTabApp : public GRIPApp {
    virtual void AddTabs() {
        tabView->AddPage(new ExampleDynamicSimulationTab(tabView), wxT("Dynamic Sim Example"));
        tabView->AddPage(new RipPlannerTab(tabView), wxT("RIP Planner"));
	}
};

IMPLEMENT_APP(RipPlannerTabApp)
