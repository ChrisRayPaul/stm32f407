#include "workflow/workflow.h"
#include "devices/devices.h"
#include "unit_test/unit_test.h"

void init()
{
	devices_init();
	workflow_init();
	log_d("system started.");
}

void loop()
{
	loop_task_execute();
	workflow_execute();
}


void launch()
{
    init();

    for(;;)
    {
        loop();
    }
}
