#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <logging/log.h>
#include "display.h"


int main(void) {

	display_init();
	int score = 0;

//    pathfinding_test_main();
    //test_encoders();
    //test_speed();
    //test_pos();
    //test_motors_speed();

    while (1){
		printk("waiting...\n");
		display_send(score++);
		k_sleep(K_MSEC(100));
    }

    return 0;

}
