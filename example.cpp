#include <rplidar.h>

int main() {
	stdio_init_all();
	
	sleep_ms(2000);

	// printf("RPLidar example\n");

	RPLidar::RPLidar lidar = RPLidar::RPLidar();

	lidar.reset();

	lidar.startScan();

	// lidar.stopScan();
	uint16_t distances[360];

	while (true) {
		// tight_loop_contents();
		sleep_ms(10);
		lidar.processData();
		// Get and print the lidar distances
		lidar.getScan(distances);
		for (int i = 0; i < 360; i++) {
			
			printf("%d, ", distances[i]);
		}
		printf("\n");
	}
}