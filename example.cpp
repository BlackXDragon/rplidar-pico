#include <rplidar.h>

int main() {
	stdio_init_all();
	
	sleep_ms(2000);

	RPLidar::RPLidar lidar = RPLidar::RPLidar();

	lidar.reset();

	lidar.startScan();

	float distances[N_BINS];

	while (true) {
		sleep_ms(10);

		lidar.getDistances(distances);

		// Print comma-separated values to the computer
		for (int i = 0; i < N_BINS; i++) {
			printf("%f,", distances[i]);
		}
		printf("\n");
	}
}