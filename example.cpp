#include <rplidar.h>

int main() {
	stdio_init_all();
	
	sleep_ms(2000);

	printf("RPLidar example\n");

	RPLidar::RPLidar lidar = RPLidar::RPLidar();

	lidar.reset();

	lidar.startScan();

	// lidar.stopScan();
	std::vector<uint16_t> angles;
	std::vector<uint16_t> distances;

	while (true) {
		// tight_loop_contents();
		sleep_ms(10);
		lidar.processData();
		// Print the buffer contents
		// printf("Buffer contents:\n");
		// lidar.debugPrintBuffer();
		// Print the length of distances
		// lidar.debugPrintLength();
		// Get and print the lidar distances
		lidar.getScan(distances, angles);
		// for (int i = 0; i < distances.size(); i++) {
		// 	printf("%d: %d\n", angles[i], distances[i]);
		// }
	}
}