#include "ahrs_post.h"

#include "log.h"
#include "proj_file.h"

#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#ifdef PLATFORM_WINDOWS
#include <windows.h>
#endif

// Helper function to parse boolean strings
static uint8_t parse_bool(const char* str)
{
	if (strcmp(str, "TRUE") == 0 || strcmp(str, "true") == 0 || strcmp(str, "1") == 0) {
		return 1;
	}
	return 0;
}

// Helper function to allocate string
static char* strdup_safe(const char* str)
{
	if (!str) return NULL;
	size_t len = strlen(str);
	char* result = (char*)malloc(len + 1);
	if (result) {
		strcpy(result, str);
	}
	return result;
}

static int read_post_config(char* post_config_file, post_configs_t *post_configs)
{
	FILE *file = fopen(post_config_file, "r");
	if (!file) {
		LOG_ERROR("Failed to open post config file: %s", post_config_file);
		return -1;
	}

	LOG_INFO("Reading post config file: %s", post_config_file);

	// Initialize post_configs
	post_configs->n = 0;
	post_configs->nmax = 10; // Initial capacity
	post_configs->config = (ahrs_post_config_t*)calloc(post_configs->nmax, sizeof(ahrs_post_config_t));
	if (!post_configs->config) {
		LOG_ERROR("Failed to allocate memory for post configs");
		fclose(file);
		return -1;
	}

	char line[512];
	ahrs_post_config_t *current_config = NULL;
	int config_index = -1;

	while (fgets(line, sizeof(line), file)) {
		// Remove newline and carriage return
		line[strcspn(line, "\r\n")] = 0;

		// Skip empty lines and comments (lines starting with %)
		if (line[0] == '\0' || line[0] == '%' || line[0] == '#') {
			continue;
		}

		// Check if this is a new configuration section (starts with %)
		// Actually, based on the file, %1, %2 etc are already handled as comments
		// So we need to detect new sections differently

		// Parse key=value pairs
		char key[128];
		char value[384];

		if (sscanf(line, "%127[^=]=%383[^\r\n]", key, value) == 2) {
			// Check if this is the "name" field which indicates a new config
			if (strcmp(key, "name") == 0) {
				// Start a new configuration
				config_index++;

				// Check if we need to expand the array
				if (config_index >= post_configs->nmax) {
					post_configs->nmax *= 2;
					ahrs_post_config_t *new_config = (ahrs_post_config_t*)realloc(
						post_configs->config,
						post_configs->nmax * sizeof(ahrs_post_config_t)
					);
					if (!new_config) {
						LOG_ERROR("Failed to reallocate memory for post configs");
						fclose(file);
						return -1;
					}
					post_configs->config = new_config;
					// Zero out new memory
					memset(&post_configs->config[config_index], 0,
						(post_configs->nmax - config_index) * sizeof(ahrs_post_config_t));
				}

				current_config = &post_configs->config[config_index];
				post_configs->n = config_index + 1;

				// Initialize with default user_params
				init_user_params();
				memcpy(&current_config->user_params, get_user_params(), sizeof(user_params_t));

				LOG_INFO("Starting new config section: %s", value);
			}

			if (current_config == NULL) {
				continue; // Skip until we have a valid config
			}

			// Parse configuration fields
			if (strcmp(key, "rover_file") == 0 || strcmp(key, "rover") == 0) {
				current_config->rover_file = strdup_safe(value);
			}
			else if (strcmp(key, "imu_file") == 0 || strcmp(key, "IMUFile") == 0) {
				current_config->imu_file = strdup_safe(value);
			}
			else if (strcmp(key, "result_file") == 0 || strcmp(key, "posresult") == 0) {
				current_config->result_file = strdup_safe(value);
			}
			else if (strcmp(key, "fileformat") == 0) {
				sscanf(value, "%hhu", &current_config->fileformat);
			}
			else if (strcmp(key, "use_start_time") == 0) {
				current_config->use_start_time = parse_bool(value);
			}
			else if (strcmp(key, "use_end_time") == 0) {
				current_config->use_end_time = parse_bool(value);
			}
			else if (strcmp(key, "start_week") == 0) {
				sscanf(value, "%hd", &current_config->start_week);
			}
			else if (strcmp(key, "start_sec") == 0) {
				sscanf(value, "%f", &current_config->start_sec);
			}
			else if (strcmp(key, "end_week") == 0) {
				sscanf(value, "%hd", &current_config->end_week);
			}
			else if (strcmp(key, "end_sec") == 0) {
				sscanf(value, "%f", &current_config->end_sec);
			}
			else if (strcmp(key, "enable") == 0) {
				current_config->enable = parse_bool(value);
			}
			// User params fields
			else if (strcmp(key, "carrier_type") == 0) {
				sscanf(value, "%hhu", &current_config->user_params.carrier_type);
			}
			else if (strcmp(key, "imutype") == 0) {
				sscanf(value, "%hhu", &current_config->user_params.imutype);
			}
			else if (strcmp(key, "estmisv") == 0) {
				sscanf(value, "%hhu", &current_config->user_params.estmisv);
			}
			else if (strcmp(key, "estlever") == 0) {
				sscanf(value, "%hhu", &current_config->user_params.estlever);
			}
			else if (strcmp(key, "estmemsscale") == 0) {
				sscanf(value, "%hhu", &current_config->user_params.estmemsscale);
			}
			else if (strcmp(key, "estmemsort") == 0) {
				sscanf(value, "%hhu", &current_config->user_params.estmemsort);
			}
			else if (strcmp(key, "backfeedtype") == 0) {
				sscanf(value, "%hhu", &current_config->user_params.backfeedtype);
			}
			else if (strcmp(key, "outsmoothtype") == 0) {
				sscanf(value, "%hhu", &current_config->user_params.outsmoothtype);
			}
			else if (strcmp(key, "gnss_arm") == 0) {
				sscanf(value, "%f,%f,%f",
					&current_config->user_params.gnss_arm[0],
					&current_config->user_params.gnss_arm[1],
					&current_config->user_params.gnss_arm[2]);
			}
			else if (strcmp(key, "imu_rot") == 0) {
				sscanf(value, "%f,%f,%f",
					&current_config->user_params.imu_rot[0],
					&current_config->user_params.imu_rot[1],
					&current_config->user_params.imu_rot[2]);
			}
			else if (strcmp(key, "imu_arm") == 0) {
				sscanf(value, "%f,%f,%f",
					&current_config->user_params.imu_arm[0],
					&current_config->user_params.imu_arm[1],
					&current_config->user_params.imu_arm[2]);
			}
			else if (strcmp(key, "club_arm") == 0) {
				sscanf(value, "%f,%f,%f",
					&current_config->user_params.club_arm[0],
					&current_config->user_params.club_arm[1],
					&current_config->user_params.club_arm[2]);
			}
		}
	}

	fclose(file);

	LOG_INFO("Loaded %d post processing configurations", post_configs->n);

	// Log each configuration
	for (int i = 0; i < post_configs->n; i++) {
		ahrs_post_config_t *cfg = &post_configs->config[i];
		LOG_INFO("========================================");
		LOG_INFO("Configuration %d:", i + 1);
		LOG_INFO("========================================");

		// File paths
		LOG_INFO("File Paths:");
		LOG_INFO("  rover_file:  %s", cfg->rover_file ? cfg->rover_file : "NULL");
		LOG_INFO("  imu_file:    %s", cfg->imu_file ? cfg->imu_file : "NULL");
		LOG_INFO("  result_file: %s", cfg->result_file ? cfg->result_file : "NULL");

		// Basic settings
		LOG_INFO("Basic Settings:");
		LOG_INFO("  fileformat: %d", cfg->fileformat);
		LOG_INFO("  enable:     %s", cfg->enable ? "TRUE" : "FALSE");

		// Time filter settings
		LOG_INFO("Time Filter:");
		LOG_INFO("  use_start_time: %s", cfg->use_start_time ? "TRUE" : "FALSE");
		LOG_INFO("  start_week:     %d", cfg->start_week);
		LOG_INFO("  start_sec:      %.3f", cfg->start_sec);
		LOG_INFO("  use_end_time:   %s", cfg->use_end_time ? "TRUE" : "FALSE");
		LOG_INFO("  end_week:       %d", cfg->end_week);
		LOG_INFO("  end_sec:        %.3f", cfg->end_sec);

		// Solve settings
		LOG_INFO("Solve Settings:");
		LOG_INFO("  carrier_type:   %d", cfg->user_params.carrier_type);
		LOG_INFO("  imutype:        %d", cfg->user_params.imutype);
		LOG_INFO("  estmisv:        %d", cfg->user_params.estmisv);
		LOG_INFO("  estlever:       %d", cfg->user_params.estlever);
		LOG_INFO("  estmemsscale:   %d", cfg->user_params.estmemsscale);
		LOG_INFO("  estmemsort:     %d", cfg->user_params.estmemsort);
		LOG_INFO("  backfeedtype:   %d", cfg->user_params.backfeedtype);
		LOG_INFO("  outsmoothtype:  %d", cfg->user_params.outsmoothtype);

		// Lever arm and rotation settings
		LOG_INFO("Lever Arms & Rotations:");
		LOG_INFO("  gnss_arm: [%.3f, %.3f, %.3f]",
			cfg->user_params.gnss_arm[0],
			cfg->user_params.gnss_arm[1],
			cfg->user_params.gnss_arm[2]);
		LOG_INFO("  imu_rot:  [%.3f, %.3f, %.3f]",
			cfg->user_params.imu_rot[0],
			cfg->user_params.imu_rot[1],
			cfg->user_params.imu_rot[2]);
		LOG_INFO("  imu_arm:  [%.3f, %.3f, %.3f]",
			cfg->user_params.imu_arm[0],
			cfg->user_params.imu_arm[1],
			cfg->user_params.imu_arm[2]);
		LOG_INFO("  club_arm: [%.3f, %.3f, %.3f]",
			cfg->user_params.club_arm[0],
			cfg->user_params.club_arm[1],
			cfg->user_params.club_arm[2]);
	}

	return 0;
}
void task_start_post(char* post_config_file)
{
	post_configs_t post_configs = { 0 };

	LOG_INFO("Starting post processing task...");
	LOG_INFO("Config file: %s", post_config_file);

	// Read post processing configurations
	int ret = read_post_config(post_config_file, &post_configs);
	if (ret != 0) {
		LOG_ERROR("Failed to read post config file");
		return;
	}

	if (post_configs.n == 0) {
		LOG_WARNING("No configurations found in post config file");
		return;
	}

	// Process each enabled configuration
	for (int i = 0; i < post_configs.n; i++) {
		ahrs_post_config_t *cfg = &post_configs.config[i];

		if (!cfg->enable) {
			LOG_INFO("Config %d is disabled, skipping", i + 1);
			continue;
		}

		LOG_INFO("========================================");
		LOG_INFO("Processing configuration %d/%d", i + 1, post_configs.n);
		LOG_INFO("========================================");

		// Verify required files exist
		if (!cfg->rover_file || !cfg->imu_file || !cfg->result_file) {
			LOG_ERROR("Config %d: Missing required file paths", i + 1);
			continue;
		}

		LOG_INFO("Rover file: %s", cfg->rover_file);
		LOG_INFO("IMU file: %s", cfg->imu_file);
		LOG_INFO("Result file: %s", cfg->result_file);

		// TODO: Implement actual post processing logic here
		// This would involve:
		// 1. Opening and reading the rover and IMU files
		// 2. Running the post processing algorithms
		// 3. Writing results to the result file
		// For now, just log that we would process this config

		LOG_INFO("Post processing would run here...");
		LOG_INFO("File format: %d", cfg->fileformat);
		LOG_INFO("Time filter: start=%d use_start=%d, end=%d use_end=%d",
			cfg->start_week, cfg->use_start_time,
			cfg->end_week, cfg->use_end_time);
	}

	// Cleanup: free allocated memory
	for (int i = 0; i < post_configs.n; i++) {
		ahrs_post_config_t *cfg = &post_configs.config[i];
		if (cfg->rover_file) free(cfg->rover_file);
		if (cfg->imu_file) free(cfg->imu_file);
		if (cfg->result_file) free(cfg->result_file);
	}
	if (post_configs.config) {
		free(post_configs.config);
	}

	LOG_INFO("Post processing task completed");
}
