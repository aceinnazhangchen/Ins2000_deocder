#pragma once
#include <stdint.h>
#include <stdio.h>

#define MAX_BUFF_LEN 2048

#pragma pack(push, 1)

typedef struct {
	uint32_t buffer_len;
	uint8_t buffer[MAX_BUFF_LEN];
	uint8_t buffer_type; //1 BIN ,2 ASCII
	uint8_t nmea_type; // index of nmea_headers
}ins200_raw_t;

typedef struct {
	uint32_t header_len;
	uint16_t message_id;
	uint8_t message_type;
	uint8_t port_address;
	uint16_t message_len;
	uint16_t sequence;
	uint8_t idle_time;
	uint8_t time_status;
	uint16_t week;
	uint32_t gps_ms;
	double gps_seconds;
	uint32_t receiver_status;
	uint16_t bds_gps_time_offset;
	uint16_t receiver_sw_version;
}ins200_header_t;

typedef struct {
	uint32_t week;
	double seconds;
	double roll;
	double pitch;
	double azimuth;
	uint32_t status;
}insatt_t;

typedef struct {
	uint32_t week;
	double seconds;
	double latitude;
	double longitude;
	double height;
	double north_velocity;
	double east_velocity;
	double up_velocity;
	double roll;
	double pitch;
	double azimuth;
	uint32_t status;
}inspva_t;

typedef struct {
	uint32_t ins_status;
	uint32_t position_type;
	double latitude;
	double longitude;
	double altitude;
	float undulation;
	double north_velocity;
	double east_velocity;
	double up_velocity;
	double roll;
	double pitch;
	double azimuth;
	float latitude_std;
	float longitude_std;
	float altitude_std;
	float north_velocity_std;
	float east_velocity_std;
	float up_velocity_std;
	float roll_std;
	float pitch_std;
	float azimuth_std;
	uint32_t ext_sol_stat;
	uint16_t seconds_since_update;
}inspvax_t;

typedef struct {
	int32_t solution_status;
	int32_t position_type;
	float length;					//Baseline length (m)
	float heading;					//Heading in degrees [0,360)
	float pitch;					//Pitch in degrees +- 90
	float reserved;
	float hdgstddev;
	float ptchstddev;
	char stnid[4];
	uint8_t SVs;
	uint8_t solnSVs;
	uint8_t obs;
	uint8_t multi;
	uint8_t reserved_2;
	uint8_t ext_sol_stat;
	uint8_t reserved_3;
	uint8_t sig_mask;
}heading_t;

typedef struct {
	int32_t solution_status;
	int32_t position_type;
	float length;				   //Baseline length (m)
	float heading;                 //Heading in degrees [0,360)
	float pitch;                   //Pitch in degrees +- 90
	float reserved;
	float hdgstddev;
	float ptchstddev;
	char rover_stnid[4];
	char baser_stnid[4];
	uint8_t SVs;
	uint8_t solnSVs;
	uint8_t obs;
	uint8_t multi;
	uint8_t reserved_2;
	uint8_t ext_sol_stat;
	uint8_t reserved_3;
	uint8_t sig_mask;
}heading2_t;

typedef struct {
	uint32_t week;
	double seconds;
	uint32_t imu_status;
	int32_t z_accel;
	int32_t y_accel;
	int32_t x_accel;
	int32_t z_gyro;
	int32_t y_gyro;
	int32_t x_gyro;
} rawimu_t;

typedef struct {
	int8_t imuinfo;
	int8_t imutype;
	uint16_t week;
	double seconds;
	uint32_t imu_status;
	int32_t z_accel;
	int32_t y_accel;
	int32_t x_accel;
	int32_t z_gyro;
	int32_t y_gyro;
	int32_t x_gyro;
} rawimusx_t;

typedef struct {
	int32_t solution_status;
	int32_t position_type;
	double lat;
	double lon;
	double hgt;
	float undulation;
	uint32_t datum_id;
	float lat_sigma;       //Latitude standard deviation (metres)
	float lon_sigma;      //Longitude standard deviation (metres)
	float height_sigma;       //Height standard deviation (metres)
	char stn_id[4];
	float diff_age;
	float sol_age;
	uint8_t SVs;
	uint8_t solnSVs;
	uint8_t reserved_1;
	uint8_t reserved_2;
	uint8_t reserved_3;
	uint8_t ext_sol_stat;
	uint8_t reserved_4;
	uint8_t sig_mask;
}bestgnsspos_t;

typedef struct {
	int32_t sol_status;
	int32_t vel_type;
	float latency;
	float age;
	double hor_spd;
	double trk_gnd;
	double vert_spd;
	float reserved;
}bestvel_t, bestgnssvel_t;

typedef struct {
	uint32_t Translation;
	uint32_t Frame;
	float X_Offset;
	float Y_Offset;
	float Z_Offset;
	float X_Uncertainty;
	float Y_Uncertainty;
	float Z_Uncertainty;
	uint32_t TranslationSource;
}insconfig_translation_t;

typedef struct {
	uint32_t Rotation;
	uint32_t Frame;
	float X_Rotation;
	float Y_Rotation;
	float Z_Rotation;
	float X_RotationStdDev;
	float Y_RotationStdDev;
	float Z_RotationStdDev;
	uint32_t RotationSource;
}insconfig_rotation_t;

typedef struct {
	uint32_t ImuType;
	uint8_t Mapping;
	uint8_t InitialAlignmentVelocity;
	uint16_t HeaveWindow;
	uint32_t Profile;
	uint32_t EnabledUpdates;
	uint32_t AlignmentMode;
	uint32_t RelativeINSOutputFrame;
	uint32_t RelativeINSOutputDirection;
	uint32_t INSReceiverStatus;
	uint8_t INSSeedEnabled;
	uint8_t INSSeedValidation;
	uint16_t Reserved1;
	uint32_t Reserved2_7[6];
}insconfig_t;

typedef struct {
	uint32_t split_id;
	uint32_t start_time;
	uint32_t end_time;
	char file_name[256];
	FILE* split_file;
}time_split_t;

#pragma pack(pop)

extern void init();
extern void set_ins200_file_basename(char* input_name);
extern void set_split_file_basename(char * input_name);
extern void close_ins2000_all_file();
extern void open_log_file();

extern void input_ins2000_raw(uint8_t c);

extern void append_split_config(char* line);