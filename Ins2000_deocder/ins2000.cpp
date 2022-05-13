#define MAIN_PROGRAM
#ifdef MAIN_PROGRAM
#include "ins2000.h"
#endif
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <map>
#include <string>
#include <vector>
#include "time_conversion.h"

using namespace std;

#define LIMIT_BUFF_LEN	3
#define SYNC_HEADER_LEN 3
#define NMEA_HEADER_LEN 6
#define NEMA_TYPES_NUM	9
#define CRC_LEN 4
#define CRC32_POLYNOMIAL 0xEDB88320L;
#define FILE_NAME_FLAF ""

insconfig_t ins_config = { 0 };
insconfig_translation_t translations[3] = { 0 };
insconfig_rotation_t  rotations[2] = { 0 };
std::vector<time_split_t>  time_split_vec;

enum {
	emBuffer_Type_Bin = 1,
	emBuffer_Type_Ascii = 2,
};

FILE* log_file = NULL;
FILE* nmea_file = NULL;
FILE* imu_file = NULL;
FILE* ins_gga_file = NULL;
FILE* ins_file = NULL;
FILE* ins_std_file = NULL;
FILE* heading_file = NULL;
FILE* gnsss_file = NULL;
FILE* gnssposvel_file = NULL;
FILE* gnssvel_file = NULL;
FILE* ins_kml_file = NULL;
FILE* gnss_kml_file = NULL;
FILE* process_file = NULL;
FILE* insconfig_file = NULL;

extern void init() {
	time_split_vec.clear();
}

static char aceinna_file_basename[256] = { 0 };
extern void set_ins200_file_basename(char* input_name) {
#ifdef MAIN_PROGRAM
	char * pch;
	int size = 0;
	pch = strrchr(input_name, '.');
	size = pch - input_name;
	memcpy(aceinna_file_basename, input_name, size);
#else
	strcpy(aceinna_file_basename, input_name);
#endif
}

void create_file(FILE* &file, const char* suffix) {
	if (strlen(aceinna_file_basename) == 0) return;
	if (file == NULL) {
		char file_name[256] = { 0 };
		sprintf(file_name, "%s_%s", aceinna_file_basename, suffix);
		file = fopen(file_name, "wb");
	}
}

void create_split_file(FILE* &file, const char* file_path){
	if (file == NULL) {
		file = fopen(file_path, "wb");
	}
}

extern void open_log_file() {
	create_file(log_file, "_debug.log");
}

void write_nmea_file(char* str) {
	if (nmea_file == NULL) {
		char file_name[256] = { 0 };
		sprintf(file_name, "%s_%s_neam.neam", aceinna_file_basename, FILE_NAME_FLAF);
		nmea_file = fopen(file_name,"w");
	}
	if (nmea_file)fprintf(nmea_file, str);
}

void open_imu_file() {
	if (imu_file == NULL) {
		char file_name[256] = { 0 };
		sprintf(file_name, "%s_%s_imu.txt", aceinna_file_basename, FILE_NAME_FLAF);
		imu_file = fopen(file_name, "w");
	}
}

void open_ins_file() {
	if (ins_file == NULL) {
		char file_name[256] = { 0 };
		sprintf(file_name, "%s_%s_ins.txt", aceinna_file_basename, FILE_NAME_FLAF);
		ins_file = fopen(file_name, "w");
	}
}

void open_ins_std_file() {
	if (ins_std_file == NULL) {
		char file_name[256] = { 0 };
		sprintf(file_name, "%s_%s_ins_std.txt", aceinna_file_basename, FILE_NAME_FLAF);
		ins_std_file = fopen(file_name, "w");
	}
}

void open_ins_gga_file() {
	if (ins_gga_file == NULL) {
		char file_name[256] = { 0 };
		sprintf(file_name, "%s_%s_ins-gga.nmea", aceinna_file_basename, FILE_NAME_FLAF);
		ins_gga_file = fopen(file_name, "w");
	}
}

void open_heading_file() {
	if (heading_file == NULL) {
		char file_name[256] = { 0 };
		sprintf(file_name, "%s_%s_heading.txt", aceinna_file_basename, FILE_NAME_FLAF);
		heading_file = fopen(file_name, "w");
	}
}

void open_gnss_file() {
	if (gnsss_file == NULL) {
		char file_name[256] = { 0 };
		sprintf(file_name, "%s_%s_gnss.txt", aceinna_file_basename, FILE_NAME_FLAF);
		gnsss_file = fopen(file_name, "w");
	}
}

void open_gnssposvel_file() {
	if (gnssposvel_file == NULL) {
		char file_name[256] = { 0 };
		sprintf(file_name, "%s_%s_gnssposvel.txt", aceinna_file_basename, FILE_NAME_FLAF);
		gnssposvel_file = fopen(file_name, "w");
	}
}

void open_gnssvel_file() {
	if (gnssvel_file == NULL) {
		char file_name[256] = { 0 };
		sprintf(file_name, "%s_%s_gnssvel.txt", aceinna_file_basename, FILE_NAME_FLAF);
		gnssvel_file = fopen(file_name, "w");
	}
}

void open_insconfig_file() {
	if (insconfig_file == NULL) {
		char file_name[256] = { 0 };
		sprintf(file_name, "%s_%s_insconfig.csv", aceinna_file_basename, FILE_NAME_FLAF);
		insconfig_file = fopen(file_name, "w");
		fprintf(insconfig_file, "ImuType,translation_X1,translation_Y1,translation_Z1,translation_X2,translation_Y2,translation_Z2,User_X,User_Y,User_X,rotation_Z,rotation_Y,rotation_Z\n");
	}
}
FILE* rangecmp_file = NULL;
void open_rangecmp_file() {
	if (rangecmp_file == NULL) {
		char file_name[256] = { 0 };
		sprintf(file_name, "%s_%s_rangecmp.csv", aceinna_file_basename, FILE_NAME_FLAF);
		rangecmp_file = fopen(file_name, "w");
	}
}

#define HEADKML1 "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
#define HEADKML2 "<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n"
#define MARKICON "http://maps.google.com/mapfiles/kml/shapes/track.png"
#define R2D   (180/3.1415926)
#define SIZP     0.3            /* mark size of rover positions */
#define SIZR     0.3            /* mark size of reference position */
#define TINT     30.0           /* time label interval (sec) */

void print_kml_header(FILE *kml_file) {
	if (kml_file) {
		const char* color[6] ={
			"ffffffff","ff0000ff","ffff00ff","ffff901e","ff00ff00","ff00aaff"
		};//B-G-R °×É« SPP RTD UDR FIX FLOAT
		int i;
		fprintf(kml_file, HEADKML1);
		fprintf(kml_file, HEADKML2);
		fprintf(kml_file, "<Document>\n");
		for (i = 0; i < 6; i++)
		{
			fprintf(kml_file, "<Style id=\"P%d\">\n", i);
			fprintf(kml_file, "<IconStyle>\n");
			fprintf(kml_file, "<color> %s </color>\n", color[i]);
			fprintf(kml_file, "<scale> %f </scale>\n", SIZP);
			fprintf(kml_file, "<Icon><href> %s </href></Icon>\n", MARKICON);
			fprintf(kml_file, "</IconStyle>\n");
			fprintf(kml_file, "</Style>\n");
		}
		fprintf(kml_file, "<Folder>\n");
		fprintf(kml_file, "<name>Rover Position</name>\n");
	}
}

void print_kml_end(FILE *kml_file) {
	if (kml_file) {
		fprintf(kml_file, "</Folder>\n");
		fprintf(kml_file, "</Document>\n");
		fprintf(kml_file, "</kml>\n");
	}
}

void open_ins_kml_file() {
	if (ins_kml_file == NULL) {
		char file_name[256] = { 0 };
		sprintf(file_name, "%s_%s_ins.kml", aceinna_file_basename, FILE_NAME_FLAF);
		ins_kml_file = fopen(file_name, "w");
		print_kml_header(ins_kml_file);
	}
}

void open_gnss_kml_file() {
	if (gnss_kml_file == NULL) {
		char file_name[256] = { 0 };
		sprintf(file_name, "%s_%s_gnss.kml", aceinna_file_basename, FILE_NAME_FLAF);
		gnss_kml_file = fopen(file_name, "w");
		print_kml_header(gnss_kml_file);
	}
}

void open_process_file() {
	if (process_file == NULL) {
		char file_name[256] = { 0 };
		sprintf(file_name, "%s_%s_process.txt", aceinna_file_basename, FILE_NAME_FLAF);
		process_file = fopen(file_name, "w");
	}
}

void write_ins_config();

extern void close_ins2000_all_file()
{
	write_ins_config();
	if (log_file)fclose(log_file); log_file = NULL;
	if (nmea_file)fclose(nmea_file); nmea_file = NULL;
	if (imu_file)fclose(imu_file); imu_file = NULL;
	if (ins_gga_file)fclose(ins_gga_file); ins_gga_file = NULL;
	if (heading_file)fclose(heading_file); heading_file = NULL;
	if (gnsss_file)fclose(gnsss_file); gnsss_file = NULL;
	if (gnssposvel_file)fclose(gnssposvel_file); gnssposvel_file = NULL;
	if (gnssvel_file)fclose(gnssvel_file); gnssvel_file = NULL;
	if (ins_kml_file) { print_kml_end(ins_kml_file); fclose(ins_kml_file); } ins_kml_file = NULL;
	if (gnss_kml_file) { print_kml_end(gnss_kml_file); fclose(gnss_kml_file); } gnss_kml_file = NULL;
	if (insconfig_file)fclose(insconfig_file); insconfig_file = NULL;
	for (int i = 0; i < time_split_vec.size(); i++) {
		if (time_split_vec[i].split_file) {
			fclose(time_split_vec[i].split_file); 
			time_split_vec[i].split_file = NULL;
		}
	}
	if(rangecmp_file)fclose(rangecmp_file); rangecmp_file = NULL;
}

uint32_t CRC32Value(int32_t i)
{
	int32_t j;
	uint32_t ulCRC;
	ulCRC = i;
	for (j = 8; j > 0; j--)
	{
		if (ulCRC & 1u) {
			ulCRC = (ulCRC >> 1u) ^ CRC32_POLYNOMIAL;
		}
		else {
			ulCRC >>= 1u;
		}			
	}
	return ulCRC;
}

uint32_t CalculateBlockCRC32(
	uint32_t ulCount,          // Number of bytes in the data block
	const uint8_t* ucBuffer)  // Data block
{
	uint32_t ulTemp1;
	uint32_t ulTemp2;
	uint32_t ulCRC = 0;
	while (ulCount-- != 0)
	{
		ulTemp1 = (ulCRC >> 8u) & 0x00FFFFFFL;
		ulTemp2 = CRC32Value(((int32_t)ulCRC ^ *ucBuffer++) & 0xffu);
		ulCRC = ulTemp1 ^ ulTemp2;
	}
	return(ulCRC);
}

#define ACEINNA_GYRO   (0.005/64)  //
#define ACEINNA_ACC    (0.005*9.80665/4000)  //
#define P2_33       1.164153218269348E-10 /* 2^-33 */
#define P2_33_DEG   6.670106611340576E-09 /* 2^-33 */
#define P2_27_F     2.270936965942383E-09 /* 2^-27 FEET */
#define P2_29_F     5.677342414855957E-10 /* 2^-29 FEET */
#define P2_29       1.862645149230957E-09 /* 2^-29 */
#define FSAS_GYRO   1.085069444444445E-07  //0.1x 2-8 arcsec/LSB
#define FSAS_ACC    1.525878906250000E-06  //0.05 x 2-15 m/s/LSB
#define ISA100C_GYRO 5.729577951308233E-08
#define ISA100C_ACC 2.0E-8
#define PI          3.1415926535897932  /* pi */

#define RATES_SIZE 26
double rates[RATES_SIZE][4] = {
	{0,100,2.0,100},
	{1,100,100,100},
	{3,200,ACEINNA_GYRO,ACEINNA_ACC},
	{4,100,100,100},
	{5,100,100,100},
	{8,200,100,100},
	{11,100,P2_33,P2_27_F},
	{12,100,100,100},
	{13,200,FSAS_GYRO,FSAS_ACC},
	{16,200,100,100},
	{19,200,100,100},
	{20,100,100,100},
	{26,200,ISA100C_GYRO,ISA100C_ACC},
	{27,100,100,100},
	{28,100,100,100},
	{31,200,100,100},
	{32,125,100,100},
	{33,200,100,100},
	{34,200,100,100},
	{38,400,100,100},
	{39,400,100,100},
	{41,125,100,100},
	{45,200,100,100},
	{52,200,100,100},
	{56,125,100,100},
	{58,100,P2_33_DEG,P2_29},
};
static int readcount = 0;
static int msg_num = 0;
const uint8_t sync_header_1[SYNC_HEADER_LEN] = { 0xAA, 0x44, 0x12 };
const uint8_t sync_header_2[SYNC_HEADER_LEN] = { 0xAA, 0x44, 0x13 };
const char* nmea_headers[NEMA_TYPES_NUM] = { "$GNGGA", "$GPGGA", "$GPGSA", "$GPGST", "$GPGSV", "$GPHDT", "$GPRMC", "$GPVTG", "$GPZDA" };
static ins200_raw_t raw = { 0 };
static ins200_header_t header = { 0 };
static uint32_t limit_buffer_index = 0;
static uint8_t limit_buffer[LIMIT_BUFF_LEN] = { 0 };
static char output_buffer[MAX_BUFF_LEN] = { 0 };
bestgnsspos_t bestgnsspos = { 0 };
bestgnssvel_t bestgnssvel = { 0 };

std::map<int, std::string> imu_types = { 
	{ 0, "UNKNOWN" 			 },
	{ 1, "HG1700_AG11"		 },
	{ 4, "HG1700_AG17"		 },
	{ 5, "HG1900_CA29"		 },
	{ 8, "LN200 Northrop"	 },
	{11, "HG1700_AG58"		 },
	{12, "HG1700_AG62"		 },
	{13, "IMAR_FSAS"		 },
	{16, "KVH_COTS"			 },
	{20, "HG1930_AA99"		 },
	{26, "ISA100C"			 },
	{27, "HG1900_CA50"		 },
	{28, "HG1930_CA50"		 },
	{31, "ADIS16488"		 },
	{32, "STIM300"			 },
	{33, "KVH_1750"			 },
	{41, "EPSON_G320"		 },
	{52, "LITEF_MICROIMU"	 },
	{56, "STIM300D"			 },
	{58, "HG4930_AN01"		 },
	{62, "EPSON_G320_200HZ"	 }
};

static void push_limit_buffer(uint8_t c) {
	if (limit_buffer_index >= LIMIT_BUFF_LEN) {
		memmove(limit_buffer, limit_buffer + 1, LIMIT_BUFF_LEN - 1);
		limit_buffer_index = LIMIT_BUFF_LEN - 1;
	}
	limit_buffer[limit_buffer_index++] = c;
}

static void euler2dcm(const double eular[3], double dc[3][3])
{
	double roll = eular[0];
	double  pitch = eular[1];
	double  heading = eular[2];
	double  cr, cp, ch, sr, sp, sh;
	cr = cos(roll); cp = cos(pitch); ch = cos(heading);
	sr = sin(roll); sp = sin(pitch); sh = sin(heading);

	dc[0][0] = cp * ch;
	dc[0][1] = -cr * sh + sr * sp*ch;
	dc[0][2] = sr * sh + cr * sp*ch;

	dc[1][0] = cp * sh;
	dc[1][1] = cr * ch + sr * sp*sh;
	dc[1][2] = -sr * ch + cr * sp * sh;

	dc[2][0] = -sp;
	dc[2][1] = sr * cp;
	dc[2][2] = cr * cp;
}

static uint8_t MatrixMutiply(const double *matrix_a, const double *matrix_b, const int matrix_a_row, const int matrix_a_column, const int matrix_b_column, double *matrix_result)
{
	double sum = 0;
	double median = 0;
    int i,j,k;
	for (i = 0; i < matrix_a_row; i++)
	{
		for (k = 0; k < matrix_b_column; k++)
		{
			for (j = 0; j < matrix_a_column; j++)
			{
				median = matrix_a[matrix_a_column*i + j] * matrix_b[matrix_b_column*j + k];
				sum = sum + median;
			}
			matrix_result[matrix_b_column*i + k] = sum;
			sum = 0;
		}
	}
	return 1;
}

uint8_t MatrixAdd(const double *matrix_a, const double *matrix_b, const int matrix_a_row, const int matrix_a_colume, double *matrix_result)
{
	int i;
	for (i = 0; i < matrix_a_row*matrix_a_colume; i++)
	{
		*(matrix_result + i) = *(matrix_a + i) + *(matrix_b + i);
	}
	return 1;
}

typedef  struct EarthParameter
{
	double a;       //Ellipsoid long axis
	double b;       //Ellipsoid short axis
	double f;       //Ellipsoidal oblate 
	double e;       //first Eccentricity of Elliopsoid 
	double e2;
	//double ep;
	//double ep2;     //second Eccentricity of Elliopsoid 
	double wie;     //rotational angular velocity of the earths  
	double GM;      //geocentric gravitational constant 
} EarthParameter;
const  EarthParameter WGS84 = { 6378137.0, 6356752.3142, 0.0033528106643315515,0.081819190837555025,0.0066943799893122479 ,  7.2922115147e-5,398600441800000.00 };

//typedef struct {        /* time struct */
//	time_t time;        /* time (s) expressed by standard time_t */
//	double sec;         /* fraction of second under 1 s */
//} gtime_t;

uint8_t UpdateMN(const double *BLH, double *M, double *N)
{
	double sinB = sin(*BLH);
	double temp = 1 - WGS84.e2 * sinB * sinB;
	double sqrttemp = sqrt(temp);
	*M = WGS84.a * (1 - WGS84.e2) / (sqrttemp*temp);
	*N = WGS84.a / sqrttemp;
	return 1;
};

int32_t get_pos_type(int position_type)
{
	int32_t ret = 0;
	switch (position_type)
	{
	case 16:
		//spp;
		ret = 1;
		break;
	case 53:
		//spp;
		ret = 1;
		break;
	case 17:
		//rtd;
		ret = 2;
		break;
	case 54:
		//rtd;
		ret = 2;
		break;
		//case 3:
		//	//udr;
		//	pcolor = 4;
		//	break;
	case 50:
		//fix;
		ret = 4;
		break;
	case 56:
		//fix;
		ret = 4;
		break;
	case 55:
		//float;
		ret = 5;
		break;
	case 34:
		//float;
		ret = 5;
		break;
	default:
		break;
	}
	return ret;
}

int32_t get_ins_status(int ins_status)
{
	if (ins_status <= 3) {
		return ins_status;
	}
	else if(ins_status <= 7) {
		return ins_status - 2;
	}
	else {
		return ins_status;
	}
}

static void deg2dms(double deg, double* dms)
{
	double sign = deg < 0.0 ? -1.0 : 1.0, a = fabs(deg);
	dms[0] = floor(a); a = (a - dms[0]) * 60.0;
	dms[1] = floor(a); a = (a - dms[1]) * 60.0;
	dms[2] = a; dms[0] *= sign;
}

static int outnmea_gga(unsigned char* buff, double time, int type, double* blh, int ns, double dop, double age)
{
	double h, ep[6], dms1[3], dms2[3];
	char* p = (char*)buff, *q, sum;

	if (type != 1 && type != 4 && type != 5) {
		p += sprintf_s(p, 255, "$GPGGA,,,,,,,,,,,,,,");
		for (q = (char*)buff + 1, sum = 0; *q; q++) sum ^= *q;
		p += sprintf_s(p, 255, "*%02X%c%c", sum, 0x0D, 0x0A);
		return p - (char*)buff;
	}
	time -= 18.0;
	ep[2] = floor(time / (24 * 3600));
	time -= ep[2] * 24 * 3600.0;
	ep[3] = floor(time / 3600);
	time -= ep[3] * 3600;
	ep[4] = floor(time / 60);
	time -= ep[4] * 60;
	ep[5] = time;
	h = 0.0;
	deg2dms(fabs(blh[0]) * 180.0 / PI, dms1);
	deg2dms(fabs(blh[1]) * 180.0 / PI, dms2);
	p += sprintf_s(p, 255, "$GPGGA,%02.0f%02.0f%05.2f,%02.0f%010.7f,%s,%03.0f%010.7f,%s,%d,%02d,%.1f,%.3f,M,%.3f,M,%.1f,",
		ep[3], ep[4], ep[5], dms1[0], dms1[1] + dms1[2] / 60.0, blh[0] >= 0 ? "N" : "S",
		dms2[0], dms2[1] + dms2[2] / 60.0, blh[1] >= 0 ? "E" : "W", type,
		ns, dop, blh[2] - h, h, age);
	for (q = (char*)buff + 1, sum = 0; *q; q++) sum ^= *q; /* check-sum */
	p += sprintf_s(p, 255, "*%02X%c%c", sum, 0x0D, 0x0A);
	return p - (char*)buff;
}

void print_ins_gga_file(const inspvax_t* msg) {
	if (fabs(msg->latitude) > 0.001
		&& (msg->ins_status == 3 || msg->ins_status == 6 || msg->ins_status == 7)) {
		double leverarm_v[3] = { 0.0,0.0, 0.0 };
		double eular[3] = { 0 };
		double C_vn[3][3];
		double leverarm_n[3];
		double d_leverarm[3];
		double pos[3] = {0};
		double M, N;
		unsigned char ggaBuffer[400] = { 0 };
		int type;
		int len;
		eular[0] = msg->roll*PI / 180;
		eular[1] = msg->pitch * PI / 180;
		eular[2] = msg->azimuth * PI / 180;
		pos[0] = msg->latitude*PI / 180;
		pos[1] = msg->longitude*PI / 180;
		pos[2] = msg->altitude + msg->undulation;
		euler2dcm(eular, C_vn);
		MatrixMutiply(*C_vn, leverarm_v, 3, 3, 1, leverarm_n);
		UpdateMN(pos, &M, &N);
		d_leverarm[0] = leverarm_n[0] / (M + pos[2]);
		d_leverarm[1] = leverarm_n[1] / ((N + pos[2])*cos(pos[0]));
		d_leverarm[2] = -leverarm_n[2];
		MatrixAdd(pos, d_leverarm, 3, 1, pos);
		type = get_pos_type(msg->position_type);
		len = outnmea_gga(ggaBuffer, header.gps_seconds, type, pos, 10, 1.0, 1.0);
		open_ins_gga_file();
		if (ins_gga_file) fprintf(ins_gga_file, "%s", ggaBuffer);
	}
}

void print_ins_file(const inspvax_t* msg) {
	sprintf(output_buffer, "%4d,%10.4f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%14.9f,%14.9f,%14.9f,%d,%d\n", header.week, header.gps_seconds,
		msg->latitude, msg->longitude, msg->altitude + msg->undulation, msg->north_velocity, msg->east_velocity, msg->up_velocity, msg->roll, msg->pitch, msg->azimuth, msg->ins_status, msg->position_type);
	open_ins_file();
	if (ins_file) fprintf(ins_file, output_buffer);
	if (process_file) fprintf(process_file, "$GPINS,%s", output_buffer);
}

void print_ins_std_file(const inspvax_t* msg) {
	sprintf(output_buffer, 
		"%4d,%10.4f,%3d,%3d"
		",%14.9f,%14.9f,%10.4f"
		",%10.4f,%10.4f,%10.4f"
		",%10.4f,%10.4f,%10.4f"
		",%8.3f,%8.3f,%8.3f"
		",%8.3f,%8.3f,%8.3f"
		",%8.3f,%8.3f,%8.3f\n"
	, header.week, header.gps_seconds
	, get_ins_status(msg->ins_status), get_pos_type(msg->position_type)
	, msg->latitude, msg->longitude, msg->altitude + msg->undulation
	, msg->north_velocity, msg->east_velocity, msg->up_velocity
	, msg->roll, msg->pitch, msg->azimuth
	, msg->latitude_std, msg->longitude_std, msg->altitude_std
	, msg->north_velocity_std, msg->east_velocity_std, msg->up_velocity_std
	, msg->roll_std, msg->pitch_std, msg->azimuth_std);
	open_ins_std_file();
	if (ins_std_file) fprintf(ins_std_file, output_buffer);
}

#define INS_KML_OUTPUT_DATARATE (1.0)
void print_ins_kml(const inspvax_t* msg) {
	if (fmod(header.gps_seconds + 0.0005, INS_KML_OUTPUT_DATARATE) >= 0.005) return;
	if (fabs(msg->latitude) < 0.001) return;
	open_ins_kml_file();
	if (ins_kml_file) {
		double ep[6] = { 0 };
		int pcolor = 0;
		gtime_t gpstime = gpst2time(header.week, header.gps_seconds);
		gtime_t utctime = gpst2utc(gpstime);
		time2epoch(utctime, ep);
		switch (msg->ins_status) {
		case 0:pcolor = 0;break;
		case 1:pcolor = 1;break;
		case 2:pcolor = 1;break;
		case 3:pcolor = 4;break;
		case 6:pcolor = 1;break;
		case 7:pcolor = 1;break;
		default:break;
		}
		fprintf(ins_kml_file, "<Placemark>\n");
		fprintf(ins_kml_file, "<TimeStamp><when>%04d-%02d-%02dT%02d:%02d:%05.2fZ</when></TimeStamp>\n", (int32_t)ep[0], (int32_t)ep[1], (int32_t)ep[2], (int32_t)ep[3], (int32_t)ep[4], ep[5]);
		//=== description start ===
		fprintf(ins_kml_file, "<description><![CDATA[\n");
		fprintf(ins_kml_file, "<TABLE border=\"1\" width=\"100 % \" Align=\"center\">\n");
		fprintf(ins_kml_file, "<TR ALIGN=RIGHT>\n");
		fprintf(ins_kml_file, "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Time:</TD><TD>%d</TD><TD>%.3f</TD><TD>%2d:%2d:%5.3f</TD><TD>%4d/%2d/%2d</TD></TR>\n",
			header.week, header.gps_seconds,(int32_t)ep[3],(int32_t)ep[4], ep[5], (int32_t)ep[0], (int32_t)ep[1], (int32_t)ep[2]);
		fprintf(ins_kml_file, "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Position:</TD><TD>%.9f</TD><TD>%.9f</TD><TD>%.4f</TD><TD>(DMS,m)</TD></TR>\n",
			msg->latitude, msg->longitude, msg->altitude + msg->undulation);
		fprintf(ins_kml_file, "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Vel(N,E,D):</TD><TD>%f</TD><TD>%f</TD><TD>%f</TD><TD>(m/s)</TD></TR>\n",
			msg->north_velocity, msg->east_velocity, -msg->up_velocity);
		fprintf(ins_kml_file, "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Att(r,p,h):</TD><TD>%f</TD><TD>%f</TD><TD>%f</TD><TD>(deg,approx)</TD></TR>\n",
			msg->roll, msg->pitch, -msg->azimuth);
		fprintf(ins_kml_file, "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Mode:</TD><TD>%d</TD><TD>%d</TD></TR>\n",
			msg->ins_status, msg->position_type);
		fprintf(ins_kml_file, "</TABLE>\n");
		fprintf(ins_kml_file, "]]></description>\n");
		//=== description end ===
		fprintf(ins_kml_file, "<styleUrl>#P%d</styleUrl>\n", pcolor);
		fprintf(ins_kml_file, "<Style>\n");
		fprintf(ins_kml_file, "<IconStyle>\n");
		fprintf(ins_kml_file, "<heading>%f</heading>\n", msg->azimuth);
		fprintf(ins_kml_file, "</IconStyle>\n");
		fprintf(ins_kml_file, "</Style>\n");
		fprintf(ins_kml_file, "<Point>\n");
		fprintf(ins_kml_file, "<coordinates>%13.9f,%12.9f,%5.3f</coordinates>\n", msg->longitude, msg->latitude, msg->altitude + msg->undulation);
		fprintf(ins_kml_file, "</Point>\n");
		fprintf(ins_kml_file, "</Placemark>\n");
	}
}

void print_inspvax_file(const inspvax_t* msg) {
	print_ins_gga_file(msg);
	print_ins_file(msg);
	print_ins_std_file(msg);
	print_ins_kml(msg);
}

void print_heading_file(const heading_t* msg) {
	sprintf(output_buffer,"%4d,%10.4f,%10.5f,%14.5f,%14.5f,%14.5f,%14.5f,%8d,%8d\n", header.week, header.gps_seconds,
		msg->length, msg->heading, msg->pitch, msg->hdgstddev, msg->ptchstddev, msg->solution_status, msg->position_type);
	open_heading_file();
	if (heading_file) fprintf(heading_file, output_buffer);
	if (process_file) fprintf(process_file, "$GPHEAD2,%s", output_buffer);
}

void print_imu_file(const rawimusx_t* msg) {
	static double lastlctime = 0;
	int i;
	double fxyz_scale, wxyz_scale, sample_rate;
	double lctime = (double)header.gps_seconds;
	double x_accel, y_accel, z_accel,x_gyro,y_gyro,z_gyro;
	for (i = 0; i < RATES_SIZE; i++) {
		if ((uint8_t)(rates[i][0]) == msg->imutype) {
			sample_rate = rates[i][1];
			wxyz_scale = rates[i][2];
			fxyz_scale = rates[i][3];
			break;
		}
	}
	x_accel = msg->x_accel * fxyz_scale * sample_rate;
	y_accel = -msg->y_accel *fxyz_scale * sample_rate;
	z_accel = msg->z_accel * fxyz_scale * sample_rate;
	x_gyro = msg->x_gyro * wxyz_scale * sample_rate;
	y_gyro = -msg->y_gyro * wxyz_scale * sample_rate;
	z_gyro = msg->z_gyro * wxyz_scale * sample_rate;

	if (fmod(lctime + 0.02, 1) < 0.01 && lctime - lastlctime > 0.98)
	{
		header.gps_seconds = floor(lctime + 0.02);
		lastlctime = lctime;
	}
	else
	{
		lctime = 0;
	}
	sprintf(output_buffer, "%4d,%10.4f,%10.4f,%14.10f,%14.10f,%14.10f,%14.10f,%14.10f,%14.10f \n", msg->week, msg->seconds, floor(lctime + 0.02), x_accel, y_accel, z_accel, x_gyro, y_gyro, z_gyro);
	open_imu_file();
	if (imu_file) fprintf(imu_file, output_buffer);
	if (process_file) fprintf(process_file, "$GPIMU,%s",output_buffer);
}

void print_bestgnsspos_file(const bestgnsspos_t* msg) {
	int type = get_pos_type(msg->position_type);
	if (msg->solution_status != 0)
	{
		type = 0;
	}
	if (fmod(header.gps_seconds + 0.001, 1) < 0.01) {
		if (type >= 0) {
			sprintf(output_buffer, "%4d,%10.4f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%d\n", header.week, header.gps_seconds,
				msg->lat, msg->lon, msg->hgt + msg->undulation, msg->lat_sigma, msg->lon_sigma, msg->height_sigma, type);
			open_gnss_file();
			if (gnsss_file) fprintf(gnsss_file, output_buffer);
			if (process_file) fprintf(process_file, "$GPGNSS,%s", output_buffer);
		}
	}
}

void print_bestgnssvel_file(const bestgnssvel_t* msg) {
	if (fmod(header.gps_seconds + 0.001, 1) < 0.01) {
		sprintf(output_buffer, "%4d,%10.4f,%14.9f,%14.9f,%10.4f,%10.4f,%8d,%8d\n", header.week, header.gps_seconds,
			msg->hor_spd, msg->trk_gnd, msg->vert_spd, msg->latency, msg->sol_status, msg->vel_type);
		open_gnssvel_file();
		if (gnssvel_file)fprintf(gnssvel_file, output_buffer);
		if (process_file) fprintf(process_file, "$GPVEL,%s", output_buffer);
	}
}

void print_gnssposvel_file(const bestgnssvel_t* vel, const bestgnsspos_t* pos) {
	if (fabs(pos->lat) > 0.001) {
		int type = get_pos_type(pos->position_type);
		double north_velocity = vel->hor_spd * cos(vel->trk_gnd * PI / 180);
		double east_velocity = vel->hor_spd * sin(vel->trk_gnd * PI / 180);
		double up_velocity = vel->vert_spd;
		sprintf(output_buffer, "%4d,%10.4f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%d,%10.4f,%10.4f,%10.4f,%10.4f\n", header.week, header.gps_seconds,
			pos->lat, pos->lon, pos->hgt + pos->undulation, pos->lat_sigma, pos->lon_sigma, pos->height_sigma, type, north_velocity, east_velocity, up_velocity, vel->trk_gnd);
		open_gnssposvel_file();
		if (gnssposvel_file) fprintf(gnssposvel_file, output_buffer);
	}
}

void print_gnss_kml(const bestgnssvel_t* vel, const bestgnsspos_t* pos) {
	if (fmod(header.gps_seconds + 0.005, 0.1) >= 0.05) return;
	if (fabs(pos->lat) < 0.001)  return;
	open_gnss_kml_file();
	if (gnss_kml_file) {
		double ep[6] = { 0 };
		double north_velocity,east_velocity,up_velocity;
		int pcolor;
		gtime_t gpstime = gpst2time(header.week, header.gps_seconds);
		gtime_t utctime = gpst2utc(gpstime);
		time2epoch(utctime, ep);
		north_velocity = vel->hor_spd * cos(vel->trk_gnd * PI / 180);
		east_velocity = vel->hor_spd * sin(vel->trk_gnd * PI / 180);
		up_velocity = vel->vert_spd;
		pcolor = get_pos_type(pos->position_type);
		fprintf(gnss_kml_file, "<Placemark>\n");
		fprintf(gnss_kml_file, "<TimeStamp><when>%04d-%02d-%02dT%02d:%02d:%05.2fZ</when></TimeStamp>\n", (int32_t)ep[0], (int32_t)ep[1], (int32_t)ep[2], (int32_t)ep[3], (int32_t)ep[4], ep[5]);
		//=== description start ===
		fprintf(gnss_kml_file, "<description><![CDATA[\n");
		fprintf(gnss_kml_file, "<TABLE border=\"1\" width=\"100 % \" Align=\"center\">\n");
		fprintf(gnss_kml_file, "<TR ALIGN=RIGHT>\n");
		fprintf(gnss_kml_file, "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Time:</TD><TD>%d</TD><TD>%.3f</TD><TD>%2d:%2d:%5.3f</TD><TD>%4d/%2d/%2d</TD></TR>\n",
			header.week, header.gps_seconds, (int32_t)ep[3], (int32_t)ep[4], ep[5], (int32_t)ep[0], (int32_t)ep[1], (int32_t)ep[2]);
		fprintf(gnss_kml_file, "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Position:</TD><TD>%.9f</TD><TD>%.9f</TD><TD>%.4f</TD><TD>(DMS,m)</TD></TR>\n",
			pos->lat, pos->lon, pos->hgt + pos->undulation);
		fprintf(gnss_kml_file, "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Vel(N,E,D):</TD><TD>%f</TD><TD>%f</TD><TD>%f</TD><TD>(m/s)</TD></TR>\n",
			north_velocity, east_velocity, -up_velocity);
		fprintf(gnss_kml_file, "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Att(r,p,h):</TD><TD>%d</TD><TD>%d</TD><TD>%f</TD><TD>(deg,approx)</TD></TR>\n",
			0, 0, vel->trk_gnd);
		fprintf(gnss_kml_file, "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Mode:</TD><TD>%d</TD><TD>%d</TD></TR>\n",
			pos->solution_status, pos->position_type);
		fprintf(gnss_kml_file, "</TABLE>\n");
		fprintf(gnss_kml_file, "]]></description>\n");
		//=== description end ===
		fprintf(gnss_kml_file, "<styleUrl>#P%d</styleUrl>\n", pcolor);
		fprintf(gnss_kml_file, "<Style>\n");
		fprintf(gnss_kml_file, "<IconStyle>\n");
		fprintf(gnss_kml_file, "<heading>%f</heading>\n", vel->trk_gnd);
		fprintf(gnss_kml_file, "</IconStyle>\n");
		fprintf(gnss_kml_file, "</Style>\n");
		fprintf(gnss_kml_file, "<Point>\n");
		fprintf(gnss_kml_file, "<coordinates>%13.9f,%12.9f,%5.3f</coordinates>\n", pos->lon, pos->lat, pos->hgt + pos->undulation);
		fprintf(gnss_kml_file, "</Point>\n");
		fprintf(gnss_kml_file, "</Placemark>\n");
	}
}

void print_insconfig(insconfig_t& config) 
{
	if (insconfig_file == NULL) return;
	fprintf(insconfig_file,"insconfig_t: \n\
ImuType = %d \n\
Mapping = %d \n\
InitialAlignmentVelocity = %d \n\
HeaveWindow, = %d \n\
Profile = %d \n\
EnabledUpdates = %d \n\
AlignmentMode = %d \n\
RelativeINSOutputFrame = %d \n\
RelativeINSOutputDirection = %d \n\
INSReceiverStatus = %d \n\
INSSeedEnabled = %d \n\
INSSeedValidation = %d \n",
config.ImuType,
config.Mapping,
config.InitialAlignmentVelocity,
config.HeaveWindow,
config.Profile,
config.EnabledUpdates,
config.AlignmentMode,
config.RelativeINSOutputFrame,
config.RelativeINSOutputDirection,
config.INSReceiverStatus,
config.INSSeedEnabled,
config.INSSeedValidation);
	fprintf(insconfig_file,"\n");
}

void print_translation(insconfig_translation_t& config) 
{
	if (insconfig_file == NULL) return;
	fprintf(insconfig_file, "insconfig_translation_t: \n\
Translation = %d \n\
Frame = %d \n\
X_Offset = %f \n\
Y_Offset = %f \n\
Z_Offset = %f \n\
X_Uncertainty = %f \n\
Y_Uncertainty = %f \n\
Z_Uncertainty = %f \n\
TranslationSource = %d \n",
config.Translation,
config.Frame,
config.X_Offset,
config.Y_Offset,
config.Z_Offset,
config.X_Uncertainty,
config.Y_Uncertainty,
config.Z_Uncertainty,
config.TranslationSource);
	fprintf(insconfig_file, "\n");
}

void print_rotation(insconfig_rotation_t& config)
{
	if (insconfig_file == NULL) return;
	fprintf(insconfig_file, "insconfig_rotation_t: \n\
Rotation = %d \n\
Frame = %d \n\
X_Rotation = %f \n\
Y_Rotation = %f \n\
Z_Rotation = %f \n\
X_RotationStdDev = %f \n\
Y_RotationStdDev = %f \n\
Z_RotationStdDev = %f \n\
RotationSource = %d \n",
config.Rotation,
config.Frame,
config.X_Rotation,
config.Y_Rotation,
config.Z_Rotation,
config.X_RotationStdDev,
config.Y_RotationStdDev,
config.Z_RotationStdDev,
config.RotationSource);
	fprintf(insconfig_file, "\n");
}

void write_ins_config() {
	if (!insconfig_file)return;
	fprintf(insconfig_file, "%s, %11.6f,%11.6f,%11.6f, %11.6f,%11.6f,%11.6f, %11.6f,%11.6f,%11.6f, %11.6f,%11.6f,%11.6f\n", imu_types[ins_config.ImuType].c_str(), translations[0].X_Offset, translations[0].Y_Offset, translations[0].Z_Offset,
		translations[1].X_Offset, translations[1].Y_Offset, translations[1].Z_Offset,
		translations[2].X_Offset, translations[2].Y_Offset, translations[2].Z_Offset,
		rotations[0].X_Rotation, rotations[0].Y_Rotation, rotations[0].Z_Rotation);
}

/* extract unsigned/signed bits ------------------------------------------------
* extract unsigned/signed bits from byte data
* args   : unsigned char *buff I byte data
*          int    pos    I      bit position from start of data (bits)
*          int    len    I      bit length (bits) (len<=32)
* return : extracted unsigned/signed bits
*-----------------------------------------------------------------------------*/
uint32_t getbitu32(const unsigned char* buff, int pos, int len)
{
	uint32_t bits = 0;
	int i;
	for (i = pos; i < pos + len; i++)
	{
		bits = (bits << 1) + ((buff[i / 8] >> (7 - i % 8)) & 1u);
	}
	return bits;
}
int32_t getbits32(const unsigned char* buff, int pos, int len)
{
	uint32_t bits = getbitu32(buff, pos, len);
	if (len <= 0 || 32 <= len || !(bits & (1u << (len - 1))))
	{
		return (int32_t)bits;
	}
	return (int32_t)(bits | (~0u << len)); /* extend sign */
}
uint64_t getbitu64(const unsigned char* buff, int pos, int len)
{
	uint64_t bits = 0;
	int i;
	for (i = pos; i < pos + len; i++)
	{
		bits = (bits << 1) + ((buff[i / 8] >> (7 - i % 8)) & 1u);
	}
	return bits;
}
int64_t getbits64(const unsigned char* buff, int pos, int len)
{
	uint64_t bits = getbitu64(buff, pos, len);
	if (len <= 0 || 64 <= len || !(bits & (1u << (len - 1))))
	{
		return (int64_t)bits;
	}
	return (int64_t)(bits | (~0u << len)); /* extend sign */
}
float get_StdDev_PSR_Value(uint32_t StdDev_PSR)
{
	switch (StdDev_PSR) {
		case 0:return 0.050;
		case 1:return 0.075;
		case 2:return 0.113;
		case 3:return 0.169;
		case 4:return 0.253;
		case 5:return 0.380;
		case 6:return 0.570;
		case 7:return 0.854;
		case 8:return 1.281;
		case 9:return 2.375;
		case 10:return 4.750;
		case 11:return 9.500;
		case 12:return 19.000;
		case 13:return 38.000;
		case 14:return 76.000;
		case 15:return 152.000;
	}
	return 0;
}
char get_sys(uint8_t Satellite_system) {
	//0 = GPS
	//1 = GLONASS
	//2 = SBAS
	//3 = Galileo
	//4 = BeiDou
	//5 = QZSS
	//6 = NavIC
	//7 = Other
	switch (Satellite_system)
	{
	case 0:
		return 'G'; //GPS
	case 1:
		return 'R'; //GLONASS
	case 2:
		return 'S'; //SBAS
	case 3:
		return 'E'; //Galileo
	case 4:
		return 'C'; //BeiDou
	case 5:
		return 'J'; //QZSS
	case 6:
		return 'I'; //NavIC
	case 7:
		return 'O'; //Other
	default:
		return '-';
	}
}
#define OBS_LEN  (24)
/* get fields (little-endian) ------------------------------------------------*/
#define U1(p) (*((unsigned char *)(p)))
#define I1(p) (*((signed char *)(p)))
static unsigned short U2(unsigned char *p) { unsigned short u; memcpy(&u, p, 2); return u; }
static unsigned int   U4(unsigned char *p) { unsigned int   u; memcpy(&u, p, 4); return u; }
static int            I4(unsigned char *p) { int            i; memcpy(&i, p, 4); return i; }
static float          R4(unsigned char *p) { float          r; memcpy(&r, p, 4); return r; }
static double         R8(unsigned char *p) { double         r; memcpy(&r, p, 8); return r; }
/* extend sign ---------------------------------------------------------------*/
static int exsign(unsigned int v, int bits)
{
	return (int)(v&(1 << (bits - 1)) ? v | (~0u << bits) : v);
}
void decode_rangecmp_obs(uint8_t* obs_buff) {
	uint32_t Channel_Tracking_Status = 0;
	memcpy(&Channel_Tracking_Status, obs_buff, 4);
	uint8_t Tracking_state = Channel_Tracking_Status & 0x1F;
	uint8_t SV_channel_number = (Channel_Tracking_Status>>5) & 0x1F;
	uint8_t Phase_lock_flag = (Channel_Tracking_Status >> 10) & 0x01;
	uint8_t Parity_known_flag = (Channel_Tracking_Status >> 11) & 0x01;
	uint8_t Code_locked_flag = (Channel_Tracking_Status >> 12) & 0x01;
	uint8_t Correlator_type = (Channel_Tracking_Status >> 13) & 0x07;
	uint8_t Satellite_system = (Channel_Tracking_Status >> 16) & 0x07;
	uint8_t Grouping = (Channel_Tracking_Status >> 20) & 0x01;
	uint8_t Signal_type = (Channel_Tracking_Status >> 21) & 0x1F;
	uint8_t Primary_L1_channel = (Channel_Tracking_Status >> 27) & 0x1;
	uint8_t Carrier_phase = (Channel_Tracking_Status >> 28) & 0x1;
	uint8_t Digital_filtering = (Channel_Tracking_Status >> 29) & 0x1;
	uint8_t PRN_lock_flag = (Channel_Tracking_Status >> 30) & 0x1;
	uint8_t Channel_assignment = (Channel_Tracking_Status >> 31) & 0x1;
	if (rangecmp_file) {
		fprintf(rangecmp_file, "%2d,%2d,%d,%d,%d,%d,%c,%d,%2d,%d,%d,%d,%d,%d,\n"
			, Tracking_state,SV_channel_number, Phase_lock_flag, Parity_known_flag, Code_locked_flag
			, Correlator_type, get_sys(Satellite_system), Grouping, Signal_type
			, Primary_L1_channel, Carrier_phase, Digital_filtering, PRN_lock_flag, Channel_assignment);
	}

	float f_Doppler_Frequency = exsign(U4(obs_buff + 4) & 0xFFFFFFF, 28) / 256.0;
	double f_Pseudorange = (U4(obs_buff + 7) >> 4) / 128.0 + U1(obs_buff + 11)*2097152.0;
	float f_ADR = I4(obs_buff + 12) / 256.0;

	int32_t  ADR = getbits32(obs_buff, 96, 32);
	uint32_t StdDev_PSR = getbitu32(obs_buff, 128, 4);
	uint32_t StdDev_ADR = getbitu32(obs_buff, 132, 4);
	uint32_t PRN = getbitu32(obs_buff, 136, 8);
	float f_Lock_Time = (U4(obs_buff + 18) & 0x1FFFFF) / 32.0; /* lock time */
	uint32_t C_No = getbitu32(obs_buff, 165, 5);
	uint32_t GLONASS_Frequency_number = getbitu32(obs_buff, 170, 5);

	float f_StdDev_PSR = get_StdDev_PSR_Value(StdDev_ADR);
	float f_StdDev_ADR = float(StdDev_PSR + 1) / 512;
	C_No += 20;
	fprintf(rangecmp_file, "%f,%f,%f,%.3f,%.3f,%d,%f,%d,%d\n", f_Doppler_Frequency, f_Pseudorange, f_ADR, f_StdDev_PSR, f_StdDev_ADR, PRN, f_Lock_Time, C_No, GLONASS_Frequency_number);
}
void decode_rangecmp()
{
	open_rangecmp_file();
	if (rangecmp_file) {
		int number = 0;
		memcpy(&number, raw.buffer + header.header_len, sizeof(int));
		fprintf(rangecmp_file, "%4d,%10.4f,%d\n", header.week, header.gps_seconds, number);
		uint8_t obs_buff[OBS_LEN] = { 0 };
		int offset = header.header_len + sizeof(number);
		for (int i = 0; i < number; i++) {
			memcpy(obs_buff, raw.buffer + offset, OBS_LEN);
			for (int j = 0; j < OBS_LEN; j++) {
				fprintf(rangecmp_file, "%02x", obs_buff[j]);
			}
			fprintf(rangecmp_file, "\n");
			decode_rangecmp_obs(obs_buff);
			offset += OBS_LEN;
		}
	}
}

void decode_message() {
	open_process_file();
	switch (header.message_id)
	{
	case 1969: {}break;
	case 2010: {}break;
	case 1429: {/*bestgnsspos*/
//		printf("%d %d/%d OK\n", header.message_id, sizeof(bestgnsspos_t), header.message_len);
		if (header.message_len != sizeof(bestgnsspos_t))break;
		memcpy(&bestgnsspos, raw.buffer + header.header_len, sizeof(bestgnsspos_t));
		print_bestgnsspos_file(&bestgnsspos);
	}break;
	case 1430: {/*bestgnssvel*/
//		printf("%d %d/%d OK\n", header.message_id, sizeof(bestgnssvel_t), header.message_len);
		if (header.message_len != sizeof(bestgnssvel_t))break;
		memcpy(&bestgnssvel, raw.buffer + header.header_len, sizeof(bestgnssvel_t));
		print_bestgnssvel_file(&bestgnssvel);
		print_gnssposvel_file(&bestgnssvel,&bestgnsspos);
		print_gnss_kml(&bestgnssvel, &bestgnsspos);
	}break;
	case 42: {}break;
	case 99: {/*bestvel*/
		bestvel_t msg = { 0 };
//		printf("%d %d/%d OK\n", header.message_id, sizeof(bestvel_t), header.message_len);
		if (header.message_len != sizeof(bestvel_t))break;
		memcpy(&msg, raw.buffer + header.header_len, sizeof(bestvel_t));
	}break;
	case 241: {}break;
	case 263:{
		insatt_t msg = { 0 };
//		printf("%d %d/%d OK\n", header.message_id, sizeof(insatt_t), header.message_len);
		if (header.message_len != sizeof(insatt_t))break;
		memcpy(&msg, raw.buffer + header.header_len, sizeof(insatt_t));
	}break;
	case 507:{
		inspva_t msg = { 0 };
//		printf("%d %d/%d OK\n", header.message_id, sizeof(inspva_t), header.message_len);
		if (header.message_len != sizeof(inspva_t))break;
		memcpy(&msg, raw.buffer + header.header_len, sizeof(inspva_t));
	}break;
	case 1465:{
		inspvax_t msg = { 0 };
//		printf("%d %d/%d OK\n", header.message_id, sizeof(inspvax_t), header.message_len);
		if (header.message_len != sizeof(inspvax_t))break;
		memcpy(&msg, raw.buffer + header.header_len, sizeof(inspvax_t));
		print_inspvax_file(&msg);
	}break;
	case 265: {}break;
	case 266: {}break;
	case 267: {}break;
	case 1122: {}break;
	case 723: {}break;
	case 7: {}break;
	case 971:{
		heading_t msg = { 0 };
//		printf("%d %d/%d OK\n", header.message_id, sizeof(heading_t), header.message_len);
		if (header.message_len != sizeof(heading_t))break;
		memcpy(&msg, raw.buffer + header.header_len, sizeof(heading_t));
		print_heading_file(&msg);
	}break;
	case 1335:{
		heading2_t msg = { 0 };
//		printf("%d %d/%d OK\n", header.message_id, sizeof(heading2_t), header.message_len);
		if (header.message_len != sizeof(heading2_t))break;
		memcpy(&msg, raw.buffer + header.header_len, sizeof(heading2_t));
	}break;
	case 8: {}break;
	case 96: {}break;
	case 6006: {}break;
	case 174: {}break;
	case 47: {}break;
	case 100: {}break;
	case 43: {
		printf("43\n");
	}break;
	case 6005: {}break;
	case 140: {
		decode_rangecmp();
	}break;
	case 268: {
		rawimu_t msg = { 0 };
//		printf("%d %d/%d OK\n", header.message_id, sizeof(rawimu_t), header.message_len);
		if (header.message_len != sizeof(rawimu_t))break;
		memcpy(&msg, raw.buffer + header.header_len, sizeof(rawimu_t));
	}break;
	case 325: {}break;
	case 1462: {
		rawimusx_t msg = { 0 };
//		printf("%d %d/%d OK\n", header.message_id, sizeof(rawimusx_t), header.message_len);
		if (header.message_len != sizeof(rawimusx_t))break;
		memcpy(&msg, raw.buffer + header.header_len, sizeof(rawimusx_t)); 
		print_imu_file(&msg); 
	}break;
	case 1461: {}break;
	case 175: {}break;
	case 1325: {}break;
	case 952: {}break;
	case 141: {}break;
	case 216: {}break;
	case 101: {}break;
	case 37: {}break;
	case 1945:
	{
		//if (insconfig_file) break;
		open_insconfig_file();
		uint8_t* cur = raw.buffer + header.header_len;
		memcpy(&ins_config, cur, sizeof(insconfig_t)); cur += sizeof(insconfig_t);
		//print_insconfig(msg);
		uint32_t NumberofTranslations = 0;
		memcpy(&NumberofTranslations, cur, sizeof(uint32_t)); cur += sizeof(uint32_t);
		if (NumberofTranslations > 0) {
			for (int i = 0; i < NumberofTranslations; i++) {
				if (i < 3) {
					memcpy(&translations[i], cur, sizeof(insconfig_translation_t));
				}
				cur += sizeof(insconfig_translation_t);
			}
		}
		uint32_t NumberofRotations = 0;
		memcpy(&NumberofRotations, cur, sizeof(uint32_t)); cur += sizeof(uint32_t);
		if (NumberofRotations > 0) {
			for (int i = 0; i < NumberofRotations; i++) {
				if (i < 2) {
					memcpy(&rotations[i], cur, sizeof(insconfig_rotation_t)); 
				}
				cur += sizeof(insconfig_rotation_t);
			}
		}
		uint32_t read_size = cur - (raw.buffer + header.header_len);
		//printf("%d, message_len = %d, insconfig_t = %d, NumberofTranslations = %d, NumberofRotations = %d, read_size = %d \n", header.message_id, header.message_len, sizeof(insconfig_t), NumberofTranslations, NumberofRotations, read_size);
	}
	break;
	default:
		break;
	}
}

void split_bin_file() {
	if (time_split_vec.size() > 0) {
		gtime_t time_stamp = gpst2time(header.week, header.gps_seconds);
		for (int i = 0; i < time_split_vec.size(); i++) {
			time_split_t& time_split = time_split_vec[i];
			if ((i == 0 || time_split.start_time <= time_stamp.time) &&
				(time_stamp.time <= time_split.end_time + 1 || i == time_split_vec.size() - 1)) {
				create_split_file(time_split.split_file, time_split.file_name);
				fwrite(raw.buffer, 1, raw.buffer_len, time_split.split_file);
				break;
			}
		}
	}
}

void decode_ins2000_bin() {
	if (header.header_len == 0 && raw.buffer_len >= SYNC_HEADER_LEN + 3) {
		memcpy(&header.message_id, &raw.buffer[SYNC_HEADER_LEN + 1], 2);
		if (header.message_id == 1462) {
			header.header_len = 12;
		}
		else {
			header.header_len = raw.buffer[SYNC_HEADER_LEN];
		}
	}
	else if (header.header_len > 0) {
		if (raw.buffer_len == header.header_len) {
			if (header.message_id == 1462) {
				header.message_type = 0;
				header.message_len = raw.buffer[SYNC_HEADER_LEN];
				memcpy(&header.week, &raw.buffer[6], 2);
				memcpy(&header.gps_ms, &raw.buffer[8], 4);
				header.gps_seconds = (double)header.gps_ms / 1000.0;
			}
			else {
				header.message_type = raw.buffer[6];
				header.port_address = raw.buffer[7];
				memcpy(&header.message_len, &raw.buffer[8], 2);
				memcpy(&header.sequence, &raw.buffer[10], 2);
				header.idle_time = raw.buffer[12];
				header.time_status = raw.buffer[13];
				memcpy(&header.week, &raw.buffer[14], 2);
				memcpy(&header.gps_ms, &raw.buffer[16], 4);
				header.gps_seconds = (double)header.gps_ms / 1000.0;
				memcpy(&header.receiver_status, &raw.buffer[20], 4);
				memcpy(&header.receiver_sw_version, &raw.buffer[26], 2);
			}
		}
		else if (header.message_len > 0) {
			if (raw.buffer_len == header.header_len + header.message_len + CRC_LEN) {
				uint32_t cal_crc = CalculateBlockCRC32(raw.buffer_len - CRC_LEN, raw.buffer);
				uint32_t read_crc = 0;
				memcpy(&read_crc, &raw.buffer[raw.buffer_len - CRC_LEN], CRC_LEN);
				if (read_crc == cal_crc) {
					split_bin_file();
					decode_message();
					if (log_file) {
						fprintf(log_file, "%4d,%10.4f,  %04d,%5d,%5d,%8d\n", header.week, header.gps_seconds, header.message_id, raw.buffer_len, readcount, ++msg_num);
					}
					readcount = 0;
					//printf("%04d %03d %d\n", header.message_id, raw.buffer_len, readcount);
				}
				else {
					printf("%d Error len=%d,crc=[%u:%u]\r\n", header.message_id, raw.buffer_len, read_crc, cal_crc);
				}
				memset(&raw, 0, sizeof(raw));
				memset(&header, 0, sizeof(header));
			}
		}
	}
}

void decode_ins2000_nmea() {
	if (raw.buffer[raw.buffer_len - 1] == '\n' && raw.buffer[raw.buffer_len - 2] == '\r') {
		raw.buffer[raw.buffer_len] = '\0';
		write_nmea_file((char*)raw.buffer);
		memset(&raw, 0, sizeof(raw));
		memset(&header, 0, sizeof(header));
	}
}
extern void input_ins2000_raw(uint8_t c) {
	readcount++;
	if (raw.buffer_len == 0) {
		push_limit_buffer(c);
		if (limit_buffer[0] == sync_header_1[0]) {
			if (memcmp(limit_buffer, sync_header_1, SYNC_HEADER_LEN) == 0) {
				raw.buffer_type = emBuffer_Type_Bin;
			}
			else if (memcmp(limit_buffer, sync_header_2, SYNC_HEADER_LEN) == 0) {
				raw.buffer_type = emBuffer_Type_Bin;
			}
		}
		else if (limit_buffer[0] == '$') {// nmea 
			int i;
			for (i = 0; i < NEMA_TYPES_NUM; i++) {
				if (memcmp(limit_buffer, nmea_headers[i], NMEA_HEADER_LEN) == 0) {
					raw.buffer_type = emBuffer_Type_Ascii;
					raw.nmea_type = i;
					break;
				}
			}
		}
		if (raw.buffer_type > 0) {
			memcpy(raw.buffer, limit_buffer, limit_buffer_index);
			raw.buffer_len = limit_buffer_index;
		}		
	}
	else {
		if (raw.buffer_len < MAX_BUFF_LEN - 1) {
			raw.buffer[raw.buffer_len++] = c;
			if (emBuffer_Type_Bin == raw.buffer_type) {
				decode_ins2000_bin();
			}
			else if (emBuffer_Type_Ascii == raw.buffer_type) {
				decode_ins2000_nmea();
			}
		}
		else {
			memset(&raw, 0, sizeof(raw));
			memset(&header, 0, sizeof(header));
		}
	}
}

void split(const string& s, vector<string>& tokens, const string& delim = " ") {
	tokens.clear();
	size_t lastPos = s.find_first_not_of(delim, 0);
	size_t pos = s.find(delim, lastPos);
	while (lastPos != string::npos) {
		tokens.emplace_back(s.substr(lastPos, pos - lastPos));
		lastPos = s.find_first_not_of(delim, pos);
		pos = s.find(delim, lastPos);
	}
}

void append_split_config(char * line)
{
	time_split_t split_time = { 0 };
	vector<string> tokens;
	split(line, tokens, ",");
	if (tokens.size() >= 4) {
		split_time.split_id = atoi(tokens[0].c_str());
		split_time.start_time = atoi(tokens[1].c_str());
		split_time.end_time = atoi(tokens[2].c_str());
		strcpy(split_time.file_name,tokens[3].c_str());
		time_split_vec.push_back(split_time);
	}
}
