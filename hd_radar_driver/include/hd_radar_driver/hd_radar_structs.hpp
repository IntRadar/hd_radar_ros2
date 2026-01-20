#ifndef HD_RADAR_STRUCTS_HPP
#define HD_RADAR_STRUCTS_HPP

#include <stdint.h>
#include <netinet/in.h>
#include <linux/can.h>
#include <net/if.h>

//Compatible versions of radar protocol
#define UDP_PROTOCOL_MAJOR 1 // 4 bit format 
#define UDP_PROTOCOL_MINOR 3 // 4 bit format
// 8 bit format b3:0 major b7:4 minor 
#define UDP_PROTOCOL_VER (((UDP_PROTOCOL_MAJOR & 0xF) << 4) |\
						 (UDP_PROTOCOL_MINOR & 0xF)) 
						 
#define CAN_PROTOCOL_MAJOR 1 // 4 bit format 
#define CAN_PROTOCOL_MINOR 2 // 4 bit format
// 8 bit format b3:0 major b7:4 minor 
#define CAN_PROTOCOL_VER (((CAN_PROTOCOL_MAJOR & 0xF) << 4) |\
						(CAN_PROTOCOL_MINOR & 0xF)) 

#define UDP_PNTS_MAX   				(32U)
#define UDP_PAYLOAD_SIZE    		(1024U)

#define CAN_PCL_MSG_ID				(0xFEU)
#define CAN_MAX_POINTS_NUM			(256U)
#define CAN_PREHEADER_MSG_ID_OFFSET	(0x100U)
#define CAN_HEADER_MSG_ID_OFFSET	(0x101U)
#define CAN_PAYLOAD_MSG_ID_OFFSET	(0x110U)

#pragma pack(push, 1) // enable 1 byte alignment
//Preheader UDP
typedef struct {
    uint8_t  version;
    uint8_t  msg_id;
    uint16_t length;
} udp_pre_hdr_t;

// Header Pcl
typedef struct {
	uint32_t frame_cnt;
	uint16_t udp_total;
	uint16_t udp_idx;
	uint16_t flags;
	uint16_t udp_pnts_num;
	float self_velocity;
	float noise_lvl;
	uint8_t pcl_type;
	uint8_t data_type;
	uint8_t time_type;
	uint8_t reserved0;
	uint32_t tv_usec_lsb;
	uint32_t tv_usec_msb;
	int32_t reserved1;
	int16_t reserved2;
	uint16_t crc16;
} udp_hdr_pcl_t;

// Header raw
typedef struct {
	uint32_t frame_cnt;
	uint16_t udp_total;
	uint16_t udp_idx;
	uint16_t flags;
	uint16_t length;
	int32_t tv_usec_lsb;
	int32_t tv_usec_msb;
	int16_t reserved2;
	uint16_t crc16;
} udp_hdr_raw_t;

// Header heat map
typedef struct {
	uint32_t frame_cnt;
	uint16_t udp_total;
	uint16_t udp_idx;
	uint8_t map_type;
	uint8_t reserved0;
	int16_t reserved1;
	uint16_t map_h_size;
	uint16_t map_v_size;
	uint32_t tv_usec_lsb;
	uint32_t tv_usec_msb;
	int16_t reserved4;
	uint16_t crc16;
} udp_hdr_heat_t;

// Arguments thresholds send
typedef struct {
    uint16_t sta_threshold;
    uint16_t sta_azm_sense;
    int16_t sta_rcs_filter;
    uint8_t reserved_1[2]{0};
    uint16_t dyn_threshold;
    uint16_t dyn_azm_sense;
    int16_t dyn_rcs_filter;
    uint8_t reserved_2[18]{0};
} udp_arg_thr_t;

// Arguments velocity send
typedef struct {
    float velocity;
    uint32_t hold_time;
    uint8_t reserved_1[24]{0};
} udp_arg_vel_t;

// Arguments mode send
typedef struct {
    uint8_t mode;
    uint8_t test_source;
    uint16_t reserved0;
    int32_t reserved[7];
} udp_arg_mode_t;

// Sending arguments thresholds message
typedef struct {
    udp_pre_hdr_t pre_header;
    udp_arg_thr_t args;
} udp_msg_arg_thr_t;

// Sending arguments velocity message
typedef struct {
    udp_pre_hdr_t pre_header;
    udp_arg_vel_t args;
} udp_msg_arg_vel_t;

// Sending arguments mode message
typedef struct {
    udp_pre_hdr_t pre_header;
    udp_arg_mode_t args;
} udp_msg_arg_mode_t;

// Received raw message
typedef struct {
    udp_pre_hdr_t pre_header;
    udp_hdr_raw_t raw_header;
    uint8_t data[UDP_PAYLOAD_SIZE];
} udp_msg_raw_t;

// Received heat map message
typedef struct {
	udp_pre_hdr_t pre_header;
	udp_hdr_heat_t heat_header;
    uint8_t data[UDP_PAYLOAD_SIZE];
} udp_msg_heat_t;

typedef struct {
    uint16_t raw_frame_num;
    int16_t reserved0;
    int32_t reserved[7];
} udp_args_get_raw_t;

// Get raw message
typedef struct {
    udp_pre_hdr_t pre_header;
    udp_args_get_raw_t raw_args;
} udp_get_raw_t;

// UDP single point
typedef struct {
	float range;
	float velocity;
	float azm;
	float elv;
	float snr;
	float rcs;
	int32_t reserved0;
} udp_pcl_data_t;

// PCL message UDP-packet structure
typedef struct {
	udp_pre_hdr_t  pre_header;
    udp_hdr_pcl_t  pcl_header;
    udp_pcl_data_t payload[UDP_PNTS_MAX];
} udp_msg_pcl_t;

//Preheader CAN
typedef struct {
	uint8_t 		version;
	uint8_t 		msg_id;
	uint16_t 		length;
} can_pre_hdr_t;

//Header CAN
typedef struct {
	uint32_t 		frame_cnt;
	uint32_t		tv_usec_lsb;
	uint32_t		tv_usec_msb;
	uint16_t 		flags;
	uint16_t 		pnts_num;
	int16_t 		ego_velocity;
	uint16_t 		reserved1;
	uint8_t 		noise_level;
	uint8_t 		reserved2;
	uint16_t 		crc16;
} can_hdr_pcl_t;

// CAN single point
typedef struct {
	uint16_t		range;
	int16_t			velocity;
	int16_t			azimuth;
	int16_t			elevation;
	uint8_t			snr;
	int8_t			rcs;
	uint16_t		reserved0;
} can_pcl_data_t;

// PCL message CAN-frame structure
typedef struct {
	can_pre_hdr_t 	pre_header;
	can_hdr_pcl_t 	header;
	can_pcl_data_t  point[CAN_MAX_POINTS_NUM];
} can_msg_pcl_t;
#pragma pack(pop) // disable 1 byte alignment

//Message type
typedef enum {
    RAW         = 0xa8,
    PCL         = 0xfe,
    HEAT        = 0xd5,
	GET_RAW     = 0xA7, 
    SET_THR     = 0xC6,
    SET_VEL     = 0x55,
    SET_MODE    = 0xB2
} msg_type_t;

//Socket structure
typedef struct {
	int fd;
    struct sockaddr_in self_addr, dest_addr;
    struct ip_mreq mreq;
} udp_sock_data_t;

typedef struct {
	int fd;
    struct sockaddr_can addr;
    struct ifreq ifr;
} can_sock_data_t;

#endif