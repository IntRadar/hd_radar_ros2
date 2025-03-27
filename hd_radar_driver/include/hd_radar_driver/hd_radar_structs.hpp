#ifndef HD_RADAR_STRUCTS_HPP
#define HD_RADAR_STRUCTS_HPP

#include <stdint.h>
#include <netinet/in.h>

#define UDP_PNTS_MAX   		(32U)
#define UDP_PAYLOAD_SIZE    (1024U)

//Preheader
typedef struct {
    uint8_t  version;
    uint8_t  msg_id;
    uint16_t length;
} pre_hdr_t;

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
} arg_thr_t;

// Arguments velocity send
typedef struct {
    float velocity;
    uint32_t hold_time;
    uint8_t reserved_1[24]{0};
} arg_vel_t;

// Arguments mode send
typedef struct {
    uint8_t mode;
    uint8_t test_source;
    uint16_t reserved0;
    int32_t reserved[7];
} arg_mode_t;

// Sending arguments thresholds message
typedef struct {
    pre_hdr_t pre_header;
    arg_thr_t args;
} msg_arg_thr_t;

// Sending arguments velocity message
typedef struct {
    pre_hdr_t pre_header;
    arg_vel_t args;
} msg_arg_vel_t;

// Sending arguments mode message
typedef struct {
    pre_hdr_t pre_header;
    arg_mode_t args;
} msg_arg_mode_t;

// Header raw
typedef struct {
    uint32_t frame_cnt;
    uint16_t udp_total;
    uint16_t udp_idx;
    uint16_t flags;
    uint16_t length;
    int32_t  reserved0;
    int32_t  reserved1;
    int16_t  reserved2;
    uint16_t crc16;
} hdr_raw_t;

// Received raw message
typedef struct {
    pre_hdr_t pre_header;
    hdr_raw_t raw_header;
    uint8_t data[UDP_PAYLOAD_SIZE];
} msg_raw_t;

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
} hdr_heat_t;

// Received heat map message
typedef struct {
	pre_hdr_t pre_header;
	hdr_heat_t heat_header;
    uint8_t data[UDP_PAYLOAD_SIZE];
} msg_heat_t;

typedef struct {
    uint16_t raw_frame_num;
    int16_t reserved0;
    int32_t reserved[7];
} args_get_raw_t;

// Get raw message
typedef struct {
    pre_hdr_t pre_header;
    args_get_raw_t raw_args;
} get_raw_t;

// Header
typedef struct {
	uint32_t frame_cnt;
	uint16_t udp_total;
	uint16_t udp_idx;
	uint16_t flags;
	uint16_t udp_pnts_num;
	float    self_velocity;
	float    noise_lvl;
	uint8_t  pcl_type;
	uint8_t  data_type;
	uint8_t  time_type;
	uint8_t  reserved0;
	uint64_t  tv_usec;
	int32_t  reserved1;
	int16_t  reserved2;
	uint16_t crc16;
} hdr_pcl_t;

// Single point
typedef struct {
	float range;
	float velocity;
	float azm;
	float elv;
	float snr;
	float rcs;
	int32_t reserved0;
} pcl_data_t;

// PCL message UDP-packet structure
typedef struct {
	pre_hdr_t  pre_header;
    hdr_pcl_t  pcl_header;
    pcl_data_t payload[UDP_PNTS_MAX];
} msg_pcl_t;

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
} sock_data_t;

#endif