#ifndef __DPD_HW_T_H__
#define __DPD_HW_T_H__

#include <stdint.h>
#include <string.h>

#define XPAR_AXI_DPD_ACTUATOR_0_BASEADDR 0x46000000
#define XPAR_AXI_DPD_CAPTURE_0_BASEADDR  0x46100000
#define XPAR_AXI_DPD_CAPTURE_1_BASEADDR  0x46200000
#define XPAR_AXI_DPD_TX_BUFF_0_BASEADDR  0x46300000
#define XPAR_AXI_DPD_TX_BUFF_1_BASEADDR  0x46400000

// DPD Base Address
#define DPD_CTRL_BASEADDR       XPAR_AXI_DPD_ACTUATOR_0_BASEADDR
#define DPD_MEM_BASEADDR        XPAR_AXI_DPD_ACTUATOR_0_BASEADDR + 0x8000
#define DPD_CAP0_BASEADDR       XPAR_AXI_DPD_CAPTURE_0_BASEADDR
#define DPD_CAP1_BASEADDR       XPAR_AXI_DPD_CAPTURE_1_BASEADDR
#define DPD_TX_BUFF0_BASEADDR   XPAR_AXI_DPD_TX_BUFF_0_BASEADDR
#define DPD_TX_BUFF1_BASEADDR   XPAR_AXI_DPD_TX_BUFF_1_BASEADDR
#define DPD_LUT_DEPTH           1024
#define DPD_LUT_MAX             64
#define DPD_CAP_SIZE            4096
#define DPD_CTRL_SIZE           0x8000
#define DPD_MEM_SIZE            0x8000
#define DPD_CAP0_SIZE           0x10000
#define DPD_CAP1_SIZE           0x10000
#define DPD_TX_BUFF0_SIZE       0x10000
#define DPD_TX_BUFF1_SIZE       0x10000

// DPD physic address device
#define DPD_HW_PHY_DEV          "/dev/mem"

// DPD Ctrl Registers
#define ADDR_IP_VERSION         0x0000
#define ADDR_ID_MASK_LOW        0x0004
#define ADDR_ID_MASK_HIGH       0x0008
#define ADDR_SCRATCH            0x000C
#define ADDR_ACT_OUT_SEL        0x0010
#define ADDR_LUTID_L            0x0014
#define ADDR_LUTID_H            0x0018

// DPD ACT Out Select
typedef enum {
    DPD_ENABLE      = 0x01u,
    DPD_BYPASS      = 0x02u,
    DPD_SHUTDOWN    = 0x04u,
    DPD_FREEZE      = 0x08u,
}Dpd_ActOut_Sel;

enum dpd_hw_init_e {
    DPD_HW_UNININED = 0x00,
    DPD_HW_ININED   = 0x01,
};

typedef struct tag_dpd_mem_space_t {
    uint32_t phy_base;
    uint32_t phy_size;
} dpd_mem_space_t;

typedef struct tag_dpd_hw_t {    
    uint8_t init_flag;
    int mem_fd;
    dpd_mem_space_t dpd_ctrl;
    dpd_mem_space_t dpd_mem;
    dpd_mem_space_t dpd_cap0;
    dpd_mem_space_t dpd_cap1;
} dpd_hw_t;

#endif
