#ifndef __DPD_HW_H__
#define __DPD_HW_H__

#include "dpd_hw_t.h"

int32_t dpd_hw_mem_write(uint32_t addr, uint32_t data);
int32_t dpd_hw_mem_read(uint32_t addr, uint32_t *data);

uint8_t dpd_download_waveform(uint32_t *waveform_i, uint32_t *waveform_q, uint32_t size);


uint8_t dpd_luts_access_test(void);
uint32_t dpd_register_write(uint8_t offset, uint32_t value);
uint32_t dpd_register_read(uint8_t offset);
uint8_t dpd_luts_write(uint8_t lutId, uint32_t *pLut);
uint8_t dpd_luts_read(uint8_t lutId, uint32_t *pLut);

uint32_t dpd_read_ipVersion(void);
uint64_t dpd_read_idMask(void);

uint32_t dpd_write_scratch_reg(uint32_t scratch);
uint32_t dpd_read_scratch_reg(void);
uint8_t dpd_write_act_out_sel(Dpd_ActOut_Sel sel);
uint8_t dpd_read_act_out_sel(void);
uint64_t dpd_write_lutid(uint64_t lutid);
uint64_t dpd_read_lutid(void);

uint8_t dpd_read_capture_buffer(uint8_t position, uint32_t *pBuf, uint32_t size);

uint8_t dpd_write_cap_control_reg(int8_t position, uint32_t ctrl);
uint32_t dpd_read_cap_control_reg(int8_t position);

uint32_t dpd_hw_open(void);
uint32_t dpd_hw_close(void);

uint32_t *dpd_hw_get_luts_entry(void);
uint8_t dpd_download_waveform_default(void);

#endif