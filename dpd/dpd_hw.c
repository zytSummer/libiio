
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "dpd_err_codes_t.h"
#include "dpd_hw.h"
static dpd_hw_t s_g_dpd_hw;

static int32_t hw_mem_write(uint32_t base, uint32_t offset, uint32_t data)
{
    int ret = 0;
    uint32_t phy_size = 0x00; 
    void *vir_base = NULL;

    if (s_g_dpd_hw.init_flag != DPD_HW_ININED)
    {
        printf("Warning!! dpd module has been initialed!\n");
        return DPD_HW_MEM_DEVICE_UNINTED_ERROR;
    }
    
    if (base == DPD_CTRL_BASEADDR && offset < DPD_CTRL_SIZE)
    {
        phy_size = DPD_CTRL_SIZE;
    } 
    else if (base == DPD_MEM_BASEADDR && offset < DPD_MEM_SIZE)
    {
        phy_size = DPD_MEM_SIZE;
    } 
    else if (base == DPD_CAP0_BASEADDR && offset < DPD_CAP0_SIZE)
    {
        phy_size = DPD_CAP0_SIZE;
    } 
    else if (base == DPD_CAP1_BASEADDR && offset < DPD_CAP1_SIZE)
    {
        phy_size = DPD_CAP1_SIZE;
    }
    else
    {
        return -1;
    }

    vir_base = mmap(NULL, phy_size, PROT_READ | PROT_WRITE, MAP_SHARED, \
                                    s_g_dpd_hw.mem_fd, base & ~phy_size);
    if (vir_base == (void *) -1)
    {
        printf("ERROR!! memory map failed at physical address 0x%x\n", phy_size);
        ret = DPD_HW_MEM_MAP_ERROR;
    }
    
    *(uint32_t *)((uintptr_t)vir_base + offset) = data;

    if(munmap(vir_base, phy_size) == -1)
		ret = DPD_HW_MEM_MAP_ERROR;

    return ret;
}

static int32_t hw_mem_read(uint32_t base, uint32_t offset, uint32_t *data)
{
    int ret = 0;
    uint32_t phy_size = 0x00; 
    void *vir_base = NULL;

    if (s_g_dpd_hw.init_flag != DPD_HW_ININED)
    {
        printf("Warning!! dpd module has been initialed!\n");
        return DPD_HW_MEM_DEVICE_UNINTED_ERROR;
    }
    
    if (base == DPD_CTRL_BASEADDR && offset < DPD_CTRL_SIZE)
    {
        phy_size = DPD_CTRL_SIZE;
    } 
    else if (base == DPD_MEM_BASEADDR && offset < DPD_MEM_SIZE)
    {
        phy_size = DPD_MEM_SIZE;
    } 
    else if (base == DPD_CAP0_BASEADDR && offset < DPD_CAP0_SIZE)
    {
        phy_size = DPD_CAP0_SIZE;
    } 
    else if (base == DPD_CAP1_BASEADDR && offset < DPD_CAP1_SIZE)
    {
        phy_size = DPD_CAP1_SIZE;
    }
    else
    {
        return -1;
    }

    vir_base = mmap(NULL, phy_size, PROT_READ | PROT_WRITE, MAP_SHARED, \
                                    s_g_dpd_hw.mem_fd, base & ~phy_size);
    if (vir_base == (void *) -1)
    {
        printf("ERROR!! memory map failed at physical address 0x%x\n", phy_size);
        ret = DPD_HW_MEM_MAP_ERROR;
    }
  
    *data = *(uint32_t *)((uintptr_t)vir_base + offset);

    return ret;
}

int32_t dpd_hw_mem_write(uint32_t addr, uint32_t data)
{
    int ret = 0;
    uint32_t base = 0x00;
    uint32_t offset = 0x00;
    
    if (addr >= DPD_CTRL_BASEADDR && addr <= DPD_CTRL_BASEADDR + DPD_CTRL_SIZE)
    {
        base = DPD_CTRL_BASEADDR;
        offset = addr - DPD_CTRL_BASEADDR;
    } 
    else if (addr >= DPD_MEM_BASEADDR && addr <= DPD_MEM_BASEADDR + DPD_MEM_SIZE)
    {
        base = DPD_MEM_BASEADDR;
        offset = addr - DPD_MEM_BASEADDR;
    } 
    else if (addr >= DPD_CAP0_BASEADDR && addr <= DPD_CAP0_BASEADDR + DPD_CAP0_SIZE)
    {
        base = DPD_CAP0_BASEADDR;
        offset = addr - DPD_CAP0_BASEADDR;
    } 
    else if (addr >= DPD_CAP1_BASEADDR && addr <= DPD_CAP1_BASEADDR + DPD_CAP1_SIZE)
    {
        base = DPD_CAP1_BASEADDR;
        offset = addr - DPD_CAP1_BASEADDR;
    }
    else
    {
        return -1;
    }
    
    ret = hw_mem_write(base, offset, data);

    return ret;
}

int32_t dpd_hw_mem_read(uint32_t addr, uint32_t *data)
{
    int ret = 0;
    uint32_t base = 0x00;
    uint32_t offset = 0x00;
    
    if (addr >= DPD_CTRL_BASEADDR && addr <= DPD_CTRL_BASEADDR + DPD_CTRL_SIZE)
    {
        base = DPD_CTRL_BASEADDR;
        offset = addr - DPD_CTRL_BASEADDR;
    } 
    else if (addr >= DPD_MEM_BASEADDR && addr <= DPD_MEM_BASEADDR + DPD_MEM_SIZE)
    {
        base = DPD_MEM_BASEADDR;
        offset = addr - DPD_MEM_BASEADDR;
    } 
    else if (addr >= DPD_CAP0_BASEADDR && addr <= DPD_CAP0_BASEADDR + DPD_CAP0_SIZE)
    {
        base = DPD_CAP0_BASEADDR;
        offset = addr - DPD_CAP0_BASEADDR;
    } 
    else if (addr >= DPD_CAP1_BASEADDR && addr <= DPD_CAP1_BASEADDR + DPD_CAP1_SIZE)
    {
        base = DPD_CAP1_BASEADDR;
        offset = addr - DPD_CAP1_BASEADDR;
    }
    else
    {
        return -1;
    }
    
    ret = hw_mem_read(base, offset, data);
    return ret;
}


uint8_t dpd_luts_access_test(void)
{
	uint8_t errCode = 0;
	uint32_t entry_addr[4] = {ADDR_IP_VERSION, ADDR_ID_MASK_LOW, ADDR_ID_MASK_HIGH, ADDR_SCRATCH};
	uint32_t wr_data[4] = {0x0000, 0x0000, 0x0000, 0x1234};
	uint32_t rd_data[4] = {0};

	// DPD Control Register Access Test
	for(int i = 0; i < sizeof(entry_addr)/sizeof(uint32_t); i++)
	{
		hw_mem_write(s_g_dpd_hw.dpd_ctrl.phy_base, entry_addr[i], wr_data[i]);

		hw_mem_read(s_g_dpd_hw.dpd_ctrl.phy_base, entry_addr[i], &rd_data[i]);
	}

    // ADDR_SCRATCH, scratch register
	if(wr_data[3] != rd_data[3])
	{
		errCode = 1;
	}

	// DPD Luts Access Test
    // Only verify the first and last 5 words
    if(errCode == 0)
    {
        dpd_write_lutid(0x01);
        for(int i = 0; i < 5; i++)
        {
            hw_mem_write(s_g_dpd_hw.dpd_mem.phy_base, i*4, 0x11111111*(i+1));
            hw_mem_read(s_g_dpd_hw.dpd_mem.phy_base, i*4, &rd_data[i]);
#if 0
            if(rd_data[i] != 0x11111111*(i+1))
            {
                errCode = 1;
                break;
            }
#endif
        }
    }

    if(errCode == 0)
    {
    	int end = DPD_LUT_DEPTH * 4;
        for(int i = 0; i < 5; i++)
        {
            hw_mem_write(s_g_dpd_hw.dpd_mem.phy_base, end-(i+1)*4, 0x11111111*(i+1));
            hw_mem_read(s_g_dpd_hw.dpd_mem.phy_base, end-(i+1)*4, &rd_data[i]);

            if(rd_data[i] != 0x11111111*(i+1))
            {
                errCode = 1;
                break;
            }
        }
    }

	return errCode;
}

uint8_t dpd_luts_write(uint8_t lutId, uint32_t *pLut)
{
    uint8_t errCode = 0u;
    uint64_t idMask = (1lu << lutId);

    // write lutid
    dpd_write_lutid(idMask);
    if((lutId >= DPD_LUT_MAX) || (pLut == NULL))
    {
        errCode = 1;
    }
    
    if(errCode == 0)
    {
        for(int i = 0; i < DPD_LUT_DEPTH; i++)
        {
            hw_mem_write(s_g_dpd_hw.dpd_mem.phy_base, i*4, pLut[i]);
        }
    }

    return errCode;
}

uint8_t dpd_luts_read(uint8_t lutId, uint32_t *pLut)
{
    uint8_t errCode = 0u;
    uint64_t idMask = (1lu << lutId);

    // write lutid
    dpd_write_lutid(idMask);

    if((lutId >= DPD_LUT_MAX) || (pLut == NULL))
    {
        errCode = 1;
    }

    if(errCode == 0)
    {
        for(int i = 0; i < DPD_LUT_DEPTH; i++)
        {
            hw_mem_read(s_g_dpd_hw.dpd_mem.phy_base, i*4, &(pLut[i]));
        }
    }

    return errCode;
}

uint32_t dpd_read_ipVersion(void)
{
    uint32_t ipVersion = 0;
    hw_mem_read(s_g_dpd_hw.dpd_ctrl.phy_base, ADDR_IP_VERSION, &ipVersion);
    return ipVersion;
}

uint64_t dpd_read_idMask(void)
{
    uint32_t idMask_high = 0u, idMask_low = 0u;
    hw_mem_read(s_g_dpd_hw.dpd_ctrl.phy_base, ADDR_ID_MASK_LOW, &idMask_low);
    hw_mem_read(s_g_dpd_hw.dpd_ctrl.phy_base, ADDR_ID_MASK_HIGH, &idMask_high);
    return ((uint64_t)idMask_high << 32u) | idMask_low;
}

uint32_t dpd_write_scratch_reg(uint32_t scratch)
{
    hw_mem_write(s_g_dpd_hw.dpd_ctrl.phy_base, ADDR_SCRATCH, scratch);
    return dpd_read_scratch_reg();
}

uint32_t dpd_read_scratch_reg(void)
{
    uint32_t scratch = 0;
    hw_mem_read(s_g_dpd_hw.dpd_ctrl.phy_base, ADDR_SCRATCH, &scratch);
    return scratch;
}

uint8_t dpd_write_act_out_sel(Dpd_ActOut_Sel sel)
{
	hw_mem_write(s_g_dpd_hw.dpd_ctrl.phy_base, ADDR_ACT_OUT_SEL, (uint32_t)sel);
	return dpd_read_act_out_sel();
}

uint8_t dpd_read_act_out_sel(void)
{
    uint32_t outSel;
    hw_mem_read(s_g_dpd_hw.dpd_ctrl.phy_base, ADDR_ACT_OUT_SEL, &outSel);
    return (uint8_t)outSel;
}

uint64_t dpd_write_lutid(uint64_t lutid)
{
	uint32_t lutid_low, lutid_high;
	lutid_high = (lutid >> 32u) & 0xFFFFFFFF;
	lutid_low = lutid & 0xFFFFFFFF;
	hw_mem_write(s_g_dpd_hw.dpd_ctrl.phy_base, ADDR_LUTID_H, lutid_high);
	hw_mem_write(s_g_dpd_hw.dpd_ctrl.phy_base, ADDR_LUTID_L, lutid_low);
	return dpd_read_lutid();
}

uint64_t dpd_read_lutid(void)
{
	uint32_t lutid_low, lutid_high;
    hw_mem_read(s_g_dpd_hw.dpd_ctrl.phy_base, ADDR_LUTID_L, &lutid_low);
    hw_mem_read(s_g_dpd_hw.dpd_ctrl.phy_base, ADDR_LUTID_H, &lutid_high);
    return (((uint64_t)lutid_high << 32u) | (lutid_low));
}

uint32_t dpd_register_write(uint8_t offset, uint32_t value)
{
    offset = offset / 4 * 4;
    hw_mem_write(s_g_dpd_hw.dpd_ctrl.phy_base, offset, value);
    return dpd_register_read(offset);
}

uint32_t dpd_register_read(uint8_t offset)
{
    uint32_t regVal = 0;
    offset = offset / 4 * 4;
    hw_mem_read(s_g_dpd_hw.dpd_ctrl.phy_base, offset, &regVal);
    return regVal;
}

uint8_t dpd_read_capture_buffer(uint8_t position, uint32_t *pBuf, uint32_t size)
{
	uint8_t err = 0u;
	uint32_t base = (position == 0u) ? s_g_dpd_hw.dpd_cap0.phy_base : s_g_dpd_hw.dpd_cap1.phy_base;
	uint32_t offset = 0u;

	if((pBuf == NULL) || (size > DPD_CAP_SIZE))
	{
		err = 1u;
	}

	if(err == 0u)
	{
		// read data capture in FPGA RAM
		for(uint32_t i = 0u; i < size; i++)
		{
			offset = i * 4;
			hw_mem_read(base, offset, (pBuf+i));
		}
	}

	return err;
}

uint8_t dpd_write_cap_control_reg(int8_t position, uint32_t ctrl)
{
	uint8_t err = 0u;
	uint32_t base = (position == 0u) ? s_g_dpd_hw.dpd_cap0.phy_base : s_g_dpd_hw.dpd_cap1.phy_base;
	uint32_t offset = 0x8000u;

	// write capture control register
	hw_mem_write(base, offset, ctrl);
	return err;
}

uint32_t dpd_read_cap_control_reg(int8_t position)
{
	uint32_t ret = 0u;
	uint32_t base = (position == 0u) ? s_g_dpd_hw.dpd_cap0.phy_base : s_g_dpd_hw.dpd_cap1.phy_base;
	uint32_t offset = 0x8000u;

	// write capture control register
	hw_mem_read(base, offset, &ret);
	return ret;
}

uint32_t dpd_hw_open(void)
{
    uint32_t ret = 0u;
    
    if (s_g_dpd_hw.init_flag == DPD_HW_ININED)
    {
        printf("Warning!! dpd module has been initialed!\n");
        goto inited;
    }

    s_g_dpd_hw.init_flag = DPD_HW_UNININED;

    s_g_dpd_hw.mem_fd = open(DPD_HW_PHY_DEV, O_RDWR | O_SYNC);
    if (-1 == s_g_dpd_hw.mem_fd)
    {
        printf("ERROR!! %s open failed!\n", DPD_HW_PHY_DEV);
        ret = DPD_HW_MEM_DEVICE_OPEN_ERROR;
        goto inited;
    }

    s_g_dpd_hw.dpd_ctrl.phy_size = DPD_CTRL_SIZE;
    s_g_dpd_hw.dpd_ctrl.phy_size = DPD_CTRL_BASEADDR;

    s_g_dpd_hw.dpd_mem.phy_size = DPD_MEM_SIZE;
    s_g_dpd_hw.dpd_mem.phy_size = DPD_MEM_BASEADDR;

    s_g_dpd_hw.dpd_cap0.phy_size = DPD_CAP0_SIZE;
    s_g_dpd_hw.dpd_cap0.phy_size = DPD_CAP0_BASEADDR;

    s_g_dpd_hw.dpd_cap1.phy_size = DPD_CAP1_SIZE;
    s_g_dpd_hw.dpd_cap1.phy_size = DPD_CAP1_BASEADDR;

    s_g_dpd_hw.init_flag = DPD_HW_ININED;

inited:
    return ret;
}

uint32_t dpd_hw_close(void)
{
    uint32_t ret = 0u;
    if (s_g_dpd_hw.init_flag == DPD_HW_UNININED)
    {
        printf("Warning!! dpd module has not been initialed!\n");
        goto closed;
    }
#if 0    
	if(munmap(s_g_dpd_hw.dpd_ctrl_addr, DPD_CTRL_SIZE) == -1)
		ret = DPD_HW_MEM_MAP_ERROR;

    if(munmap(s_g_dpd_hw.dpd_mem_addr, DPD_MEM_SIZE) == -1)
		ret |= DPD_HW_MEM_MAP_ERROR;

	if(munmap(s_g_dpd_hw.dpd_cap0_addr, DPD_CAP0_SIZE) == -1)
		ret = DPD_HW_MEM_MAP_ERROR;
    
    if(munmap(s_g_dpd_hw.dpd_cap0_addr, DPD_CAP1_SIZE) == -1)
		ret = DPD_HW_MEM_MAP_ERROR;
#endif
    close(s_g_dpd_hw.mem_fd);
    
    s_g_dpd_hw.init_flag = DPD_HW_UNININED;

closed:
    return ret;
}