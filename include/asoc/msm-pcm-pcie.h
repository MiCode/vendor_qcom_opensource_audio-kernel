/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

extern int data_cmd_read_done(void*);

extern int data_cmd_write_done(void*);

extern int update_pcie_memhandle(void *data , void *paddr, struct gpr_device * gprdev);
extern int update_pcie_wr_memhandle(uint32_t wr_mem_handle, void *paddr, struct gpr_device * gprdev);
extern int update_pcie_rd_memhandle(uint32_t rd_mem_handle, void *paddr, struct gpr_device * gprdev);

extern void msm_pcm_pcie_crash_handler(void);
