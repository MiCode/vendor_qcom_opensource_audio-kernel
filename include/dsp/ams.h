/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */
#ifndef __AMS_H__
#define __AMS_H__


#define DSP_AMS_CMD_MEM_MAP (0x0001336E)
/** Memory region. */
typedef struct dsp_ams_mem_region_t dsp_ams_mem_region_t;

struct dsp_ams_mem_region_t {
	uint32_t addr_lsw;
	/**< Lower 32 bits of the memory address to map. */
	uint32_t addr_msw;
	/**< Higher 32 bits of the memory address to map. */
	uint32_t size_bytes;
	/**< Size of the memory in bytes. */
};

/** Command payload for DSP_AMS_CMD_MEM_MAP */
typedef struct dsp_ams_cmd_mem_map_t dsp_ams_cmd_mem_map_t;

struct dsp_ams_cmd_mem_map_t {
	uint32_t num_regions;
	/**< Number of regions to map. */

	/** Followed by array below.
	The start address for the array must be 4-byte aligned.
	dsp_ams_mem_region_t regions[num_regions]; */
};


#define DSP_AMS_CMD_SET_PARAM (0x00013371)

typedef struct dsp_ams_cmd_set_param_t dsp_ams_cmd_set_param_t;

struct dsp_ams_cmd_set_param_t {
	uint32_t handle;
	/**< Graph handle. */
	uint32_t module_id;
	/**< Module ID: user assigned ID in dsp_ams_module_t
		Note this is not the CAPIv2 module ID. */
	uint32_t param_id;
	/**< Parameter ID. */
	uint32_t param_size;
	/**< Parameter data size in bytes. */
	uint32_t mem_map_handle;
	/**< Memory map handle if the parameter data is in shared memory.
		The shared memory contains uint8_t data[param_size].
		Set to 0 if the parameter is in-band. */
	uint32_t address_lsw;
	/**< Lower 32 bit of the parameter data address if data is in shared
		memory. */
	uint32_t address_msw;
	/**< Higher 32 bit of the parameter data address if data is in shared
		memory. */
};

#define DSP_AMS_CMD_GET_PARAM (0x00013372)

typedef struct dsp_ams_cmd_get_param_t dsp_ams_cmd_get_param_t;

struct dsp_ams_cmd_get_param_t {
	uint32_t handle;
	/**< Graph handle. */
	uint32_t module_id;
	/**< Module ID: user assigned ID in dsp_ams_module_t
		Note this is not the CAPIv2 module ID. */
	uint32_t param_id;
	/**< Parameter ID. */
	uint32_t param_max_size;
	/**< Maximum size of the parameter data based on the module ID/parameter ID
		combination.
		For out-of-band mode, the shared memory provided must be at least this
		size. */
	uint32_t mem_map_handle;
	/**< Memory map handle if shared memory is provided. Parameter data
		(dsp_ams_param_data_t param_data) will be written into the memory.
		Set to 0 if the parameter is to be returned in-band. */
	uint32_t address_lsw;
	/**< Lower 32 bit of the memory provided. */
	uint32_t address_msw;
	/**< Higher 32 bit of the memory provided. */
};

#endif