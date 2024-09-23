#ifndef __APN_BOARDS_CUBUS_BBM_SRC_MTD_H
#define __APN_BOARDS_CUBUS_BBM_SRC_MTD_H
#include <nuttx/config.h>

#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>
#include <string.h>

#include <nuttx/spi/spi.h>
#include <stm32_spi.h>
#include <nuttx/board.h>
#include <nuttx/fs/fs.h>
#include <fcntl.h>
#include <nuttx/kmalloc.h>

#include <stdint.h>
#include <nuttx/mtd/mtd.h>

#define MAX_MTD_INSTANCES 5u

// The data needed to interface with mtd device's

typedef struct
{
    struct mtd_dev_s  *mtd_dev;
    int               *partition_block_counts;
    int               *partition_types;
    const char        **partition_names;
    struct mtd_dev_s  **part_dev;
    uint8_t           bus_id;
    uint32_t          devid;
    unsigned          n_partitions_current;
} mtd_instance_s;

typedef enum 
{
    MFT = 0,
    MTD = 1,
    LAST_MFT_TYPE
} cubus_manifest_types_e;

#define CUBUS_MFT_TYPES     {MFT, MTD}
#define CUBUS_MFT_STR_TYPES {"MFT", "MTD"}

typedef enum cubus_bus_type
{
    I2C = 0,
    SPI = 1,
    ONCHIP = 2,
    FLEXSPI = 3
} bus_type;

typedef struct 
{
    bus_type type;
    int bus_id;
    uint32_t devid;
} cubus_mft_device_t;

typedef struct 
{
    const cubus_manifest_types_e    type;
    const void                      *pmft;
} cubus_mft_entry_s;

typedef struct 
{
    const uint32_t  nmft;
    const cubus_mft_entry_s *mfts[];
} cubus_mft_s;

typedef enum 
{
    MTD_ID              = 1,
    MTD_MAINSTORAGE     = 2,
    MTD_SAT_LOG         = 3,
    MTD_BEACON          = 4,
    MTD_MISSION         = 5,
    MTD_FLAGS           = 6,
    MTD_RSV_TABLE       = 7
} cubus_mtd_types_t;

#define CUBUS_MFT_MTD_TYPES     {MTD_ID, MTD_MAINSTORAGE, MTD_SAT_LOG, MTD_BEACON, MTD_MISSION, MTD_FLAGS, MTD_RSV_TABLE}
#define CUBUS_MFT_MTD_STR_TYPES {"MTD_ID", "MTD_WAYPOINTS", "MTD_SAT_LOG", "MTD_BEACON", "MTD_MISSION", "MTD_FLAGS", "MTD_RSV_TABLE"}

typedef struct 
{
    const cubus_mtd_types_t         type;
    const char                      *path;
    const uint32_t                  nblocks;
} cubus_mtd_part_t;

typedef struct 
{
    const cubus_mft_device_t    *device;
    const uint32_t              npart;
    const cubus_mtd_part_t        partd[];
} cubus_mtd_entry_t;

typedef struct 
{
    const uint32_t              nconfigs;
    const cubus_mtd_entry_t     *entries[];
} cubus_mtd_manifest_t;


const cubus_mft_s* board_get_manifest(void);

/**
 * mtd operations
 */
int mt25ql_attach(mtd_instance_s *instance);
/*
  Get device complete geometry or a device
 */
int  cubus_mtd_get_geometry(const mtd_instance_s *instance, unsigned long *blocksize, unsigned long *erasesize,
				   unsigned long *neraseblocks, unsigned *blkpererase, unsigned *nblocks,
				   unsigned *partsize);
/*
  Get size of a parttion on an instance.
 */
ssize_t cubus_mtd_get_partition_size(const mtd_instance_s *instance, const char *partname);

/*
 * Get device an pinter to the array of mtd_instance_s of the system
 *  count - receives the number of instances pointed to by the pointer
 *  retunred.
 *
 *  returns: - A pointer to the mtd_instance_s of the system
 *            This can be  Null if there are no mtd instances.
 *
 */
mtd_instance_s **cubus_mtd_get_instances(unsigned int *count);

/************************************************************************************
 * Name: cubus_mtd_config
 *
 * Description:
 *   A board will call this function, to set up the mtd partitions
 *
 * Input Parameters:
 *  mtd_list    - cubus_mtd_config list/count
 *
 * Returned Value:
 *   non zero if error
 *
 ************************************************************************************/
int cubus_mtd_config(const cubus_mtd_manifest_t *mft_mtd);

/************************************************************************************
 * Name: cubus_mtd_query
 *
 * Description:
 *   A Query interface that will lookup a type and either a) verify it exists  by
 *   value.
 *
 *   or it will return the path for a type.
 *
 *
 * Input Parameters:
 *  type  - a string-ized version of cubus_mtd_types_t
 *  value - string to verity is that type.
 *  get   - a pointer to a string to optionally return the path for the type.
 *
 * Returned Value:
 *   non zero if error
 *   0 (get == null) item by type and value was found.
 *   0 (get !=null) item by type's value is returned at get;
 *
 ************************************************************************************/
int cubus_mtd_query(const char *type, const char *val, const char **get);

/************************************************************************************
 * Name: cubus_mft_configure
 *
 * Description:
 *   The Cubus layer will provide this interface to start/configure the
 *   hardware.
 *
 * Input Parameters:
 *  mft    - a pointer to the manifest
 *
 * Returned Value:
 *   non zero if error
 *
 ************************************************************************************/

int cubus_mft_configure(const cubus_mft_s *mft);

#endif  // __APN_BOARDS_CUBUS_BBM_SRC_MTD_H