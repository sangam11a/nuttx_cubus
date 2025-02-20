#include <nuttx/config.h>
#include <nuttx/fs/fs.h>
#include <nuttx/mtd/mtd.h>
#include <../../../nuttx/fs/littlefs/lfs_vfs.h>
#include <sys/mount.h>
#include <nuttx/spi/spi.h>

#include <inttypes.h>
#include <errno.h>
#include <stdbool.h>
#include "cubus_mtd.h"
#include <sys/statfs.h>

extern struct mtd_dev_s *mtd_partition(FAR struct mtd_dev_s *mtd,
                                       off_t firstblock, off_t nblocks);
static int num_instances = 0;
static int total_blocks = 0;
static mtd_instance_s *instances[MAX_MTD_INSTANCES] = {};

static const cubus_mft_device_t spi3_dev = { // MT25QL on FMUM 1Gb 2048 X 64K
    .type = SPI,
    .bus_id = 3,
    .devid = SPIDEV_FLASH(0)};

static const cubus_mft_device_t spi4_dev = { // MT25QL on FMUM 1Gb 2048 X 64K
    .type = SPI,
    .bus_id = 4,
    .devid = SPIDEV_FLASH(0)};

static const cubus_mtd_entry_t cubus_mfm = {
    .device = &spi3_dev,
    .npart = 2,
    .partd = {
        {
            .type = MTD_MAINSTORAGE, // storage space for  HK data logging, flag data storage, reservation table.
            .path = "/fs/mfm/mtd_mainstorage",
            // .nblocks = 51200				// 12.5MB in no of pages, each page having 256 bytes
            .nblocks = 262144 // 64 MB in no of pages
        },
        {
            .type = MTD_MISSION, // storage space for missions
            .path = "/fs/mfm/mtd_mission",
            .nblocks = 262144 // 64 MB in no of pages
        }},
};

static const cubus_mtd_entry_t cubus_sfm = {
    .device = &spi4_dev,
    .npart = 2,
    .partd = {
        {
            .type = MTD_MAINSTORAGE, // Partition for storing MSN data
            .path = "/fs/sfm/mtd_mainstorage",
            .nblocks = 262144 // 128 MB in no of pages, each pages having 256 bytes
        },
        {
            .type = MTD_MISSION, // storage space for missions
            .path = "/fs/sfm/mtd_mission",
            .nblocks = 262144 // 64 MB in no of pages
        }},
};

static const cubus_mtd_manifest_t board_mfm_mtd_config = {
    .nconfigs = 1,
    .entries = {
        &cubus_mfm}};

static const cubus_mft_entry_s mtd_mft = {
    .type = MTD,
    .pmft = (void *)&board_mfm_mtd_config,
};

static const cubus_mft_s mft = {
    .nmft = 1,
    .mfts = {
        &mtd_mft}};

const cubus_mft_s *board_mfm_get_manifest(void)
{
    return &mft;
}

static const cubus_mtd_manifest_t board_sfm_mtd_config = {
    .nconfigs = 1,
    .entries = {
        &cubus_sfm}};

static const cubus_mft_entry_s mtd_mft_sfm = {
    .type = MTD,
    .pmft = (void *)&board_sfm_mtd_config,
};

static const cubus_mft_s mft_sfm = {
    .nmft = 1,
    .mfts = {
        &mtd_mft_sfm}};

const cubus_mft_s *board_sfm_get_manifest(void)
{
    return &mft_sfm;
}

int mt25ql_attach(mtd_instance_s *instance)
{
    syslog(LOG_INFO, "Starting MTD, MT25QL driver\n");

    /* start the MT25QL driver, attempt 10 times */

    int spi_speed_mhz = 21;

    for (int i = 0; i < 21; i++)
    {
        /* initialize the right spi */
        struct spi_dev_s *spi = stm32_spibus_initialize(instance->bus_id);

        if (spi == NULL)
        {
            syslog(LOG_ERR, "failed to locate spi bus");
            return ENXIO;
        }

        /* this resets the spi bus, set correct bus speed again */
        SPI_LOCK(spi, true);
        SPI_SETFREQUENCY(spi, spi_speed_mhz * 1000 * 1000);
        SPI_SETBITS(spi, 8);
        SPI_SETMODE(spi, SPIDEV_MODE0);
        SPI_SELECT(spi, instance->devid, false);
        SPI_LOCK(spi, false);

        instance->mtd_dev = mt25ql_initialize(spi);

        if (instance->mtd_dev)
        {
            /* abort on first valid result */
            if (i > 0)
            {
                syslog(LOG_WARNING, "mtd needed %d attempts to attach", i + 1);
            }
            break;
        }

        // try reducing speed for next attempt
        spi_speed_mhz--;
        usleep(10000);
    }

    /* if last attempt is still unsuccessful, abort */
    if (instance->mtd_dev == NULL)
    {
        syslog(LOG_ERR, "failed to initialize mtd driver");
        return -EIO;
    }

    int ret = instance->mtd_dev->ioctl(instance->mtd_dev, MTDIOC_SETSPEED, (unsigned long)spi_speed_mhz * 1000 * 1000);

    if (ret != OK)
    {
        // FIXME: From the previous warning call, it looked like this should have been fatal error instead. Tried
        // that but setting the bus speed does fail all the time. Which was then exiting and the board would
        // not run correctly. So changed to CUBUS_WARN.
        syslog(LOG_WARNING, "failed to set bus speed");
    }

    return 0;
}

int cubus_mtd_get_geometry(const mtd_instance_s *instance, unsigned long *blocksize, unsigned long *erasesize,
                           unsigned long *neraseblocks,
                           unsigned *blkpererase, unsigned *nblocks, unsigned *partsize)
{
    FAR struct mtd_geometry_s geo;

    int ret = instance->mtd_dev->ioctl(instance->mtd_dev, MTDIOC_GEOMETRY, (unsigned long)((uintptr_t)&geo));

    if (ret < 0)
    {
        syslog(LOG_ERR, "mtd->ioctl failed: %d", ret);
        return ret;
    }

    *blocksize = geo.blocksize;
    *erasesize = geo.erasesize;
    *neraseblocks = geo.neraseblocks;

    /* Determine the size of each partition.  Make each partition an even
     * multiple of the erase block size (perhaps not using some space at the
     * end of the FLASH).
     */

    *blkpererase = geo.erasesize / geo.blocksize;
    *nblocks = (geo.neraseblocks / instance->n_partitions_current) * *blkpererase;
    *partsize = *nblocks * geo.blocksize;

    return ret;
}

/*
  get partition size in bytes
 */
ssize_t cubus_mtd_get_partition_size(const mtd_instance_s *instance, const char *partname)
{
    unsigned long blocksize, erasesize, neraseblocks;
    unsigned blkpererase, nblocks, partsize = 0;

    int ret = cubus_mtd_get_geometry(instance, &blocksize, &erasesize, &neraseblocks, &blkpererase, &nblocks, &partsize);

    if (ret != OK)
    {
        syslog(LOG_ERR, "Failed to get geometry");
        return 0;
    }

    unsigned partn = 0;

    for (unsigned n = 0; n < instance->n_partitions_current; n++)
    {
        if (instance->partition_names[n] != NULL &&
            partname != NULL &&
            strcmp(instance->partition_names[n], partname) == 0)
        {
            partn = n;
            break;
        }
    }

    return instance->partition_block_counts[partn] * blocksize;
}

mtd_instance_s **cubus_mtd_get_instances(unsigned int *count)
{
    *count = num_instances;
    return instances;
}

int cubus_mtd_config(const cubus_mtd_manifest_t *mft_mtd, int block_number)
{
    // stm32_gpiowrite();
    int rv = -EINVAL;
    total_blocks = block_number;

    const cubus_mtd_manifest_t *mtd_list = mft_mtd;

    if (mtd_list == NULL)
    {
        syslog(LOG_ERR, "Invalid mtd configuration!\n");
        return rv;
    }

    if (mtd_list->nconfigs == 0)
    {
        return 0;
    }

    rv = -ENOMEM;
    uint8_t total_new_instances = mtd_list->nconfigs + num_instances;

    if (total_new_instances >= MAX_MTD_INSTANCES)
    {
        syslog(LOG_ERR, "reached limit of max %u mtd instances", MAX_MTD_INSTANCES);
        return rv;
    }

    for (uint8_t i = num_instances, num_entry = 0u; i < total_new_instances; ++i, ++num_entry)
    {

        instances[i] = (mtd_instance_s *)malloc(sizeof(mtd_instance_s));

        if (instances[i] == NULL)
        {
        memoryout:
            syslog(LOG_ERR, "failed to allocate memory!");
            return rv;
        }

        num_instances++;

        uint32_t nparts = mtd_list->entries[num_entry]->npart;
        instances[i]->devid = mtd_list->entries[num_entry]->device->devid;
        instances[i]->bus_id = mtd_list->entries[num_entry]->device->bus_id;
        instances[i]->mtd_dev = NULL;
        instances[i]->n_partitions_current = 0;

        rv = -ENOMEM;
        instances[i]->part_dev = (struct mtd_dev_s **)malloc(nparts * sizeof(struct mtd_dev_s));

        if (instances[i]->part_dev == NULL)
        {
            goto memoryout;
        }

        instances[i]->partition_block_counts = (int *)malloc(nparts * sizeof(int));

        if (instances[i]->partition_block_counts == NULL)
        {
            goto memoryout;
        }

        instances[i]->partition_types = (int *)malloc(nparts * sizeof(int));

        if (instances[i]->partition_types == NULL)
        {
            goto memoryout;
        }

        instances[i]->partition_names = (const char **)malloc(nparts * sizeof(const char));

        if (instances[i]->partition_names == NULL)
        {
            goto memoryout;
        }

        for (uint32_t p = 0; p < nparts; p++)
        {
            instances[i]->partition_block_counts[p] = mtd_list->entries[num_entry]->partd[p].nblocks;
            instances[i]->partition_names[p] = mtd_list->entries[num_entry]->partd[p].path;
            instances[i]->partition_types[p] = mtd_list->entries[num_entry]->partd[p].type;
        }

        if (mtd_list->entries[num_entry]->device->type == SPI)
        {

            rv = mt25ql_attach(instances[i]);
        }
        else if (mtd_list->entries[num_entry]->device->type == ONCHIP)
        {
            instances[i]->n_partitions_current++;
            return 0;
        }

        if (rv != 0)
        {
            goto errout;
        }

        unsigned long blocksize;
        unsigned long erasesize;
        unsigned long neraseblocks;
        unsigned int blkpererase;
        unsigned int nblocks;
        unsigned int partsize;

        rv = cubus_mtd_get_geometry(instances[i], &blocksize, &erasesize, &neraseblocks, &blkpererase, &nblocks, &partsize);

        if (rv != 0)
        {
            goto errout;
        }

        /* Now create MTD FLASH partitions */

        char blockname[32];
        char mount_point[52];

        unsigned long offset;
        unsigned part;

        for (offset = 0, part = 0; rv == 0 && part < nparts; offset += instances[i]->partition_block_counts[part], part++)
        {

            /* Create the partition */

            instances[i]->part_dev[part] = mtd_partition(instances[i]->mtd_dev, offset, instances[i]->partition_block_counts[part]);

            if (instances[i]->part_dev[part] == NULL)
            {
                syslog(LOG_ERR, "mtd_partition failed. offset=%lu nblocks=%u",
                       offset, nblocks);
                rv = -ENOSPC;
                goto errout;
            }

            /* Initialize to provide an FTL block driver on the MTD FLASH interface */

            snprintf(blockname, sizeof(blockname), "/dev/mtdblock%d", total_blocks);

            syslog(LOG_INFO, "blockname: %s, type: %d, name: %s\n", blockname, instances[i]->partition_types[part],
                   instances[i]->partition_names[part]);

            rv = register_mtddriver(blockname, instances[i]->part_dev[part], 0755, NULL);

            if (rv < 0)
            {
                syslog(LOG_ERR, "MTD driver %s failed: %d\n", blockname, rv);
                goto errout;
            }

            // snprintf(mount_point, sizeof(mount_point), "/mnt%s", instances[i]->partition_names[part]);
            memset(mount_point, '\n', sizeof(mount_point));
            sprintf(mount_point, "/mnt%s", instances[i]->partition_names[part]);
            printf("nx_mount: blockname: %s partition: %s mount_point: %s\n", blockname, instances[i]->partition_names[part],
                   mount_point);
            rv = nx_mount(blockname, mount_point, "littlefs", 0, "");
            if (rv < 0)
            {
                syslog(LOG_ERR, "NX_Mount %s on mount point: %s failed: %d\n", instances[i]->partition_names[part], mount_point, rv);
                syslog(LOG_INFO, "Trying to mount again");
                rv = nx_mount(blockname, mount_point, "littlefs", 1, "autoformat");
                if (rv < 0)
                {
                    syslog(LOG_ERR, "remount with autoformat failed.\n");
                    goto errout;
                }
            }
            else
            {
                syslog(LOG_INFO, "Mount Successful\n");

                syslog(LOG_INFO, "Performing write testing.");
                struct file file_p;
                char file_path[65];
                sprintf(file_path, "%s/test.txt", mount_point);
                // if(nx_umount2(mount_point,0) >=0){
                //     printf("\n\n\n-------------------uUnmount success ful-------------------------\n");
                // }
                // int fd = open(file_path, O_CREAT | O_RDWR);
                // int fd = file_open(&file_p, file_path, O_CREAT | O_RDWR | O_APPEND);
                // if(fd < 0)
                // {
                // 	syslog(LOG_ERR, "Error opening file in mainstorage of MFM.\n");
                // 	// close(fd);
                // 	file_close(&file_p);
                // } else {
                // 	const char *write_data = "Write test for LittleFS mounted system.\n";
                // 	// ssize_t bytes_written = write(fd, write_data, strlen(write_data));
                // 	ssize_t bytes_written = file_write(&file_p, write_data, strlen(write_data));
                // 	if(bytes_written > 0)
                // 	{
                // 		syslog(LOG_INFO, "Flash Write Successful.\n Data Len: %d\n", bytes_written);
                // 		// close(fd);
                // 		file_close(&file_p);
                // 	} else {
                // 		syslog(LOG_INFO, "Write Failure.\n");
                // 	}
                // 		// close(fd);
                // 		file_syncfs(&file_p);
                // 		file_close(&file_p);
                // }
            }

            total_blocks++;

            instances[i]->n_partitions_current++;
        }

    errout:

        if (rv < 0)
        {
            syslog(LOG_ERR, "mtd failure: %d bus %i class %d",
                   rv,
                   instances[i]->bus_id,
                   mtd_list->entries[num_entry]->partd[instances[i]->n_partitions_current].type);
            break;
        }
    }

    return rv;
}

// int cubus_mtd_unmount(const cubus_mtd_manifest_t *mft_mtd, char path[])
// {
//     // stm32_gpiowrite();
//     int rv = -EINVAL;

//     const cubus_mtd_manifest_t *mtd_list = mft_mtd;
//     rv = nx_umount2(path, 1);
//     if (rv >= 0)
//     {
//         printf("\n\n\n-------------------Unmounted %s successfully-------------------------\n", path);
//     }
//     char block[100] = {'\0'};
//     if (strcmp(path, "/mnt/fs/mfm/mtd_mainstorage") == 0)
//     {
//         strcpy(block, "/dev/mtdblock0");
//     }
//     else if (strcmp(path, "/mnt/fs/mfm/mtd_mission") == 0)
//     {
//         strcpy(block, "/dev/mtdblock1");
//     }
//     else if (strcmp(path, "/mnt/fs/sfm/mtd_mainstorage") == 0)
//     {
//         strcpy(block, "/dev/mtdblock2\0");
//     }
//     else if (strcmp(path, "/mnt/fs/sfm/mtd_mission") == 0)
//     {
//         strcpy(block, "/dev/mtdblock3\0");
//     }
//     int ret = unregister_mtddriver(block); // https://github.com/apache/nuttx/blob/master/include/nuttx/fs/fs.h
//     if (ret < 0)
//     {
//         printf("ERROR: Failed to unlink MTD device %s: %d\n", block, ret);
//         return ret;
//     }
//     else
//     {
//         printf("MTD device %s unlinked successfully\n", "/dev/block0");
//         if (strcmp(path, "/mnt/fs/mfm/mtd_mission") == 0)
//             num_instances--;
//     }
//     return rv;
// }
int cubus_mtd_unmount(const cubus_mtd_manifest_t *mft_mtd, char path[])
{
    if (mft_mtd == NULL || path == NULL)
    {
        syslog(LOG_ERR, "Invalid MTD configuration or path provided!\n");
        return -EINVAL;
    }

    // Attempt to unmount the given path
    int ret = nx_umount2(path, 0);
    if (ret < 0)
    {
        syslog(LOG_ERR, "Failed to unmount %s: %d\n", path, ret);
        return ret;
    }
    else
    {
        if (strcmp(path, "/mnt/fs/mfm/mtd_mission") == 0 || strcmp(path, "/mnt/fs/sfm/mtd_mission") == 0)
        {
            printf("Unmount path : %s successful\n",path);
            num_instances--;
        }
    }

    syslog(LOG_INFO, "Unmounted %s successfully\n", path);

    // Determine the block device corresponding to the path
    char block[32];
    if (strcmp(path, "/mnt/fs/mfm/mtd_mainstorage") == 0)
    {
        strcpy(block, "/dev/mtdblock0");
    }
    else if (strcmp(path, "/mnt/fs/mfm/mtd_mission") == 0)
    {
        strcpy(block, "/dev/mtdblock1");
    }
    else if (strcmp(path, "/mnt/fs/sfm/mtd_mainstorage") == 0)
    {
        strcpy(block, "/dev/mtdblock2");
    }
    else if (strcmp(path, "/mnt/fs/sfm/mtd_mission") == 0)
    {
        strcpy(block, "/dev/mtdblock3");
    }
    else
    {
        syslog(LOG_ERR, "Unknown path: %s\n", path);
        return -EINVAL;
    }

    // Unregister the MTD block device
    ret = unregister_mtddriver(block);
    if (ret < 0)
    {
        syslog(LOG_ERR, "Failed to unregister MTD device %s: %d\n", block, ret);
        return ret;
    }

    syslog(LOG_INFO, "MTD device %s unregistered successfully\n", block);

    // Decrement instance count for mission storage unmounts

    return 0;
}

int cubus_mtd_query(const char *sub, const char *val, const char **get)
{
    int rv = -ENODEV;

    static const char *keys[] = CUBUS_MFT_MTD_STR_TYPES;
    static const cubus_mtd_types_t types[] = CUBUS_MFT_MTD_TYPES;
    int key = 0;
    int key_len = (sizeof(keys) / sizeof(keys[0]));
    for (unsigned int k = 0; k < key_len; k++)
    {
        if (!strcmp(keys[k], sub))
        {
            key = types[k];
            break;
        }
    }

    rv = -EINVAL;

    if (key != 0)
    {
        rv = -ENOENT;

        for (int i = 0; i < num_instances; i++)
        {
            for (unsigned n = 0; n < instances[i]->n_partitions_current; n++)
            {
                if (instances[i]->partition_types[n] == key)
                {
                    if (get != NULL && val == NULL)
                    {
                        *get = instances[i]->partition_names[n];
                        return 0;
                    }

                    if (val != NULL && strcmp(instances[i]->partition_names[n], val) == 0)
                    {
                        return 0;
                    }
                }
            }
        }
    }

    return rv;
}

int cubus_mft_configure(const cubus_mft_s *mft_p, int block_number)
{

    if (mft_p != NULL)
    {
        for (uint32_t m = 0; m < mft_p->nmft; m++)
        {
            switch (mft_p->mfts[m]->type)
            {
            case MTD:
                cubus_mtd_config((const cubus_mtd_manifest_t *)mft_p->mfts[m]->pmft, block_number);
                break;

            case MFT:
            default:
                break;
            }
        }
    }

    return 0;
}

int cubus_unmount(const cubus_mtd_manifest_t *mft_mtd, char path[])
{
    if (mft_mtd == NULL)
    {
        syslog(LOG_ERR, "Invalid MTD configuration provided!\n");
        return -EINVAL;
    }

    if (mft_mtd->nconfigs == 0)
    {
        syslog(LOG_INFO, "No MTD configurations to unmount.\n");
        return 0;
    }

    for (uint8_t config_idx = 0; config_idx < mft_mtd->nconfigs; ++config_idx)
    {
        const cubus_mtd_entry_t *entry = mft_mtd->entries[config_idx];
        if (entry == NULL)
        {
            syslog(LOG_ERR, "Invalid MTD entry at index %d. Skipping...\n", config_idx);
            continue;
        }

        for (uint32_t part_idx = 0; part_idx < entry->npart; ++part_idx)
        {
            const char *mount_point = entry->partd[part_idx].path;
            if (mount_point == NULL || strlen(mount_point) == 0)
            {
                syslog(LOG_ERR, "Invalid mount point for config %d, partition %d. Skipping...\n", config_idx, part_idx);
                continue;
            }

            syslog(LOG_INFO, "Attempting to unmount partition: %s\n", mount_point);

            // Check if the mount point is active
            struct statfs stat;
            if (statfs(mount_point, &stat) < 0)
            {
                syslog(LOG_ERR, "Mount point %s is not an active filesystem or is inaccessible.\n", mount_point);
                continue;
            }

            // Use the correct mount point from the configuration
            int ret = nx_umount2("/mnt/fs/mfm/mtd_mainstorage/", 0);
            if (ret < 0)
            {
                syslog(LOG_ERR, "Failed to unmount %s. Error: %d\n", mount_point, ret);
                return ret; // Return early on failure
            }
            else
            {
                syslog(LOG_INFO, "Successfully unmounted %s\n", mount_point);
            }
            ret = nx_umount2("/mnt/fs/mfm/mtd_mission/", 0);
            if (ret < 0)
            {
                syslog(LOG_ERR, "Failed to unmount %s. Error: %d\n", mount_point, ret);
                return ret; // Return early on failure
            }
            else
            {
                syslog(LOG_INFO, "Successfully unmounted %s\n", mount_point);
            }

            ret = mtd_unlink("/dev/mtdblock0"); // mtd_unlink() is hypothetical, depending on your NuttX version
            if (ret < 0)
            {
                printf("ERROR: Failed to unlink MTD device %s: %d\n", "/dev/block0", ret);
                return ret;
            }
            else
            {
                printf("MTD device %s unlinked successfully\n", "/dev/block0");
                num_instances--;
            }
            ret = mtd_unlink("/dev/mtdblock1"); // mtd_unlink() is hypothetical, depending on your NuttX version
            if (ret < 0)
            {
                printf("ERROR: Failed to unlink MTD device %s: %d\n", "/dev/block1", ret);
                return ret;
            }
            else
            {
                num_instances--;
                printf("MTD device %s unlinked successfully\n", "/dev/block1");
            }
        }
    }

    syslog(LOG_INFO, "All MTD partitions unmounted successfully.\n");
    return 0;
}
