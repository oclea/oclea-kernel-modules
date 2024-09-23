#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/nvmem-provider.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/list.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <linux/err.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/namei.h>
#include <linux/stat.h>

#define DEFAULT_BIN_DIR "/mnt/settings/"

// Module parameter for base bin path
static char *bin_dir = DEFAULT_BIN_DIR;
module_param(bin_dir, charp, 0644);
MODULE_PARM_DESC(bin_dir, "Base path for EEPROM bin files");

// Structure to hold EEPROM metadata
struct virtual_eeprom {
    struct nvmem_device *nvmem_dev;
    struct nvmem_config nvmem_cfg;
    struct mutex mutex;
    int size;
    const char *name;
    char *bin_path;
    struct list_head list;
    struct platform_device *pdev;
};

// Initialize global list to track all EEPROMs
static LIST_HEAD(eeprom_list);

// Function prototypes
static int eeprom_read(void *context, unsigned int offset, void *val, size_t bytes);
static int eeprom_write(void *context, unsigned int offset, void *val, size_t bytes);
static int init_storage_file(struct virtual_eeprom *eeprom);

static int eeprom_read(void *context, unsigned int offset, void *val, size_t bytes)
{
    struct virtual_eeprom *eeprom = context;
    struct file *file;
    loff_t pos = offset;
    ssize_t ret, total_read = 0;

    if (offset >= eeprom->size)
        return -EINVAL;

    if (offset + bytes > eeprom->size) {
        pr_err("Failed to read %zu bytes. Exceeds EEPROM size %d\n", bytes, eeprom->size);
        return -EINVAL;
    }

    file = filp_open(eeprom->bin_path, O_RDONLY, 0644);
    if (IS_ERR(file)) {
        pr_err("Failed to open file %s for reading: %ld\n", eeprom->bin_path, PTR_ERR(file));
        return PTR_ERR(file);
    }

    // Try to read requested bytes iteratively
    do {
        ret = kernel_read(file, val + total_read, bytes - total_read, &pos);
        if (ret < 0) {
            pr_err("Failed to read data from file %s: %zd\n", eeprom->bin_path, ret);
            break;
        }
        total_read += ret;
    } while (total_read < bytes && ret > 0);

    filp_close(file, NULL);

    return ret < 0 ? ret : 0;
}

static int eeprom_write(void *context, unsigned int offset, void *val, size_t bytes)
{
    struct virtual_eeprom *eeprom = context;
    struct file *file;
    loff_t pos = offset;
    ssize_t ret, total_written = 0;

    if (offset >= eeprom->size)
        return -EINVAL;

    if (offset + bytes > eeprom->size) {
        pr_err("Failed to write %zu bytes. Exceeds EEPROM size %d\n", bytes, eeprom->size);
        return -EINVAL;
    }

    file = filp_open(eeprom->bin_path, O_WRONLY | O_CREAT, 0644);
    if (IS_ERR(file)) {
        pr_err("Failed to open file %s for writing: %ld\n", eeprom->bin_path, PTR_ERR(file));
        return PTR_ERR(file);
    }

    // Try to write requested bytes iteratively
    do {
        ret = kernel_write(file, val + total_written, bytes - total_written, &pos);
        if (ret < 0) {
            pr_err("Failed to write data to file %s: %zd\n", eeprom->bin_path, ret);
            break;
        }
        total_written += ret;
    } while (total_written < bytes && ret > 0);

    filp_close(file, NULL);

    return ret < 0 ? ret : 0;
}

// Function to init the storage file
static int init_storage_file(struct virtual_eeprom *eeprom)
{
    loff_t file_size;
    struct path file_path;
    struct kstat stat;
    char *zero_buffer;
    int ret;

    // Check if bin file already exists
    ret = kern_path(eeprom->bin_path, LOOKUP_FOLLOW, &file_path);
    if (ret == -ENOENT) {
        // File doesn't exist, create and initialise the new bin file with all zeros
        zero_buffer = kzalloc(eeprom->size, GFP_KERNEL);
        if (!zero_buffer) {
            pr_err("Failed to allocate memory for zero buffer\n");
            return -ENOMEM;
        }

        ret = eeprom_write(eeprom, 0, zero_buffer, eeprom->size);
        kfree(zero_buffer);

    } else if (0 != ret) {
        // File exists but can't access
        pr_err("Error accessing file %s: %d\n", eeprom->bin_path, ret);
        return ret;
    } else {
        // Can access the file, check size
        ret = vfs_getattr(&file_path, &stat, STATX_SIZE, AT_STATX_SYNC_AS_STAT);
        path_put(&file_path);
        if (ret) {
            pr_err("Failed to get file size for %s: %d\n", eeprom->bin_path, ret);
            return ret;
        }

        file_size = stat.size;

        // Check if file size matches the expected EEPROM size
        if (file_size != eeprom->size) {
            pr_err("Size mismatch for %s: DTS size is %d, but file size is %lld\n",
                   eeprom->bin_path, eeprom->size, file_size);
            return -EINVAL;
        }
    }

    return 0;
}

static int __init eeprom_init(void)
{
    struct device_node *np, *child;
    int ret;

    // Find the parent device node
    np = of_find_node_by_name(NULL, "virtual_eeproms");
    if (!np) {
        pr_err("No device node found for virtual_eeproms\n");
        return -ENODEV;
    }

    // Iterate through all child nodes
    for_each_child_of_node(np, child) {
        const char *name = child->name;
        u32 size;
        struct virtual_eeprom *eeprom;

        if (of_property_read_u32(child, "size", &size)) {
            pr_err("Failed to read size for %s\n", name);
            continue;
        }

        eeprom = kzalloc(sizeof(struct virtual_eeprom), GFP_KERNEL);
        if (!eeprom) {
            pr_err("Failed to allocate memory for %s\n", name);
            continue;
        }

        // Construct bin_path from base directory and EEPROM name
        eeprom->bin_path = kasprintf(GFP_KERNEL, "%s/%s.bin", bin_dir, name);
        if (!eeprom->bin_path) {
            pr_err("Failed to allocate memory for bin_path\n");
            kfree(eeprom);
            continue;
        }

        mutex_init(&eeprom->mutex);
        eeprom->size = size;
        eeprom->name = name;

        // Init storage file (create if necessary) and validate size configured in DTS
        ret = init_storage_file(eeprom);
        if (ret) {
            pr_err("Skipping registration of %s EEPROM\n", name);
            kfree(eeprom->bin_path);
            kfree(eeprom);
            continue;
        }

        // Create unique platform device name by appending EEPROM name
        eeprom->pdev = platform_device_register_simple(name, -1, NULL, 0);
        if (IS_ERR(eeprom->pdev)) {
            pr_err("Failed to register platform device for %s\n", name);
            kfree(eeprom->bin_path);
            kfree(eeprom);
            continue;
        }

        // Configure nvmem
        eeprom->nvmem_cfg.name = name;
        eeprom->nvmem_cfg.type = NVMEM_TYPE_EEPROM;
        eeprom->nvmem_cfg.read_only = false;
        eeprom->nvmem_cfg.root_only = false;
        eeprom->nvmem_cfg.size = size;
        eeprom->nvmem_cfg.reg_read = eeprom_read;
        eeprom->nvmem_cfg.reg_write = eeprom_write;
        eeprom->nvmem_cfg.owner = THIS_MODULE;
        eeprom->nvmem_cfg.priv = eeprom;
        eeprom->nvmem_cfg.dev = &eeprom->pdev->dev;

        eeprom->nvmem_dev = nvmem_register(&eeprom->nvmem_cfg);
        if (IS_ERR(eeprom->nvmem_dev)) {
            pr_err("Failed to register nvmem device for %s: %ld\n", name, PTR_ERR(eeprom->nvmem_dev));
            platform_device_unregister(eeprom->pdev);
            kfree(eeprom->bin_path);
            kfree(eeprom);
            continue;
        }

        list_add(&eeprom->list, &eeprom_list);

        pr_info("Virtual EEPROM %s loaded with nvmem\n", name);
    }

    return 0;
}

static void __exit eeprom_exit(void)
{
    struct virtual_eeprom *eeprom, *tmp;

    list_for_each_entry_safe(eeprom, tmp, &eeprom_list, list) {
        nvmem_unregister(eeprom->nvmem_dev);
        platform_device_unregister(eeprom->pdev);
        kfree(eeprom->bin_path);
        kfree(eeprom);
        pr_info("Virtual EEPROM %s unloaded\n", eeprom->name);
    }
}

module_init(eeprom_init);
module_exit(eeprom_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Oclea <support@oclea.com>");
MODULE_DESCRIPTION("Virtual EEPROM Driver");
MODULE_ALIAS("platform:virtual-eeprom-driver");
