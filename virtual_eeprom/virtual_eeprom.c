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

// Structure to hold EEPROM data and metadata
struct virtual_eeprom {
    char *data;
    struct nvmem_device *nvmem_dev;
    struct nvmem_config nvmem_cfg;
    struct mutex mutex;
    int size;
    const char *name;
    const char *bin_path;
    struct list_head list;
    struct platform_device *pdev;
};

// Function prototypes
static int save_eeprom_to_file(struct virtual_eeprom *eeprom);
static int load_eeprom_from_file(struct virtual_eeprom *eeprom);

// Initialize global list to track all EEPROMs
static LIST_HEAD(eeprom_list);

static int eeprom_read(void *context, unsigned int offset, void *val, size_t bytes)
{
    struct virtual_eeprom *eeprom = context;

    if (offset >= eeprom->size)
        return -EINVAL;

    if (offset + bytes > eeprom->size)
        bytes = eeprom->size - offset;

    mutex_lock(&eeprom->mutex);
    memcpy(val, eeprom->data + offset, bytes);
    mutex_unlock(&eeprom->mutex);

    return 0;
}

static int eeprom_write(void *context, unsigned int offset, void *val, size_t bytes)
{
    struct virtual_eeprom *eeprom = context;
    int ret;

    if (offset >= eeprom->size)
        return -EINVAL;

    if (offset + bytes > eeprom->size)
        bytes = eeprom->size - offset;

    mutex_lock(&eeprom->mutex);
    memcpy(eeprom->data + offset, val, bytes);
    // Save the data to the file after each write
    ret = save_eeprom_to_file(eeprom);
    mutex_unlock(&eeprom->mutex);

    return ret < 0 ? ret : 0;
}

static int save_eeprom_to_file(struct virtual_eeprom *eeprom)
{
    struct file *file;
    loff_t pos = 0;
    ssize_t ret;

    file = filp_open(eeprom->bin_path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (IS_ERR(file)) {
        pr_err("Failed to open file %s for writing: %ld\n", eeprom->bin_path, PTR_ERR(file));
        return PTR_ERR(file);
    }

    ret = kernel_write(file, eeprom->data, eeprom->size, &pos);
    filp_close(file, NULL);

    return ret < 0 ? ret : 0;
}

static int load_eeprom_from_file(struct virtual_eeprom *eeprom)
{
    struct file *file;
    loff_t pos = 0;
    ssize_t ret;

    file = filp_open(eeprom->bin_path, O_RDONLY, 0644);
    if (IS_ERR(file)) {
	    // don't print read error if the file doesn't exist
        if (PTR_ERR(file) != -ENOENT)
            pr_err("Failed to open file %s for reading: %ld\n", eeprom->bin_path, PTR_ERR(file));
        return PTR_ERR(file);
    }

    ret = kernel_read(file, eeprom->data, eeprom->size, &pos);
    filp_close(file, NULL);

    return ret < 0 ? ret : 0;
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
        const char *bin_path;
        u32 size;

        if (of_property_read_u32(child, "size", &size)) {
            pr_err("Failed to read size for %s\n", name);
            continue;
        }

        if (of_property_read_string(child, "bin-path", &bin_path)) {
            pr_err("Failed to read bin-path for %s\n", name);
            continue;
        }

        struct virtual_eeprom *eeprom = kzalloc(sizeof(struct virtual_eeprom), GFP_KERNEL);
        if (!eeprom) {
            pr_err("Failed to allocate memory for %s\n", name);
            continue;
        }

        eeprom->data = kzalloc(size, GFP_KERNEL);
        if (!eeprom->data) {
            pr_err("Failed to allocate memory for EEPROM %s\n", name);
            kfree(eeprom);
            continue;
        }

        mutex_init(&eeprom->mutex);
        eeprom->size = size;
        eeprom->name = name;
        eeprom->bin_path = bin_path;

        // Create unique platform device name by appending EEPROM name
        eeprom->pdev = platform_device_register_simple(name, -1, NULL, 0);
        if (IS_ERR(eeprom->pdev)) {
            pr_err("Failed to register platform device for %s\n", name);
            kfree(eeprom->data);
            kfree(eeprom);
            continue;
        }

        ret = load_eeprom_from_file(eeprom);
        if (ret < 0) {
            // Initialize with 0 if file load fails
            memset(eeprom->data, 0, size);
        }

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
            kfree(eeprom->data);
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

    // Iterate through all created EEPROMs and clean them up
    list_for_each_entry_safe(eeprom, tmp, &eeprom_list, list) {
        save_eeprom_to_file(eeprom);
        nvmem_unregister(eeprom->nvmem_dev);
        platform_device_unregister(eeprom->pdev);
        kfree(eeprom->data);
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
