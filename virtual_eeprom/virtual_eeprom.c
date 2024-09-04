#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/list.h>

// Structure to hold EEPROM data and metadata
struct virtual_eeprom {
    char *data;
    struct proc_dir_entry *proc_entry;
    struct mutex mutex;
    int size;
    const char *name;
    // head for linked list management
    struct list_head list;
};

// Function prototypes
static int save_eeprom_to_file(struct virtual_eeprom *eeprom);
static int load_eeprom_from_file(struct virtual_eeprom *eeprom);

// Directory for the virtual EEPROMs
static struct proc_dir_entry *eeprom_dir;
// Initialize global list to track all EEPROMs
static LIST_HEAD(eeprom_list);

static ssize_t eeprom_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    struct virtual_eeprom *eeprom = PDE_DATA(file_inode(file));
    ssize_t ret;

    if (*ppos >= eeprom->size)
        return 0;

    if (*ppos + count > eeprom->size)
        count = eeprom->size - *ppos;

    mutex_lock(&eeprom->mutex);
    ret = copy_to_user(buf, eeprom->data + *ppos, count);
    mutex_unlock(&eeprom->mutex);

    if (ret == 0) {
        *ppos += count;
        return count;
    } else {
        return -EFAULT;
    }
}

static ssize_t eeprom_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    struct virtual_eeprom *eeprom = PDE_DATA(file_inode(file));
    int ret;

    if (*ppos >= eeprom->size)
        return -EFBIG;

    if (*ppos + count > eeprom->size)
        count = eeprom->size - *ppos;

    mutex_lock(&eeprom->mutex);
    if (copy_from_user(eeprom->data + *ppos, buf, count)) {
        mutex_unlock(&eeprom->mutex);
        return -EFAULT;
    }

    *ppos += count;
    // Save the data to the file after each write
    ret = save_eeprom_to_file(eeprom);
    mutex_unlock(&eeprom->mutex);

    return ret < 0 ? ret : count;
}

static const struct file_operations eeprom_fops = {
    .owner = THIS_MODULE,
    .read = eeprom_read,
    .write = eeprom_write,
};

static int save_eeprom_to_file(struct virtual_eeprom *eeprom)
{
    struct file *file;
    mm_segment_t old_fs;
    ssize_t ret;
    char file_path[128];

    snprintf(file_path, sizeof(file_path), "/mnt/settings/%s.bin", eeprom->name);

    old_fs = get_fs();
    set_fs(KERNEL_DS);
    file = filp_open(file_path, O_WRONLY | O_CREAT, 0644);
    if (IS_ERR(file)) {
        set_fs(old_fs);
        return PTR_ERR(file);
    }

    ret = kernel_write(file, eeprom->data, eeprom->size, &file->f_pos);
    filp_close(file, NULL);
    set_fs(old_fs);

    return ret < 0 ? ret : 0;
}

static int load_eeprom_from_file(struct virtual_eeprom *eeprom)
{
    struct file *file;
    mm_segment_t old_fs;
    ssize_t ret;
    char file_path[128];

    snprintf(file_path, sizeof(file_path), "/mnt/settings/%s.bin", eeprom->name);

    old_fs = get_fs();
    set_fs(KERNEL_DS);
    file = filp_open(file_path, O_RDONLY, 0644);
    if (IS_ERR(file)) {
        set_fs(old_fs);
        return PTR_ERR(file);
    }

    ret = kernel_read(file, eeprom->data, eeprom->size, &file->f_pos);
    filp_close(file, NULL);
    set_fs(old_fs);

    return ret < 0 ? ret : 0;
}

static int __init eeprom_init(void)
{
    struct device_node *np, *child;
    int ret;
    char proc_name[32];

    // Create the /proc/virtual_eeprom directory
    eeprom_dir = proc_mkdir("virtual_eeprom", NULL);
    if (!eeprom_dir) {
        pr_err("Failed to create /proc/virtual_eeprom directory\n");
        return -ENOMEM;
    }

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

        if (of_property_read_u32(child, "size", &size)) {
            pr_err("Failed to read size for %s\n", name);
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

        ret = load_eeprom_from_file(eeprom);
        if (ret < 0) {
            memset(eeprom->data, 0, size);
        }

        snprintf(proc_name, sizeof(proc_name), "%s", name);
        eeprom->proc_entry = proc_create_data(proc_name, 0666, eeprom_dir, &eeprom_fops, eeprom);
        if (!eeprom->proc_entry) {
            kfree(eeprom->data);
            kfree(eeprom);
            pr_err("Failed to create proc entry for %s\n", name);
            continue;
        }

        // Add to the global list
        list_add(&eeprom->list, &eeprom_list);

        pr_info("Virtual EEPROM /proc/virtual_eeprom/%s loaded\n", name);
    }

    return 0;
}

static void __exit eeprom_exit(void)
{
    struct virtual_eeprom *eeprom, *tmp;

    // Iterate through all created EEPROMs and clean them up
    list_for_each_entry_safe(eeprom, tmp, &eeprom_list, list) {
        save_eeprom_to_file(eeprom);
        proc_remove(eeprom->proc_entry);
        kfree(eeprom->data);
        kfree(eeprom);
        pr_info("Virtual EEPROM /proc/virtual_eeprom/%s unloaded\n", eeprom->name);
    }

    // Remove the /proc/virtual_eeprom directory
    proc_remove(eeprom_dir);
}

module_init(eeprom_init);
module_exit(eeprom_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Oclea <support@oclea.com>");
MODULE_DESCRIPTION("Virtual EEPROM Driver");
MODULE_ALIAS("platform:virtual-eeprom-driver");
