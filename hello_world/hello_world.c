/* SPDX-License-Identifier: MIT */
#include <linux/kernel.h>
#include <linux/module.h>

int init_hello_world(void)
{
    printk(KERN_INFO "Hello World!\n");
    return 0;
}

void cleanup_hello_world(void)
{
    printk(KERN_INFO "Cleanup\n");
}

module_init(init_hello_world)
module_exit(cleanup_hello_world)

MODULE_AUTHOR("Oclea <support@oclea.com>");
MODULE_DESCRIPTION("Hello World example");
MODULE_LICENSE("Dual MIT/GPL");
