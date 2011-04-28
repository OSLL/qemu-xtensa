#include "sysemu.h"
#include "boards.h"
#include "loader.h"
#include "elf.h"

static void xtensa_init(ram_addr_t ram_size,
        const char *boot_device,
        const char *kernel_filename, const char *kernel_cmdline,
        const char *initrd_filename, const char *cpu_model)
{
    CPUState *env = NULL;
    ram_addr_t ram_offset;
    int n;

    for (n = 0; n < smp_cpus; n++) {
        env = cpu_init(cpu_model);
        if (!env) {
            fprintf(stderr, "Unable to find CPU definition\n");
            exit(1);
        }
        env->sregs[PRID] = n;
    }

    ram_offset = qemu_ram_alloc(NULL, "xtensa.dram", 0x10000);
    cpu_register_physical_memory(0x5ffd0000, 0x10000, ram_offset);

    ram_offset = qemu_ram_alloc(NULL, "xtensa.iram", 0x20000);
    cpu_register_physical_memory(0x5ffe0000, 0x20000, ram_offset);

    ram_offset = qemu_ram_alloc(NULL, "xtensa.sram", ram_size);
    cpu_register_physical_memory(0x60000000, ram_size, ram_offset);

    if (kernel_filename) {
        uint64_t elf_entry;
        uint64_t elf_lowaddr;
#ifdef TARGET_WORDS_BIGENDIAN
        int success = load_elf(kernel_filename, NULL, NULL, &elf_entry,
                &elf_lowaddr, NULL, 1, ELF_MACHINE, 0);
#else
        int success = load_elf(kernel_filename, NULL, NULL, &elf_entry,
                &elf_lowaddr, NULL, 0, ELF_MACHINE, 0);
#endif
        if (success > 0) {
            env->pc = elf_entry;
        }
    }
}

static void xtensa_sample_init(ram_addr_t ram_size,
                     const char *boot_device,
                     const char *kernel_filename, const char *kernel_cmdline,
                     const char *initrd_filename, const char *cpu_model)
{
    if (!cpu_model) {
        cpu_model = "sample-xtensa-core";
    }
    xtensa_init(ram_size, boot_device, kernel_filename, kernel_cmdline,
                  initrd_filename, cpu_model);
}

static QEMUMachine xtensa_sample_machine = {
    .name = "sample-xtensa-machine",
    .desc = "Sample Xtensa machine (sample Xtensa core)",
    .init = xtensa_sample_init,
    .max_cpus = 4,
};

static void xtensa_sample_machine_init(void)
{
    qemu_register_machine(&xtensa_sample_machine);
}

machine_init(xtensa_sample_machine_init);
