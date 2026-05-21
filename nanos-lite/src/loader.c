#include <proc.h>
#include <elf.h>

#ifdef __LP64__
# define Elf_Ehdr Elf64_Ehdr
# define Elf_Phdr Elf64_Phdr
#else
# define Elf_Ehdr Elf32_Ehdr
# define Elf_Phdr Elf32_Phdr
#endif

// Match the AM target the ramdisk was built for. PA3 only runs one ISA at
// a time, so this is a sanity check, not portability.
#if defined(__ISA_AM_NATIVE__)
# define EXPECT_TYPE EM_X86_64
#elif defined(__ISA_X86__)
# define EXPECT_TYPE EM_386
#elif defined(__ISA_MIPS32__)
# define EXPECT_TYPE EM_MIPS
#elif defined(__riscv)
# define EXPECT_TYPE EM_RISCV
#else
# error Unsupported ISA
#endif

size_t ramdisk_read(void *buf, size_t offset, size_t len);
size_t get_ramdisk_size();

static uintptr_t loader(PCB *pcb, const char *filename) {
  Elf_Ehdr ehdr;
  ramdisk_read(&ehdr, 0, sizeof(ehdr));

  // ELF magic guards against loading a non-ELF blob (e.g. raw .bin).
  assert(*(uint32_t *)ehdr.e_ident == 0x464c457f);
  // Catches the "loaded the wrong ISA's image" foot-gun.
  assert(ehdr.e_machine == EXPECT_TYPE);

  Elf_Phdr phdr[ehdr.e_phnum];
  ramdisk_read(phdr, ehdr.e_phoff, sizeof(phdr));

  for (int i = 0; i < ehdr.e_phnum; i++) {
    if (phdr[i].p_type != PT_LOAD) continue;
    // Copy file image, then zero-fill the .bss tail [FileSiz, MemSiz).
    ramdisk_read((void *)(uintptr_t)phdr[i].p_vaddr,
                 phdr[i].p_offset, phdr[i].p_filesz);
    memset((void *)(uintptr_t)(phdr[i].p_vaddr + phdr[i].p_filesz), 0,
           phdr[i].p_memsz - phdr[i].p_filesz);
  }

  return (uintptr_t)ehdr.e_entry;
}

void naive_uload(PCB *pcb, const char *filename) {
  uintptr_t entry = loader(pcb, filename);
  Log("Jump to entry = %p", (void *)entry);
  ((void(*)())entry) ();
}

