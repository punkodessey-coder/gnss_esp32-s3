# Patch Sqlite3Esp32's VFS to use Newlib's explicit large-file API.
# ESP32's ordinary off_t is 32-bit; _FILE_OFFSET_BITS=64 is not honored by
# this Arduino toolchain. Newlib exposes fopen64/fseeko64/fstat64 instead.
Import("env")
import os

libdeps = env.subst("$PROJECT_LIBDEPS_DIR")
pioenv = env.subst("$PIOENV")
candidates = [
    os.path.join(libdeps, pioenv, "Sqlite3Esp32", "src", "esp32.cpp"),
    os.path.join(libdeps, pioenv, "Sqlite3Esp32", "esp32.cpp"),
]
target = next((p for p in candidates if os.path.isfile(p)), None)
if not target:
    raise RuntimeError("Sqlite3Esp32 esp32.cpp not found; cannot apply large-file VFS patch")

with open(target, "r", encoding="utf-8") as f:
    source = f.read()

if "#define _LARGEFILE64_SOURCE" not in source:
    source = "#define _LARGEFILE64_SOURCE 1\n" + source

# Keep the operations consistent: large-file streams must be opened with fopen64.
source = source.replace("fopen(", "fopen64(")
# stat() is also used while opening a database; use the matching 64-bit structure/API.
source = source.replace("struct stat st;", "struct stat64 st;")
source = source.replace("stat( zName, &st )", "stat64( zName, &st )")
source = source.replace("fseek(p->fp, iOfst, SEEK_SET)", "fseeko64(p->fp, (_off64_t)iOfst, SEEK_SET)")
source = source.replace("struct stat st;", "struct stat64 st;")
source = source.replace("fstat(fno, &st)", "fstat64(fno, &st)")

if "fseeko64(p->fp, (_off64_t)iOfst, SEEK_SET)" not in source:
    raise RuntimeError("Large-file seek patch was not applied")
if "fopen64(" not in source or "fstat64(fno, &st)" not in source:
    raise RuntimeError("Large-file open/stat patch was not applied")
if "static_assert(sizeof(_off64_t) >= 8" not in source:
    marker = '#include <sys/types.h>'
    guard = '\n#ifndef __LARGE64_FILES\n#error "Newlib large-file API is unavailable in this ESP32 toolchain"\n#endif\nstatic_assert(sizeof(_off64_t) >= 8, "Newlib _off64_t must be 64-bit for MBTiles");\n'
    if marker not in source:
        raise RuntimeError("Cannot locate sys/types.h in Sqlite3Esp32 VFS")
    source = source.replace(marker, marker + guard, 1)

with open(target, "w", encoding="utf-8") as f:
    f.write(source)

print("Patched Sqlite3Esp32 VFS to use fopen64/fseeko64/fstat64:", target)
