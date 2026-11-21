# PlatformIO pre-build patch for Sqlite3Esp32's ESP32 VFS.
# The upstream VFS uses fseek(long), which truncates SQLite's 64-bit file offsets
# on 32-bit ESP32 builds. Build with _FILE_OFFSET_BITS=64 and use fseeko(off_t).
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
    raise RuntimeError("Sqlite3Esp32 esp32.cpp not found; cannot apply 64-bit MBTiles seek patch")

with open(target, "r", encoding="utf-8") as f:
    source = f.read()

old = "fseek(p->fp, iOfst, SEEK_SET)"
new = "fseeko(p->fp, (off_t)iOfst, SEEK_SET)"
count = source.count(old)
if count == 0:
    if new not in source:
        raise RuntimeError("Sqlite3Esp32 VFS changed; expected fseek calls not found")
else:
    source = source.replace(old, new)
    with open(target, "w", encoding="utf-8") as f:
        f.write(source)

# Compile-time guard: never silently build a 32-bit-offset SQLite VFS.
guard = "\\nstatic_assert(sizeof(off_t) >= 8, \"MBTiles requires 64-bit off_t; check _FILE_OFFSET_BITS=64\");\\n"
with open(target, "r", encoding="utf-8") as f:
    source = f.read()
if "MBTiles requires 64-bit off_t" not in source:
    marker = '#include <sys/types.h>'
    if marker not in source:
        raise RuntimeError("Cannot locate sys/types.h in Sqlite3Esp32 VFS")
    source = source.replace(marker, marker + guard, 1)
    with open(target, "w", encoding="utf-8") as f:
        f.write(source)

print("Patched Sqlite3Esp32 VFS for checked 64-bit file offsets:", target)
