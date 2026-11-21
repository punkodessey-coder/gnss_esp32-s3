# PlatformIO pre-build patch for Sqlite3Esp32's ESP32 VFS.
# SQLite passes 64-bit offsets; upstream used fseek(long), which truncates offsets
# on 32-bit ESP32. _FILE_OFFSET_BITS=64 and fseeko(off_t) preserve full offsets.
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
if old in source:
    source = source.replace(old, new)
elif new not in source:
    raise RuntimeError("Sqlite3Esp32 VFS changed; expected seek call not found")

guard = chr(10) + 'static_assert(sizeof(off_t) >= 8, "MBTiles requires 64-bit off_t; check _FILE_OFFSET_BITS=64");' + chr(10)
if "MBTiles requires 64-bit off_t" not in source:
    marker = "#include <sys/types.h>"
    if marker not in source:
        raise RuntimeError("Cannot locate sys/types.h in Sqlite3Esp32 VFS")
    source = source.replace(marker, marker + guard, 1)

with open(target, "w", encoding="utf-8") as f:
    f.write(source)

if "fseek(p->fp, iOfst, SEEK_SET)" in source:
    raise RuntimeError("32-bit fseek call remains in Sqlite3Esp32 VFS")
print("Patched Sqlite3Esp32 VFS for 64-bit file offsets:", target)
