# Patch Sqlite3Esp32 VFS to use FatFs directly for main MBTiles reads.
# Arduino File::seek(uint32_t) still calls fseek(long), so it cannot reliably
# address offsets >= 2 GiB on ESP32. FatFs FSIZE_t is unsigned 32-bit on FAT32.
Import("env")
import os
import re
from datetime import datetime, timezone

# Embed the exact Git revision through a generated header. Passing a quoted
# string via CPPDEFINES is not portable across SCons/PlatformIO toolchains.
git_sha = os.environ.get('GITHUB_SHA', 'local')
build_utc = datetime.now(timezone.utc)
build_date = build_utc.strftime('%Y-%m-%d')
build_time = build_utc.strftime('%H:%M:%S')
build_timestamp = build_utc.strftime('%Y-%m-%dT%H:%M:%SZ')
project_dir = env.subst("$PROJECT_DIR")
include_dir = os.path.join(project_dir, "include")
os.makedirs(include_dir, exist_ok=True)
header_path = os.path.join(include_dir, "firmware_build_info.h")
with open(header_path, "w", encoding="utf-8") as header:
    header.write('#pragma once\n#define FIRMWARE_GIT_SHA "' + git_sha + '"\n#define FIRMWARE_VERSION "' + os.environ.get('FIRMWARE_VERSION', '0.0.0-local') + '"\n#define FIRMWARE_BUILD_DATE "' + build_date + '"\n#define FIRMWARE_BUILD_TIME "' + build_time + ' UTC"\n#define FIRMWARE_BUILD_TIMESTAMP "' + build_timestamp + '"\n')

libdeps = env.subst("$PROJECT_LIBDEPS_DIR")
pioenv = env.subst("$PIOENV")
candidates = [
    os.path.join(libdeps, pioenv, "Sqlite3Esp32", "src", "esp32.cpp"),
    os.path.join(libdeps, pioenv, "Sqlite3Esp32", "esp32.cpp"),
]
target = next((p for p in candidates if os.path.isfile(p)), None)
if not target:
    raise RuntimeError("Sqlite3Esp32 esp32.cpp not found")

with open(target, "r", encoding="utf-8") as f:
    source = f.read()

if '#include "ff.h"' not in source and '#include <ff.h>' not in source:
    if "#include <Arduino.h>" not in source:
        raise RuntimeError("Arduino.h include not found")
    source = source.replace("#include <Arduino.h>", '#include <Arduino.h>\n#include "ff.h"', 1)

field = "  FIL *fatFile;                    /* direct FatFs handle for >2GiB MBTiles */"
if "FIL *fatFile;" not in source:
    source, n = re.subn(r"(  FILE \*fp;\s*/\* File descriptor \*/)", r"\1\n" + field, source, count=1)
    if n != 1:
        raise RuntimeError("Could not add FatFs handle to ESP32File")

close_marker = "  fclose(p->fp);"
close_code = "  if (p->fatFile) { f_close(p->fatFile); delete p->fatFile; p->fatFile = NULL; }\n" + close_marker
if "f_close(p->fatFile)" not in source:
    if close_marker not in source:
        raise RuntimeError("Could not locate ESP32Close")
    source = source.replace(close_marker, close_code, 1)

read_pattern = r"(?m)^[ \t]*ofst\s*=\s*fseek\(p->fp,\s*iOfst,\s*SEEK_SET\);[^\n]*\n(?:[ \t]*//[^\n]*\n)*[ \t]*nRead\s*=\s*fread\(zBuf,\s*1,\s*iAmt,\s*p->fp\);[^\n]*"
read_code = """  if (p->fatFile) {
    if (iOfst < 0 || (uint64_t)iOfst > 0xFFFFFFFFULL)
      return SQLITE_IOERR_READ;
    FRESULT fr = f_lseek(p->fatFile, (FSIZE_t)iOfst);
    if (fr != FR_OK) return SQLITE_IOERR_READ;
    UINT bytesRead = 0;
    fr = f_read(p->fatFile, zBuf, (UINT)iAmt, &bytesRead);
    if (fr != FR_OK) return SQLITE_IOERR_READ;
    if (bytesRead < (UINT)iAmt) {
      memset((char *)zBuf + bytesRead, 0, iAmt - bytesRead);
      return SQLITE_IOERR_SHORT_READ;
    }
    return SQLITE_OK;
  }
  ofst = fseek(p->fp, (long)iOfst, SEEK_SET);
  if (ofst != 0) return SQLITE_IOERR_READ;
  nRead = fread(zBuf, 1, iAmt, p->fp);"""
if "f_lseek(p->fatFile" not in source:
    source, n = re.subn(read_pattern, read_code, source, count=1)
    if n != 1:
        raise RuntimeError("Could not locate stock ESP32Read fseek/fread pair")

if "f_size(p->fatFile)" not in source:
    fn = source.find("static int ESP32FileSize(")
    ptr = source.find("ESP32File *p = (ESP32File*)pFile;", fn)
    if fn < 0 or ptr < 0:
        raise RuntimeError("Could not locate ESP32FileSize")
    at = ptr + len("ESP32File *p = (ESP32File*)pFile;")
    source = source[:at] + """
  if (p->fatFile) {
    *pSize = (sqlite3_int64)f_size(p->fatFile);
    return SQLITE_OK;
  }""" + source[at:]

if "f_open(p->fatFile" not in source:
    open_pattern = r"(  if\s*\(\s*p->fp\s*==\s*NULL\s*\)\s*\{\s*.*?\n\s*\}\s*\n)(\s*p->aBuffer\s*=\s*aBuf;)"
    open_code = r"""\1  if (flags & SQLITE_OPEN_MAIN_DB) {
    const char *relPath = zName;
    if (strncmp(relPath, "/sdcard", 7) == 0) relPath += 7;
    char fatPath[MAXPATHNAME + 8];
    snprintf(fatPath, sizeof(fatPath), "0:%s", relPath);
    p->fatFile = new FIL();
    if (!p->fatFile || f_open(p->fatFile, fatPath, FA_READ) != FR_OK) {
      delete p->fatFile;
      p->fatFile = NULL;
      fclose(p->fp);
      if (aBuf) sqlite3_free(aBuf);
      return SQLITE_CANTOPEN;
    }
  }
\2"""
    source, n = re.subn(open_pattern, open_code, source, count=1, flags=re.S)
    if n != 1:
        raise RuntimeError("Could not insert direct FatFs open")

for token in ["FIL *fatFile;", "f_close(p->fatFile)", "f_lseek(p->fatFile", "f_read(p->fatFile", "f_size(p->fatFile)", "f_open(p->fatFile"]:
    if token not in source:
        raise RuntimeError("FatFs VFS patch validation failed: " + token)
if "p->sdFile" in source or "fseeko64(" in source or "fopen64(" in source:
    raise RuntimeError("Obsolete large-file VFS patch remains in source")

with open(target, "w", encoding="utf-8") as f:
    f.write(source)
print("Patched SQLite VFS to use direct FatFs unsigned offsets:", target)
