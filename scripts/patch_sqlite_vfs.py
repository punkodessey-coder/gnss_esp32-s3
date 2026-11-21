# Patch Sqlite3Esp32 VFS: use SD_MMC's unsigned 32-bit seek for the main MBTiles DB.
# The stock VFS uses fseek(long), which cannot represent offsets above 2 GiB
# on the ESP32 Arduino toolchain. This script is intentionally regex-based
# against the exact 2.5 upstream layout, while validating every replacement.
Import("env")
import os
import re

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

if "#include <SD_MMC.h>" not in source:
    if "#include <Arduino.h>" not in source:
        raise RuntimeError("Arduino.h include not found in Sqlite3Esp32 VFS")
    source = source.replace("#include <Arduino.h>", "#include <Arduino.h>\n#include <SD_MMC.h>", 1)

field = "  File *sdFile;                   /* heap-owned SD_MMC handle for large MBTiles */"
if "File *sdFile;" not in source:
    source, n = re.subn(r"(  FILE \*fp;\s*/\* File descriptor \*/)", r"\1\n" + field, source, count=1)
    if n != 1:
        raise RuntimeError("Could not insert SD_MMC handle into ESP32File")

close_marker = "  fclose(p->fp);"
close_code = "  if (p->sdFile) { p->sdFile->close(); delete p->sdFile; p->sdFile = NULL; }\n" + close_marker
if "p->sdFile->close()" not in source:
    if close_marker not in source:
        raise RuntimeError("Could not locate ESP32Close fclose")
    source = source.replace(close_marker, close_code, 1)

# Replace the stock fseek/fread pair, allowing upstream trailing comments and spacing.
read_pattern = r"(?m)^[ \t]*ofst\s*=\s*fseek\(p->fp,\s*iOfst,\s*SEEK_SET\);[^\n]*\n(?:[ \t]*//[^\n]*\n)*[ \t]*nRead\s*=\s*fread\(zBuf,\s*1,\s*iAmt,\s*p->fp\);[^\n]*"
read_code = """  if (p->sdFile) {
    if (iOfst < 0 || (uint64_t)iOfst > 0xFFFFFFFFULL)
      return SQLITE_IOERR_READ;
    if (!p->sdFile->seek((uint32_t)iOfst, SeekSet))
      return SQLITE_IOERR_READ;
    nRead = (int)p->sdFile->read((uint8_t *)zBuf, (size_t)iAmt);
    if (nRead < 0) return SQLITE_IOERR_READ;
    if (nRead < iAmt) {
      memset((char *)zBuf + nRead, 0, iAmt - nRead);
      return SQLITE_IOERR_SHORT_READ;
    }
    return SQLITE_OK;
  }
  ofst = fseek(p->fp, (long)iOfst, SEEK_SET);
  if (ofst != 0) return SQLITE_IOERR_READ;
  nRead = fread(zBuf, 1, iAmt, p->fp);"""
if "p->sdFile->seek((uint32_t)iOfst, SeekSet)" not in source:
    source, n = re.subn(read_pattern, read_code, source, count=1)
    if n != 1:
        raise RuntimeError("Could not locate stock ESP32Read fseek/fread pair (pattern mismatch)")

# Use the SD_MMC File size for the main database.
# Locate the function and inject after its ESP32File pointer declaration; this
# avoids depending on whitespace or exact signature formatting.
if "p->sdFile->size()" not in source:
    fn = source.find("static int ESP32FileSize(")
    if fn < 0:
        raise RuntimeError("ESP32FileSize function not found in Sqlite3Esp32 VFS")
    ptr = source.find("ESP32File *p = (ESP32File*)pFile;", fn)
    if ptr < 0:
        raise RuntimeError("ESP32FileSize file pointer declaration not found")
    insert_at = ptr + len("ESP32File *p = (ESP32File*)pFile;")
    source = source[:insert_at] + """
  if (p->sdFile) {
    *pSize = (sqlite3_int64)p->sdFile->size();
    return SQLITE_OK;
  }""" + source[insert_at:]

# Attach the SD_MMC handle after the stock stdio open succeeds.
if "p->sdFile = new File" not in source:
    open_pattern = r"(  if\s*\(\s*p->fp\s*==\s*NULL\s*\)\s*\{\s*.*?\n\s*\}\s*\n)(\s*p->aBuffer\s*=\s*aBuf;)"
    open_code = r"""\1  if (flags & SQLITE_OPEN_MAIN_DB) {
    const char *filePath = zName;
    // The firmware mounts SD_MMC at /sdcard; File::open expects a path
    // relative to that mount, e.g. /maps/archive.mbtiles.
    if (strncmp(filePath, "/sdcard", 7) == 0)
      filePath += 7;
    p->sdFile = new File(SD_MMC.open(filePath, FILE_READ));
    if (!p->sdFile || !(*p->sdFile)) {
      delete p->sdFile;
      p->sdFile = NULL;
      fclose(p->fp);
      if (aBuf) sqlite3_free(aBuf);
      return SQLITE_CANTOPEN;
    }
  }
\2"""
    source, n = re.subn(open_pattern, open_code, source, count=1, flags=re.S)
    if n != 1:
        raise RuntimeError("Could not insert SD_MMC open after stdio open")

required = [
    "File *sdFile;",
    "p->sdFile->close()",
    "p->sdFile->seek((uint32_t)iOfst, SeekSet)",
    "p->sdFile->size()",
    "p->sdFile = new File(SD_MMC.open(filePath, FILE_READ))",
]
for token in required:
    if token not in source:
        raise RuntimeError("VFS patch validation failed; missing: " + token)

with open(target, "w", encoding="utf-8") as f:
    f.write(source)

print("Patched Sqlite3Esp32 main MBTiles reads using SD_MMC File::seek(uint32_t):", target)
