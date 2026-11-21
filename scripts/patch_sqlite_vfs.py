# Patch Sqlite3Esp32 VFS to read the main MBTiles database through SD_MMC File.
# ESP32 Arduino's stdio fseek(long) is signed 32-bit. SD_MMC File::seek(uint32_t)
# supports unsigned offsets and addresses files up to 4 GiB.
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
    raise RuntimeError("Sqlite3Esp32 esp32.cpp not found")

with open(target, "r", encoding="utf-8") as f:
    source = f.read()

if '#include <SD_MMC.h>' not in source:
    source = source.replace('#include <Arduino.h>', '#include <Arduino.h>\n#include <SD_MMC.h>', 1)

if 'File sdFile;' not in source:
    source = source.replace('  FILE *fp;                       /* File descriptor */',
                            '  FILE *fp;                       /* File descriptor */\n  File sdFile;                    /* 32-bit unsigned SD_MMC file handle for large MBTiles */', 1)

# Close the Arduino FS handle along with the stdio handle.
source = source.replace('  fclose(p->fp);', '  p->sdFile.close();\n  fclose(p->fp);', 1)

# Read the main DB using SD_MMC's uint32_t seek. SQLite xRead requires zero-filled
# remainder on a short read. Reject offsets outside the SD_MMC API's 4-GiB range.
old_read = '''  ofst = fseek(p->fp, iOfst, SEEK_SET);
  //if( ofst != 0 ){
  //  return SQLITE_IOERR_READ;
  //}
  nRead = fread(zBuf, 1, iAmt, p->fp); // read(p->fd, zBuf, iAmt);'''
new_read = '''  if (p->sdFile) {
    if (iOfst < 0 || (uint64_t)iOfst > 0xFFFFFFFFULL)
      return SQLITE_IOERR_READ;
    if (!p->sdFile.seek((uint32_t)iOfst, SeekSet))
      return SQLITE_IOERR_SEEK;
    nRead = (int)p->sdFile.read((uint8_t *)zBuf, (size_t)iAmt);
    if (nRead < iAmt && nRead >= 0)
      memset((char *)zBuf + nRead, 0, iAmt - nRead);
  } else {
    ofst = fseek(p->fp, (long)iOfst, SEEK_SET);
    if (ofst != 0)
      return SQLITE_IOERR_READ;
    nRead = fread(zBuf, 1, iAmt, p->fp);
  }'''
if old_read in source:
    source = source.replace(old_read, new_read, 1)
elif new_read not in source:
    raise RuntimeError("Could not locate ESP32Read seek/read block; upstream VFS changed")

# File::size() is size_t (32-bit unsigned on ESP32), sufficient for MBTiles < 4 GiB.
needle = '  ESP32File *p = (ESP32File*)pFile;\n  int rc;                         /* Return code from fstat() call */'
replacement = '  ESP32File *p = (ESP32File*)pFile;\n  if (p->sdFile) {\n    *pSize = (sqlite3_int64)p->sdFile.size();\n    return SQLITE_OK;\n  }\n  int rc;                         /* Return code from fstat() call */'
if needle in source:
    source = source.replace(needle, replacement, 1)
elif replacement not in source:
    raise RuntimeError("Could not locate ESP32FileSize insertion point")

# Open a second handle for the main database through the mounted SD_MMC filesystem.
needle = '''  if( p->fp == NULL){
    if (aBuf)
      sqlite3_free(aBuf);
    //Serial.println("Can't open");
    return SQLITE_CANTOPEN;
  }
  p->aBuffer = aBuf;'''
replacement = '''  if( p->fp == NULL){
    if (aBuf)
      sqlite3_free(aBuf);
    return SQLITE_CANTOPEN;
  }
  if (flags & SQLITE_OPEN_MAIN_DB) {
    const char *filePath = zName;
    const char *mountPath = SD_MMC.mountpoint();
    size_t mountLen = mountPath ? strlen(mountPath) : 0;
    if (mountLen && strncmp(filePath, mountPath, mountLen) == 0)
      filePath += mountLen;
    p->sdFile = SD_MMC.open(filePath, FILE_READ);
    if (!p->sdFile) {
      fclose(p->fp);
      if (aBuf) sqlite3_free(aBuf);
      return SQLITE_CANTOPEN;
    }
  }
  p->aBuffer = aBuf;'''
if needle in source:
    source = source.replace(needle, replacement, 1)
elif replacement not in source:
    raise RuntimeError("Could not locate ESP32Open insertion point")

if 'fopen64(' in source or 'fseeko64(' in source or 'fstat64(' in source:
    raise RuntimeError("Unsupported Newlib *64 calls remain in patched VFS")
if 'p->sdFile->seek((uint32_t)iOfst, SeekSet)' not in source:
    raise RuntimeError("SD_MMC large-file read patch was not applied")

with open(target, "w", encoding="utf-8") as f:
    f.write(source)

print("Patched Sqlite3Esp32 main DB reads to use SD_MMC File::seek(uint32_t):", target)
