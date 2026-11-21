# Patch Sqlite3Esp32 VFS to read the main MBTiles database through SD_MMC File.
# stdio fseek(long) is signed 32-bit on ESP32. SD_MMC File::seek(uint32_t)
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

if "#include <SD_MMC.h>" not in source:
    source = source.replace("#include <Arduino.h>", "#include <Arduino.h>\n#include <SD_MMC.h>", 1)

old_field = "  FILE *fp;                       /* File descriptor */"
new_field = old_field + "\n  File *sdFile;                   /* heap-owned SD_MMC handle for large MBTiles */"
if "File *sdFile;" not in source:
    if old_field not in source:
        raise RuntimeError("Could not locate ESP32File FILE field")
    source = source.replace(old_field, new_field, 1)

old_close = "  fclose(p->fp);"
new_close = "  if (p->sdFile) { p->sdFile->close(); delete p->sdFile; p->sdFile = NULL; }\n  fclose(p->fp);"
if new_close not in source:
    if old_close not in source:
        raise RuntimeError("Could not locate ESP32Close fclose")
    source = source.replace(old_close, new_close, 1)

old_read = """  ofst = fseek(p->fp, iOfst, SEEK_SET);
  //if( ofst != 0 ){
  //  return SQLITE_IOERR_READ;
  //}
  nRead = fread(zBuf, 1, iAmt, p->fp); // read(p->fd, zBuf, iAmt);"""
new_read = """  if (p->sdFile) {
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
if new_read not in source:
    if old_read not in source:
        raise RuntimeError("Could not locate ESP32Read seek/read block")
    source = source.replace(old_read, new_read, 1)

old_size = """  ESP32File *p = (ESP32File*)pFile;
  int rc;                         /* Return code from fstat() call */"""
new_size = """  ESP32File *p = (ESP32File*)pFile;
  if (p->sdFile) {
    *pSize = (sqlite3_int64)p->sdFile->size();
    return SQLITE_OK;
  }
  int rc;                         /* Return code from fstat() call */"""
if new_size not in source:
    if old_size not in source:
        raise RuntimeError("Could not locate ESP32FileSize insertion point")
    source = source.replace(old_size, new_size, 1)

old_open = """  if( p->fp == NULL){
    if (aBuf)
      sqlite3_free(aBuf);
    //Serial.println("Can't open");
    return SQLITE_CANTOPEN;
  }
  p->aBuffer = aBuf;"""
new_open = """  if( p->fp == NULL){
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
    p->sdFile = new File(SD_MMC.open(filePath, FILE_READ));
    if (!p->sdFile || !(*p->sdFile)) {
      delete p->sdFile;
      p->sdFile = NULL;
      fclose(p->fp);
      if (aBuf) sqlite3_free(aBuf);
      return SQLITE_CANTOPEN;
    }
  }
  p->aBuffer = aBuf;"""
if new_open not in source:
    if old_open not in source:
        raise RuntimeError("Could not locate ESP32Open insertion point")
    source = source.replace(old_open, new_open, 1)

if "fopen64(" in source or "fseeko64(" in source or "fstat64(" in source:
    raise RuntimeError("Unsupported Newlib *64 calls remain in patched VFS")
if "p->sdFile->seek((uint32_t)iOfst, SeekSet)" not in source:
    raise RuntimeError("SD_MMC large-file read patch was not applied")

with open(target, "w", encoding="utf-8") as f:
    f.write(source)

print("Patched Sqlite3Esp32 main DB reads to use SD_MMC File::seek(uint32_t):", target)
