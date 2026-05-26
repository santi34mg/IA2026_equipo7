@echo off
REM Flashea la ESP32 con los binarios recien compilados en embedded/build/.
REM Requiere haber corrido antes:  cd embedded ^&^& idf.py build
REM Uso:  flash esp32.bat            (usa COM3 por defecto)
REM       flash esp32.bat COM5       (especifica otro puerto)

setlocal
set PORT=%1
if "%PORT%"=="" set PORT=COM3

python -m esptool ^
  --chip esp32 ^
  -p %PORT% ^
  -b 460800 ^
  --before default-reset ^
  --after hard-reset ^
  write-flash ^
  --flash-mode dio ^
  --flash-size 2MB ^
  --flash-freq 40m ^
  0x1000  "%~dp0embedded\build\bootloader\bootloader.bin" ^
  0x8000  "%~dp0embedded\build\partition_table\partition-table.bin" ^
  0x10000 "%~dp0embedded\build\embedded.bin"

endlocal
