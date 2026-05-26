# firmware/

Pre-built ESP32 binaries ready to flash. Flash offsets:

| File | Offset |
|---|---|
| `bootloader.bin` | `0x1000` |
| `partition-table.bin` | `0x8000` |
| `embedded.bin` | `0x10000` |

## Regenerating

Run this once on a machine with ESP-IDF v6.0 installed (expected at `~/.espressif/v6.0/esp-idf`):

```bash
cd embedded
idf.py build
cd ..
cp embedded/build/bootloader/bootloader.bin             firmware/
cp embedded/build/partition_table/partition-table.bin   firmware/
cp embedded/build/embedded.bin                          firmware/
git add firmware/*.bin && git commit -m "Update pre-built firmware binaries"
```

The web app reads from this directory. If any file is missing, the **Run measurement** button will be disabled and the firmware banner will show which files are missing.
