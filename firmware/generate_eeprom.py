import struct

def calc_checksum(b):
    checksum = 0x55
    for byte in b:
        checksum ^= byte
    return checksum

magic = b"THEX"
version = b"2024"
fsoffset = 32
fspage_size = 32
fstotal_size = 0
vid = 0xcafe
pid = 0xdc01
uid = 0
friendly_name = b"OMNIWHEEL"

eep = struct.pack("<4s4sHHIHHH9s", 
                  magic,
                  version,
                  fsoffset,
                  fspage_size,
                  fstotal_size,
                  vid,
                  pid,
                  uid,
                  friendly_name)

eep += bytes([calc_checksum(eep[1:])])

with open("Core/Src/fake_eeprom.c", "w") as fw:
    fw.write("#include <stdint.h>\n")
    fw.write("\n")
    fw.write("const uint8_t eeprom[256] = \"")
    for x in eep:
        fw.write(f"\\x{x:02x}")
    fw.write("\";\n")
