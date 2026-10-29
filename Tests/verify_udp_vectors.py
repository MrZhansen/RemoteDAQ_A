"""Offline validation of synthetic interoperability artifacts; no network access."""
import binascii, csv, json, pathlib, struct, sys

checks = 0
def check(value, label):
    global checks
    checks += 1
    if not value:
        raise AssertionError(label)

def words(data):
    if len(data) & 1:
        data += b"\0"
    total = sum(struct.unpack("!" + "H" * (len(data)//2), data))
    while total >> 16:
        total = (total & 65535) + (total >> 16)
    return total

directory = pathlib.Path(sys.argv[1])
check(binascii.crc_hqx(b"123456789", 0xffff) == 0x29b1, "independent CRC vector")
rows = list(csv.DictReader((directory/"vectors.csv").open(newline="", encoding="ascii")))
check(len(rows) == 18, "18 deterministic payloads")
wire_payloads = []
for row in rows:
    data = (directory/row["file"]).read_bytes()
    check(len(data) == int(row["bytes"]), "payload length")
    check(data[:2] == b"\xaa\x55", "header")
    check(binascii.crc_hqx(data[:-2], 0xffff) == int.from_bytes(data[-2:], "little"), "CCITT-FALSE LE CRC")
    if row["kind"] == "data":
        check(data[2] == 1 and data[5] == int(row["gear"]) and data[6] == int(row["mask"]) and data[7] == 50, "data fields")
        check(int.from_bytes(data[3:5], "little") == int(row["seq"]), "SEQ endian")
        samplefile = "samples_" + row["file"][5:7] + ".csv"
        samples = list(csv.DictReader((directory/samplefile).open(newline="", encoding="ascii")))
        check(len(samples) == 50, "sample count")
        mask = int(row["mask"])
        for point, sample in enumerate(samples):
            off = 8 + point*5
            ch1, ch2, dig = struct.unpack_from("<HHB", data, off)
            check(ch1 == (int(sample["raw_ch1"]) << 4 if mask & 1 else 0), "CH1 data")
            check(ch2 == (int(sample["raw_ch2"]) << 4 if mask & 2 else 0), "CH2 data")
            check(dig == (int(sample["dig_input"]) & ((mask >> 2) & 7)), "DIG mapping/mask")
        wire_payloads.append(data)
    elif row["kind"] == "status":
        check(data[2] == 3 and data[3] == int(row["gear"]) and data[4] == 31, "status fields")
        check(data[5:8] == b"\0\0\0" and int.from_bytes(data[8:10], "little") == 1000, "truthful example fields")
        wire_payloads.append(data)
    else:
        check(data[2] == 2 and data[6] == 0, "control type/reserved")

pcap = (directory/"synthetic_a_udp.pcap").read_bytes()
check(struct.unpack_from("<IHH", pcap) == (0xa1b2c3d4, 2, 4), "PCAP LE/version")
check(struct.unpack_from("<I", pcap, 20)[0] == 1, "Ethernet link type")
pos, packets = 24, 0
while pos < len(pcap):
    seconds, micros, captured, original = struct.unpack_from("<IIII", pcap, pos)
    pos += 16
    wire = pcap[pos:pos+captured]
    pos += captured
    check(seconds == 0 and micros == packets, "synthetic ordering only")
    check(captured == original and len(wire) == captured, "complete PCAP packet")
    check(wire[:6] == bytes.fromhex("0080e1000020"), "confirmed B MAC")
    check(wire[6:12] == bytes.fromhex("0246da00000a"), "configured A MAC")
    check(wire[12:14] == b"\x08\x00" and wire[14] == 0x45 and wire[23] == 17, "IPv4 UDP")
    total = int.from_bytes(wire[16:18], "big")
    check(total+14 == captured and words(wire[14:34]) == 0xffff, "IPv4 length/checksum")
    check(wire[26:30] == bytes((192,168,1,10)) and wire[30:34] == bytes((192,168,1,20)), "IP addresses")
    source, dest, length, checksum = struct.unpack_from("!HHHH", wire, 34)
    check(source == 5006 and dest == 5005 and length+20 == total and checksum != 0, "ports/UDP length")
    pseudo = wire[26:34] + bytes((0,17)) + struct.pack("!H", length)
    check(words(pseudo+wire[34:]) == 0xffff, "independent UDP checksum")
    check(wire[42:] == wire_payloads[packets], "wire payload matches BIN")
    packets += 1
check(packets == 11 and pos == len(pcap), "exactly 11 complete packets")
result = {"checks": checks, "payloads": len(rows), "pcap_packets": packets, "synthetic": True, "hardware_access": False}
(directory/"independent_validation.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
print("PASS independent Python UDP/vector validation: %d checks" % checks)
