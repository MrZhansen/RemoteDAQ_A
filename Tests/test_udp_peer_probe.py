"""No socket/board access; exercise the PC peer parser and bounded retention."""
import binascii, pathlib, sys
from udp_peer_probe import Probe,decode,read_pcap

directory = pathlib.Path(sys.argv[1]); checks = 0
def check(value):
    global checks
    checks += 1
    if not value:
        raise AssertionError(checks)
def sequence(payload,value):
    data = bytearray(payload); data[3:5] = value.to_bytes(2,"little")
    data[-2:] = binascii.crc_hqx(data[:-2],0xffff).to_bytes(2,"little")
    return bytes(data)
peer = ("192.168.1.10",5006)
p = Probe()
for payload,source,stamp in read_pcap(directory/"synthetic_a_udp.pcap"):
    p.add(payload,source,stamp)
summary = p.summary()
check(summary["valid_data"] == 8 and summary["valid_status"] == 3)
check(summary["bad_crc"] == summary["bad_format"] == 0)
check(summary["seq_backward_or_restart"] == 1)
check(summary["quality_from_first_at_most_32_selected_frames"]["ch1"]["n"] == 300)
check(summary["quality_from_first_at_most_32_selected_frames"]["ch2"]["n"] == 300)
check(summary["adc_sampling_cycles_verified"] is False)
base = (directory/"data_00.bin").read_bytes()
p.add(base,("192.168.1.99",5006),1)
check(p.wrong_peer == 1)
damaged = bytearray(base); damaged[10] ^= 1
p.add(damaged,peer,1)
check(p.bad_crc == 1)
p.add(base[:-1],peer,1)
check(p.bad_format == 1)
damaged = bytearray(base); damaged[8] |= 1
damaged[-2:] = binascii.crc_hqx(damaged[:-2],0xffff).to_bytes(2,"little")
p.add(damaged,peer,1)
check(p.bad_format == 2)
z = Probe(2)
for i in range(100):
    z.add(sequence(base,i),peer,i/1000)
check(len(z.retained) == 0)  # GEAR0 cannot masquerade as GEAR2 quality data.
candidate = (directory/"data_02.bin").read_bytes()
for i in range(100):
    z.add(sequence(candidate,100+i),peer,(100+i)/1000)
check(len(z.retained) == 32)
check(z.summary()["quality_from_first_at_most_32_selected_frames"]["ch1"]["n"] == 1600)
w = Probe()
w.add(sequence(base,65535),peer,0); w.add(sequence(base,0),peer,.001)
check(w.forward_gaps == 0 and w.backward_or_restart == 0)
w.add(sequence(base,0),peer,.002); check(w.duplicates == 1)
w.add(sequence(base,2),peer,.003); check(w.forward_gaps == 1)
check(decode((directory/"status_01.bin").read_bytes())["gear"] == 1)
print("PASS PC UDP peer parser: %d checks; no socket created" % checks)
