"""Bounded PC-side A-board UDP probe, not B-board firmware.
Default/offline/self-test modes never open a socket. --live is explicit and
must be used only after the user authorizes a physical network test.
"""
import argparse, binascii, json, pathlib, statistics, struct, time

class FrameError(ValueError):
    def __init__(self, kind):
        super().__init__(kind)
        self.kind = kind

def decode(payload):
    if len(payload) not in (12,260):
        raise FrameError("format")
    if payload[:2] != b"\xaa\x55":
        raise FrameError("format")
    if binascii.crc_hqx(payload[:-2],0xffff) != int.from_bytes(payload[-2:],"little"):
        raise FrameError("crc")
    if len(payload) == 12:
        if payload[2] != 3 or payload[3] > 4 or payload[4] > 31 or payload[5] > 5 or payload[6] > 2:
            raise FrameError("format")
        frequency = int.from_bytes(payload[8:10],"little")
        if not 100 <= frequency <= 10000:
            raise FrameError("format")
        return {"kind":"status","gear":payload[3],"mask":payload[4],
                "trigger_source":payload[5],"trigger_mode":payload[6],
                "trigger_level":payload[7],"wave_hz":frequency}
    if payload[2] != 1 or payload[5] > 4 or payload[6] > 31 or payload[7] != 50:
        raise FrameError("format")
    mask = payload[6]
    ch1, ch2, dig = [], [], []
    for offset in range(8,258,5):
        one,two,bits = struct.unpack_from("<HHB",payload,offset)
        if one & 15 or two & 15 or bits & ~((mask >> 2) & 7):
            raise FrameError("format")
        if (not mask & 1 and one) or (not mask & 2 and two):
            raise FrameError("format")
        ch1.append(one >> 4); ch2.append(two >> 4); dig.append(bits)
    return {"kind":"data","gear":payload[5],"mask":mask,
            "seq":int.from_bytes(payload[3:5],"little"),"ch1":ch1,"ch2":ch2,"dig":dig}

class Probe:
    def __init__(self, gear=None):
        self.gear = gear
        self.data = self.status = self.bad_crc = self.bad_format = self.wrong_peer = 0
        self.forward_gaps = self.duplicates = self.backward_or_restart = 0
        self.last_seq = None
        self.first = self.last = None
        self.by_gear = {}
        self.retained = []
        self.last_status = None
        self.last_frame = None
    def add(self, payload, peer, timestamp):
        if peer != ("192.168.1.10",5006):
            self.wrong_peer += 1
            return
        try:
            frame = decode(payload)
        except FrameError as error:
            if error.kind == "crc":
                self.bad_crc += 1
            else:
                self.bad_format += 1
            return
        if frame["kind"] == "status":
            self.status += 1
            self.last_status = frame
            return
        self.data += 1
        seq = frame["seq"]
        if self.last_seq is not None:
            if seq == self.last_seq:
                self.duplicates += 1
            else:
                delta = (seq-self.last_seq-1) & 65535
                if delta < 32768:
                    self.forward_gaps += delta
                else:
                    self.backward_or_restart += 1
        self.last_seq = seq
        if self.first is None:
            self.first = timestamp
        self.last = timestamp
        item = self.by_gear.setdefault(frame["gear"],{"count":0,"first":timestamp,"last":timestamp})
        item["count"] += 1; item["last"] = timestamp
        self.last_frame = payload.hex()
        if len(self.retained) < 32 and (self.gear is None or frame["gear"] == self.gear):
            self.retained.append(frame)
    def summary(self):
        rates = {}
        for gear,item in self.by_gear.items():
            span = item["last"]-item["first"]
            rates[gear] = {"frames":item["count"],"receive_window_fps":(item["count"]-1)/span if span > 0 else None}
        quality = {}
        for name,bit in (("ch1",1),("ch2",2)):
            samples = [value for frame in self.retained if frame["mask"] & bit for value in frame[name]]
            quality[name] = {"n":len(samples),"mean_codes":statistics.mean(samples) if samples else None,
                             "peak_to_peak_codes":max(samples)-min(samples) if samples else None,
                             "population_std_codes":statistics.pstdev(samples) if samples else None}
        return {"valid_data":self.data,"valid_status":self.status,"bad_crc":self.bad_crc,
                "bad_format":self.bad_format,"unexpected_peer":self.wrong_peer,
                "seq_forward_gaps":self.forward_gaps,"seq_duplicates":self.duplicates,
                "seq_backward_or_restart":self.backward_or_restart,
                "gear_receive_rates":rates,"quality_from_first_at_most_32_selected_frames":quality,
                "quality_gear_filter":self.gear,"adc_sampling_cycles_verified":False,
                "retained_frames":self.retained,"last_status":self.last_status,
                "last_data_frame_hex":self.last_frame,
                "limits":"Receive statistics do not prove ADC timing, transmitter group rate, analog accuracy or B-board acceptance; SEQ discontinuities alone do not identify their cause."}

def read_pcap(path):
    data = pathlib.Path(path).read_bytes()
    if len(data)<24 or data[:4] != b"\xd4\xc3\xb2\xa1":
        raise ValueError("Expected little-endian Ethernet PCAP")
    pos = 24
    while pos < len(data):
        sec,usec,n,original = struct.unpack_from("<IIII",data,pos)
        pos += 16
        wire = data[pos:pos+n]; pos += n
        if len(wire) != n or n != original or len(wire)<42 or wire[12:14] != b"\x08\x00" or wire[14] != 0x45 or wire[23] != 17:
            raise ValueError("Unsupported/truncated PCAP packet")
        source_port,dest_port,length = struct.unpack_from("!HHH",wire,34)
        if dest_port != 5005 or len(wire) != 34+length:
            raise ValueError("Unexpected UDP destination/length")
        source_ip = ".".join(str(value) for value in wire[26:30])
        yield wire[42:],(source_ip,source_port),sec+usec/1e6

def live(probe,seconds,request_status):
    # Deliberately imported/created only on this explicitly selected path.
    import socket
    command = bytes.fromhex("aa550206000000")
    command += binascii.crc_hqx(command,0xffff).to_bytes(2,"little")
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as channel:
        channel.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,1024*1024)
        channel.bind(("192.168.1.20",5005))
        channel.settimeout(0.2)
        if request_status:
            channel.sendto(command,("192.168.1.10",5006))  # CMD06 once, no gear/mask writes.
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            try:
                payload,peer = channel.recvfrom(65535)
            except socket.timeout:
                continue
            probe.add(payload,peer,time.monotonic())

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--pcap")
    mode.add_argument("--live",action="store_true")
    parser.add_argument("--seconds",type=float,default=10)
    parser.add_argument("--request-status",action="store_true")
    parser.add_argument("--gear",type=int,choices=(0,1,2))
    parser.add_argument("--output",required=True)
    args = parser.parse_args()
    if not 1 <= args.seconds <= 60:
        parser.error("--seconds must be 1..60")
    if args.request_status and not args.live:
        parser.error("--request-status requires explicit --live")
    target = pathlib.Path(args.output)
    if target.exists():
        parser.error("Output already exists; choose a new path")
    probe = Probe(args.gear)
    if args.live:
        live(probe,args.seconds,args.request_status)
    else:
        for payload,peer,stamp in read_pcap(args.pcap):
            probe.add(payload,peer,stamp)
    result = probe.summary()
    result["mode"] = "live-receiver" if args.live else "offline-synthetic-pcap"
    result["timing_evidence"] = ("PC monotonic receive timestamps only; not ADC/IRQ measurement"
                                 if args.live else "synthetic ordering only; no rate evidence")
    if not args.live:
        for item in result["gear_receive_rates"].values():
            item["receive_window_fps"] = None
    target.parent.mkdir(parents=True,exist_ok=True)
    with target.open("x",encoding="utf-8") as stream:
        json.dump(result,stream,indent=2)
    print(json.dumps({key:result[key] for key in ("mode","valid_data","valid_status","bad_crc","bad_format")}))
    return 0 if probe.data and (not args.request_status or probe.status) else 2

if __name__ == "__main__":
    raise SystemExit(main())
