/* Offline interoperability vectors. Real DAQ protocol + LwIP, fake Ethernet.
 * No production Ethernet/IDCODE function is linked, no socket is opened.
 * All data and PCAP timestamps are deterministic synthetic test inputs. */
#define main network_fixture_main
#include "test_network.c"
#undef main
#include <stdint.h>

static void LE16(FILE *f,uint16_t value)
{ CHECK(fputc(value & 255U,f)!=EOF); CHECK(fputc(value>>8U,f)!=EOF); }
static void LE32(FILE *f,uint32_t value)
{ LE16(f,(uint16_t)value); LE16(f,(uint16_t)(value>>16U)); }
static FILE *Open(const char *directory,const char *name)
{
  char path[1024];
  CHECK(snprintf(path,sizeof(path),"%s/%s",directory,name)>0);
  { FILE *f=fopen(path,"wb"); CHECK(f!=NULL); return f; }
}
static void Save(const char *directory,const char *name,const uint8_t *bytes,uint32_t size)
{
  FILE *f=Open(directory,name);
  CHECK(fwrite(bytes,1U,size,f)==size); CHECK(fclose(f)==0);
}
static void Capture(FILE *f,uint32_t ordinal)
{
  /* Timestamp ordinal is just an ordering label, not a measured send rate. */
  LE32(f,0U); LE32(f,ordinal); LE32(f,output_size); LE32(f,output_size);
  CHECK(fwrite(output,1U,output_size,f)==output_size);
}
int main(int argc,char **argv)
{
  static const struct {uint32_t gear,mask;uint16_t sequence;} cases[]={
    {0U,31U,0U},{1U,31U,1U},{2U,31U,2U},{2U,31U,65535U},
    {2U,0U,0U},{2U,1U,1U},{2U,3U,2U},{2U,30U,3U},
    {3U,31U,3U},{4U,31U,4U}
  };
  static const struct {uint8_t cmd;uint16_t parameter;} commands[]={
    {6U,0U},{1U,1U},{1U,0U},{1U,2U},{1U,3U},{1U,4U},
    {2U,0U},{2U,1U},{2U,31U}
  };
  uint16_t ch1[50],ch2[50],crc; uint8_t dig[50],frame[260],control[9];
  DAQ_ControlCommand decoded;
  DAQ_StatusState status={0U,31U,0U,0U,0U,1000U};
  FILE *pcap,*manifest,*csv;
  char filename[64];
  uint32_t i,c,encoded1,encoded2,dmask;
  static const uint8_t confirmed_b_mac[6]={0U,0x80U,0xE1U,0U,0U,0x20U};
  static const uint8_t actual_a_mac[6]=DAQ_NET_MAC;
  CHECK(argc==2);
  CHECK(DAQ_NetSetEnabled(1U)==1U); (void)DAQ_NetPoll(); ArpReply(); (void)DAQ_NetPoll();
  CHECK(DAQ_NetReady()!=0U);
  pcap=Open(argv[1],"synthetic_a_udp.pcap");
  LE32(pcap,0xA1B2C3D4U); LE16(pcap,2U); LE16(pcap,4U);
  LE32(pcap,0U); LE32(pcap,0U); LE32(pcap,65535U); LE32(pcap,1U);
  manifest=Open(argv[1],"vectors.csv");
  CHECK(fprintf(manifest,"kind,file,gear,mask,seq,bytes,crc,scope\n")>0);
  for(c=0U;c<sizeof(cases)/sizeof(cases[0]);c++)
  {
    for(i=0U;i<50U;i++) { ch1[i]=(uint16_t)((i*83U+c*17U)&4095U);ch2[i]=(uint16_t)(4095U-ch1[i]);dig[i]=(uint8_t)(i&7U); }
    if(cases[c].gear==2U)
    { CHECK(DAQ_BuildUDPGEAR2CandidateFrame(ch1,ch2,dig,cases[c].mask,cases[c].sequence,frame,sizeof(frame))==DAQ_FRAME_OK); }
    else
    { CHECK(DAQ_BuildSelectedFrameForGear(ch1,ch2,dig,cases[c].gear,cases[c].mask,cases[c].sequence,frame,sizeof(frame))==DAQ_FRAME_OK); }
    CHECK(frame[0]==0xAAU && frame[1]==0x55U && frame[2]==1U && frame[7]==50U);
    CHECK(frame[3]==(uint8_t)cases[c].sequence && frame[4]==(uint8_t)(cases[c].sequence>>8U));
    CHECK(frame[5]==cases[c].gear && frame[6]==cases[c].mask);
    crc=DAQ_CRC16_CCITT_FALSE(frame,258U); CHECK(frame[258]==(uint8_t)crc && frame[259]==(uint8_t)(crc>>8U));
    dmask=(cases[c].mask>>2U)&7U;
    for(i=0U;i<50U;i++)
    {
      encoded1=(cases[c].mask&1U)?((uint32_t)ch1[i]<<4U):0U;
      encoded2=(cases[c].mask&2U)?((uint32_t)ch2[i]<<4U):0U;
      CHECK(frame[8U+5U*i]==(uint8_t)encoded1 && frame[9U+5U*i]==(uint8_t)(encoded1>>8U));
      CHECK(frame[10U+5U*i]==(uint8_t)encoded2 && frame[11U+5U*i]==(uint8_t)(encoded2>>8U));
      CHECK(frame[12U+5U*i]==(uint8_t)(dig[i]&dmask));
    }
    CHECK(DAQ_NetSendFrame(frame,260U)==1U && output_size==302U);
    CheckWireChecksums(); CHECK(memcmp(output+42U,frame,260U)==0);
    CHECK(memcmp(output,confirmed_b_mac,6U)==0 && memcmp(output+6U,actual_a_mac,6U)==0);
    CHECK(output[34]==0x13U && output[35]==0x8EU && output[36]==0x13U && output[37]==0x8DU);
    Capture(pcap,c);
    CHECK(snprintf(filename,sizeof(filename),"data_%02u.bin",(unsigned)c)>0); Save(argv[1],filename,frame,260U);
    CHECK(fprintf(manifest,"data,%s,%u,%u,%u,260,%04X,%s\n",filename,(unsigned)cases[c].gear,(unsigned)cases[c].mask,(unsigned)cases[c].sequence,(unsigned)crc,cases[c].gear==2U?"candidate-not-CMD01-supported":"production-gear")>0);
    CHECK(snprintf(filename,sizeof(filename),"samples_%02u.csv",(unsigned)c)>0);
    csv=Open(argv[1],filename); CHECK(fprintf(csv,"point,raw_ch1,raw_ch2,dig_input\n")>0);
    for(i=0U;i<50U;i++) { CHECK(fprintf(csv,"%u,%u,%u,%u\n",(unsigned)i,(unsigned)ch1[i],(unsigned)ch2[i],(unsigned)dig[i])>0); }
    CHECK(fclose(csv)==0);
  }
  for(c=0U;c<5U;c++)
  {
    status.gear=c; CHECK(DAQ_BuildStatusFrame(&status,frame,sizeof(frame))==DAQ_STATUS_OK);
    CHECK(DAQ_NetSendFrame(frame,12U)==1U && output_size==54U);
    CheckWireChecksums(); CHECK(memcmp(output+42U,frame,12U)==0);
    CHECK(memcmp(output,confirmed_b_mac,6U)==0 && memcmp(output+6U,actual_a_mac,6U)==0); Capture(pcap,8U+c);
    crc=DAQ_CRC16_CCITT_FALSE(frame,10U);
    CHECK(snprintf(filename,sizeof(filename),"status_%02u.bin",(unsigned)c)>0); Save(argv[1],filename,frame,12U);
    CHECK(fprintf(manifest,"status,%s,%u,31,NA,12,%04X,production-gear\n",filename,(unsigned)c,(unsigned)crc)>0);
  }
  for(c=0U;c<sizeof(commands)/sizeof(commands[0]);c++)
  {
    MakeControl(control,commands[c].cmd,commands[c].parameter);
    CHECK(DAQ_ParseControlFrame(control,9U,&decoded)==DAQ_CONTROL_OK);
    CHECK(decoded.command==commands[c].cmd && decoded.parameter==commands[c].parameter);
    Datagram(control,9U,20U,5005U);
    CHECK(DAQ_NetTakeControl(&decoded)==1U && decoded.command==commands[c].cmd && decoded.parameter==commands[c].parameter);
    CHECK(snprintf(filename,sizeof(filename),"control_%02u.bin",(unsigned)c)>0); Save(argv[1],filename,control,9U);
    crc=DAQ_CRC16_CCITT_FALSE(control,7U);
    CHECK(fprintf(manifest,"control,%s,NA,NA,NA,9,%04X,%s\n",filename,(unsigned)crc,(commands[c].cmd==1U && commands[c].parameter==2U)?"parse-valid-execution-unsupported":"parsed-only-no-HAL-execution")>0);
  }
  CHECK(fclose(manifest)==0 && fclose(pcap)==0);
  CHECK(daq_net.data_submitted==10U && daq_net.status_submitted==5U);
  printf("PASS offline UDP interoperability vectors: %lu checks; 24 payloads/15 synthetic PCAP packets, no hardware\n",(unsigned long)checks);
  return 0;
}
