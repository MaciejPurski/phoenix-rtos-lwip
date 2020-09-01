#ifndef NET_INTEL82567_H_
#define NET_INTEL82567_H_

/* General Registers */
#define REG_CTRL		0x00000
#define REG_STATUS		0x00008
#define REG_STRAP		0x0000C
#define REG_EEC			0x00010
#define REG_EEPROM		0x00014
#define REG_CTRL_EXIT	0x00018
#define REG_MDIC		0x00020
#define REG_FEXTNVM		0x00028
#define REG_FEXT		0x0002C
#define REG_BUSNUM		0x00038
#define REG_FCTTV		0x00170
#define REG_FCRTV		0x05F40
#define REG_LEDCTL		0x00E00
#define REG_EXTCNF_CTRL	0x00F00
#define REG_EXTCNF_SIZE	0x00F08
#define REG_PHY_CTRL	0x00F10
#define REG_PCIANACFG	0x00F18
#define REG_PBA			0x01000
#define REG_PBS			0x01008

/* Interrupt Registers */
#define REG_ICR			0x000C0
#define REG_ITR			0x000C4
#define REG_ICS			0x000C8
#define REG_IMS			0x000D0
#define REG_IMC			0x000D8
#define REG_IAM			0x000E0

/* Receive Registers */
#define REG_RCTL		0x00100
#define REG_RCTL1		0x00104
#define REG_ERT			0x02008
#define REG_FCRTL		0x02160
#define REG_FCRTH		0x02168
#define REG_PSRCTL		0x02170
#define REG_RDBAL0		0x02800
#define REG_RDBAH0		0x02804
#define REG_RDLEN0		0x02808
#define REG_RDH0		0x02810
#define REG_RDT0		0x02818
#define REG_RDTR		0x02820
#define REG_RXDCTL		0x02828
#define REG_RADV		0x0282C
#define REG_RDBAL1		0x02900
#define REG_RDBAH1		0x02904
#define REG_RDLEN		0x02908
#define REG_RDH1		0x02910
#define REG_RDT1		0x02918
#define REG_RSRPD		0x02C00
#define REG_RAID		0x02C08
#define REG_CPUVEC		0x02C10
#define REG_RXCSUM		0x05000
#define REG_RFCTL		0x05008
#define REG_MTA			0x52000
#define REG_RAL			0x05400
#define REG_RAH			0x05404

/* Transmit Registers */
#define REG_TCTL		0x00400
#define REG_TIPG		0x00410
#define REG_AIT			0x00458
#define REG_KABGTXD		0x03004
#define REG_TDBAL0		0x03800
#define REG_TDBAH0		0x03804
#define REG_TDLEN0		0x03808
#define REG_TDH0		0x03810
#define REG_TDT0		0x03818
#define REG_TIDV		0x0382C
#define REG_TARC		0x03840

/* Transmit Control Register flags */
#define TCTL_EN			0x00000001
#define TCTL_PSP		0x00000008
#define TCTL_CT     	0x00000ff0
#define TCTL_COLD   	0x003ff000
#define TCTL_RTLC   	0x01000000
#define TCTL_MULR   	0x10000000


/* Device Control Register flags*/
#define CTRL_FD				0x00000001
#define CTRL_MDISABLE 		0x00000004
#define CTRL_SPDMASK  		0x00000300
#define CTRL_SPD10   		0x00000000
#define CTRL_SPD100  		0x00000100
#define CTRL_SPD_1000 		0x00000200 
#define CTRL_FRCSPD			0x00000800
#define CTRL_FRCDPLX		0x00001000
#define CTRL_LCDPD			0x01000000
#define CTRL_H2MEINT		0x02000000
#define CTRL_SWRST			0x04000000
#define CTRL_RFCE			0x08000000
#define CTRL_TFCE			0x10000000
#define CTRL_VME			0x40000000
#define CTRL_LCD_RST		0x80000000

/* Interrupt Mask Set Register flags */
#define IMS_TXDW			0x00000001
#define IMS_TXQE			0x00000002
#define IMS_LSC				0x00000004
#define IMS_RXDMT0			0x00000010
#define IMS_DSW				0x00000020
#define IMS_RXO				0x00000040
#define IMS_RXT0			0x00000080
#define IMS_MDAC			0x00000200
#define IMS_PHYINT			0x00001000
#define IMS_TXD_LOW			0x00008000
#define IMS_SRPD          	0x00010000 /* Small Receive Packet Detected */
#define IMS_ACK           	0x00020000 /* Receive ACK Frame Detected */
#define IMS_MNG           	0x00040000 /* Manageability Event Detected */
#define IMS_EPRST			0x00100000

/* Receive Descriptor Status Field */
#define RSTAT_DD			0x01	/* Descriptor Done */
#define RSTAT_EOP			0x02	/* End of Packet */
	

/* From OSDEV */
#define RCTL_EN                         (1 << 1)    // Receiver Enable
#define RCTL_SBP                        (1 << 2)    // Store Bad Packets
#define RCTL_UPE                        (1 << 3)    // Unicast Promiscuous Enabled
#define RCTL_MPE                        (1 << 4)    // Multicast Promiscuous Enabled
#define RCTL_LPE                        (1 << 5)    // Long Packet Reception Enable
#define RCTL_LBM_NONE                   (0 << 6)    // No Loopback
#define RCTL_LBM_PHY                    (3 << 6)    // PHY or external SerDesc loopback
#define RTCL_RDMTS_HALF                 (0 << 8)    // Free Buffer Threshold is 1/2 of RDLEN
#define RTCL_RDMTS_QUARTER              (1 << 8)    // Free Buffer Threshold is 1/4 of RDLEN
#define RTCL_RDMTS_EIGHTH               (2 << 8)    // Free Buffer Threshold is 1/8 of RDLEN
#define RCTL_MO_36                      (0 << 12)   // Multicast Offset - bits 47:36
#define RCTL_MO_35                      (1 << 12)   // Multicast Offset - bits 46:35
#define RCTL_MO_34                      (2 << 12)   // Multicast Offset - bits 45:34
#define RCTL_MO_32                      (3 << 12)   // Multicast Offset - bits 43:32
#define RCTL_BAM                        (1 << 15)   // Broadcast Accept Mode
#define RCTL_VFE                        (1 << 18)   // VLAN Filter Enable
#define RCTL_CFIEN                      (1 << 19)   // Canonical Form Indicator Enable
#define RCTL_CFI                        (1 << 20)   // Canonical Form Indicator Bit Value
#define RCTL_DPF                        (1 << 22)   // Discard Pause Frames
#define RCTL_PMCF                       (1 << 23)   // Pass MAC Control Frames
#define RCTL_SECRC                      (1 << 26)   // Strip Ethernet CRC

/* Transmit commands */
#define CMD_EOP                         (1 << 0)    // End of Packet
#define CMD_IFCS                        (1 << 1)    // Insert FCS
#define CMD_IC                          (1 << 2)    // Insert Checksum
#define CMD_RS                          (1 << 3)    // Report Status
#define CMD_RPS                         (1 << 4)    // Report Packet Sent
#define CMD_VLE                         (1 << 6)    // VLAN Packet Enable
#define CMD_IDE                         (1 << 7)    // Interrupt Delay Enable

// Buffer Sizes
#define RCTL_BSIZE_256                  (3 << 16)
#define RCTL_BSIZE_512                  (2 << 16)
#define RCTL_BSIZE_1024                 (1 << 16)
#define RCTL_BSIZE_2048                 (0 << 16)
#define RCTL_BSIZE_4096                 ((3 << 16) | (1 << 25))
#define RCTL_BSIZE_8192                 ((2 << 16) | (1 << 25))
#define RCTL_BSIZE_16384                ((1 << 16) | (1 << 25))
 



#endif