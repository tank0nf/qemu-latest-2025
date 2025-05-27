#ifndef HW_I82596_H
#define HW_I82596_H

#define I82596_IOPORT_SIZE       0x20

#include "system/memory.h"
#include "system/address-spaces.h"

#define PORT_RESET              0x00    /* reset 82596 */
#define PORT_SELFTEST           0x01    /* selftest */
#define PORT_ALTSCP             0x02    /* alternate SCB address */
#define PORT_ALTDUMP            0x03    /* Alternate DUMP address */
#define PORT_CA                 0x04    /* QEMU-internal CA signal */
#define PORT_BYTEMASK           0x07    /* all valid bits */

#define MIN_BUF_SIZE    60      /* minimum Ethernet frame size */
#define PKT_BUF_SZ      1536
#define PKT_BUF_SZ_MAX  1514    /* max. size of a packet buffer, including CRC */


/* modes in which the 82596 can operate */
#define MODE_82586              0       /* 24 bit address space */
#define MODE_32BIT_SEGMENTED    1
#define MODE_LINEAR             2       /* 32 bit address space */
#define MODE_UNKNOWN            3

typedef struct {
    /* Configuration Area - 0x00-0x0F */
    uint8_t config_bytes[14];           /* 0x00-0x0D: Configuration bytes */
    uint16_t reserved1;                 /* 0x0E-0x0F: Reserved */
    
    /* Individual Address - 0x10-0x17 */
    uint8_t individual_addr[6];         /* 0x10-0x15: MAC address */
    uint16_t reserved2;                 /* 0x16-0x17: Reserved */
    
    /* Statistical Counters - 0x18-0x27 */
    uint32_t crc_errors;               /* 0x18-0x1B: CRC error counter */
    uint32_t alignment_errors;         /* 0x1C-0x1F: Alignment error counter */
    uint32_t resource_errors;          /* 0x20-0x23: Resource error counter */
    uint32_t overrun_errors;           /* 0x24-0x27: Overrun error counter */
    
    /* Multicast Hash Register - 0x28-0x2F */
    uint8_t multicast_hash[8];         /* 0x28-0x2F: Multicast hash bits */
    
    /* Current Status - 0x30-0x3F */
    uint16_t cu_ru_status;             /* 0x30-0x31: CU/RU status */
    uint16_t link_status;              /* 0x32-0x33: Link status */
    uint32_t scb_address;              /* 0x34-0x37: SCB address */
    uint32_t current_cmd_ptr;          /* 0x38-0x3B: Current command pointer */
    uint32_t bus_control;              /* 0x3C-0x3F: Bus control registers */
    
    /* Internal Microcode Area - 0x40-0x11F */
    uint8_t microcode_area[0xE0];      /* 0x40-0x11F: Internal microcode (224 bytes) */
    
    /* System Configuration - 0x120-0x12F */
    uint32_t sysbus_config;            /* 0x120-0x123: System bus configuration */
    uint32_t additional_control;       /* 0x124-0x127: Additional control registers */
    uint64_t reserved3;                /* 0x128-0x12F: Reserved area */
    
    /* PORT_ALTDUMP status word appears at offset 0x130 (304) */
} __attribute__((packed)) I82596DumpArea;

typedef struct {
    uint32_t crc_errors;
    uint32_t alignment_errors; 
    uint32_t resource_errors;
    uint32_t overrun_errors;
    uint32_t collisions;
    uint32_t frames_transmitted;
    uint32_t frames_received;
} I82596Stats;

typedef struct I82596State_st I82596State;

struct I82596State_st {
    MemoryRegion mmio;
    MemoryRegion *as;

    I82596Stats stats;
    qemu_irq irq;
    NICState *nic;
    NICConf conf;
    QEMUTimer *flush_queue_timer;

    hwaddr scp;         /* pointer to SCP */
    uint8_t  sysbus;    /* SYSBUS byte */
    uint32_t scb;       /* SCB */
    uint16_t scb_status;
    uint8_t  CUS:3;     /* Command Unit status word in SCB */
    uint8_t  RUS:4;     /* Receive Unit status word in SCB */
    uint16_t lnkst;
    uint32_t cmd_p;     /* addr of current command */
    int ca;
    int ca_active;
    uint8_t  send_irq;
    /* Hash register (multicast mask array, multiple individual addresses). */
    uint8_t mult[8];
    uint8_t config[14]; /* config bytes from CONFIGURE command */

    uint8_t tx_buffer[PKT_BUF_SZ_MAX];
};

void i82596_h_reset(void *opaque);
void i82596_ioport_writew(void *opaque, uint32_t addr, uint32_t val);
uint32_t i82596_ioport_readw(void *opaque, uint32_t addr);
void i82596_ioport_writel(void *opaque, uint32_t addr, uint32_t val);
uint32_t i82596_ioport_readl(void *opaque, uint32_t addr);
uint32_t i82596_bcr_readw(I82596State *s, uint32_t rap);
ssize_t i82596_receive(NetClientState *nc, const uint8_t *buf, size_t size_);
bool i82596_can_receive(NetClientState *nc);
void i82596_set_link_status(NetClientState *nc);
// void i82596_perform_dump(I82596State *s, uint32_t dump_addr, bool is_port_dump);
// void i82596_update_tx_stats(I82596State *s, bool success, uint16_t status);
// void i82596_update_rx_stats(I82596State *s, bool frame_ok, uint16_t status, size_t frame_len);
void i82596_common_init(DeviceState *dev, I82596State *s, NetClientInfo *info);
extern const VMStateDescription vmstate_i82596;
#endif
