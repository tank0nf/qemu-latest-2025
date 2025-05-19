#ifndef HW_I82596_H
#define HW_I82596_H

#define I82596_IOPORT_SIZE       0x20

#include "system/memory.h"
#include "system/address-spaces.h"

#define PORT_RESET              0x00    /* reset 82596 */
#define PORT_SELFTEST           0x01    /* selftest */
#define PORT_ALTSCP             0x02    /* alternate SCB address */
#define PORT_ALTDUMP            0x03    /* Alternate DUMP address */
#define PORT_CA                 0x10    /* QEMU-internal CA signal */
#define PORT_DUMP_STATS         0x04    /* Debug port to dump statistics */

typedef struct I82596State_st I82596State;

struct I82596State_st {
    MemoryRegion mmio;
    MemoryRegion *as;
    qemu_irq irq;
    NICState *nic;
    NICConf conf;
    QEMUTimer *flush_queue_timer;

    hwaddr scp;         /* pointer to SCP */
    uint8_t sysbus;
    uint32_t scb;       /* SCB */
    uint16_t scb_status;
    uint8_t cu_status, rx_status;
    uint16_t lnkst;

    uint32_t cmd_p;     /* addr of current command */
    int ca;
    int ca_active;
    int send_irq;

    /* Statistical counters */
    uint32_t crc_err;             /* CRC errors */
    uint32_t align_err;           /* Alignment errors */
    uint32_t resource_err;        /* Resource errors */
    uint32_t overrun_err;         /* Overrun errors */
    uint32_t rx_frames;           /* Total frames received */
    uint32_t rx_bytes;            /* Total bytes received */
    uint32_t tx_frames;           /* Total frames transmitted */
    uint32_t tx_bytes;            /* Total bytes transmitted */
    uint32_t collisions;          /* Collision count */
    uint32_t late_collisions;     /* Late collision count */
    uint32_t deferred_tx;         /* Deferred transmissions */
    uint32_t rx_short_frame;      /* Frames shorter than minimum */
    uint32_t rx_too_long_frame;   /* Frames longer than maximum */

    /* Hash register (multicast mask array, multiple individual addresses). */
    uint8_t mult[8];
    uint8_t config[14]; /* config bytes from CONFIGURE command */

    uint8_t tx_buffer[0x4000];
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
void i82596_common_init(DeviceState *dev, I82596State *s, NetClientInfo *info);
extern const VMStateDescription vmstate_i82596;
#endif
