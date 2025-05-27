/*
 * QEMU Intel i82596 (Apricot) emulation
 *
 * Copyright (c) 2019 Helge Deller <deller@gmx.de>
 * This work is licensed under the GNU GPL license version 2 or later.
 *
 * This software was written to be compatible with the specification:
 * https://www.intel.com/assets/pdf/general/82596ca.pdf
 */

#include "qemu/osdep.h"
#include "qemu/timer.h"
#include "net/net.h"
#include "net/eth.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "qemu/module.h"
#include "trace.h"
#include "i82596.h"
#include <zlib.h> /* for crc32 */

#if defined(ENABLE_DEBUG)
#define DBG(x)          x
#else
#define DBG(x)          do { } while (0)
#endif

#define USE_TIMER       0

#define BITS(n, m) (((0xffffffffU << (31 - n)) >> (31 - n + m)) << m)

#define MAX_MC_CNT      64

#define ISCP_BUSY       0x0001

#define I596_NULL       ((uint32_t)0xffffffff)

#define SCB_STATUS_CX   0x8000 /* CU finished command with I bit */
#define SCB_STATUS_FR   0x4000 /* RU finished receiving a frame */
#define SCB_STATUS_CNA  0x2000 /* CU left active state */
#define SCB_STATUS_RNR  0x1000 /* RU left active state */

#define SCB_COMMAND_ACK_MASK \
        (SCB_STATUS_CX | SCB_STATUS_FR | SCB_STATUS_CNA | SCB_STATUS_RNR)

#define CU_IDLE         0       /* CUS values */
#define CU_SUSPENDED    1
#define CU_ACTIVE       2

#define RX_IDLE         0       /* RUS values */
#define RX_SUSPENDED    1
#define RX_NO_RESOURCES 2
#define RX_READY        4
#define RX_NO_RESO_RBD  (8 + RX_NO_RESOURCES)
#define RX_NO_MORE_RBD  (8 + RX_READY)

#define CMD_EOL         0x8000  /* The last command of the list, stop. */
#define CMD_SUSP        0x4000  /* Suspend after doing cmd. */
#define CMD_INTR        0x2000  /* Interrupt after doing cmd. */

#define CMD_FLEX        0x0008  /* Enable flexible memory model */

enum commands {
        CmdNOp = 0, CmdSASetup = 1, CmdConfigure = 2, CmdMulticastList = 3,
        CmdTx = 4, CmdTDR = 5, CmdDump = 6, CmdDiagnose = 7
};

#define STAT_C          0x8000  /* Set to 0 after execution */
#define STAT_B          0x4000  /* Command being executed */
#define STAT_OK         0x2000  /* Command executed ok */
#define STAT_A          0x1000  /* Command aborted */

#define I596_EOF        0x8000
#define SIZE_MASK       0x3fff

/* various flags in the chip config registers */
#define I596_PREFETCH       (s->config[0] & 0x80)
#define I596_NO_SRC_ADD_IN  (s->config[3] & 0x08) /* if 1, do not insert MAC in Tx Packet */
#define I596_PROMISC        (s->config[8] & 0x01)
#define I596_BC_DISABLE     (s->config[8] & 0x02)   /* broadcast disable */
#define I596_NOCRC_INS      (s->config[8] & 0x08)   /* do not append CRC to Tx frame */
#define I596_CRC16_32       (s->config[8] & 0x10)   /* CRC-16 or CRC-32 */
#define I596_CRCINM         (s->config[11] & 0x04)  /* Rx CRC appended in memory */
#define I596_MC_ALL         (s->config[11] & 0x20)
#define I596_MULTIIA        (s->config[13] & 0x40)
#define I596_LOOPBACK       (s->config[3] >> 6)     /* loopback mode, 3 = external loopback */


static uint8_t get_byte(uint32_t addr)
{
    return ldub_phys(&address_space_memory, addr);
}

static void set_byte(uint32_t addr, uint8_t c)
{
    return stb_phys(&address_space_memory, addr, c);
}

static uint16_t get_uint16(uint32_t addr)
{
    return lduw_be_phys(&address_space_memory, addr);
}

static void set_uint16(uint32_t addr, uint16_t w)
{
    return stw_be_phys(&address_space_memory, addr, w);
}

static uint32_t get_uint32(uint32_t addr)
{
    uint32_t lo = lduw_be_phys(&address_space_memory, addr);
    uint32_t hi = lduw_be_phys(&address_space_memory, addr + 2);
    return (hi << 16) | lo;
}

static void set_uint32(uint32_t addr, uint32_t val)
{
    set_uint16(addr, (uint16_t) val);
    set_uint16(addr + 2, val >> 16);
}


struct qemu_ether_header {
    uint8_t ether_dhost[6];
    uint8_t ether_shost[6];
    uint16_t ether_type;
};

#define PRINT_PKTHDR(txt, BUF) do {                  \
    struct qemu_ether_header *hdr = (void *)(BUF); \
    printf(txt ": packet dhost=" MAC_FMT ", shost=" MAC_FMT ", type=0x%04x\n",\
           MAC_ARG(hdr->ether_dhost), MAC_ARG(hdr->ether_shost),        \
           be16_to_cpu(hdr->ether_type));       \
} while (0)

static void i82596_update_tx_stats(I82596State *s, bool success, uint16_t status)
{
    s->stats.frames_transmitted++;
    
    if (!success) {
        if (status & (1 << 8)) { /* DMA underrun */
            s->stats.overrun_errors++;
        }
        if (status & (1 << 5)) { /* Too many collisions */
            s->stats.collisions += 16; /* Max collisions reached */
        }
    } else {
        /* Count actual collisions from status bits 0-3 */
        uint8_t collision_count = status & 0x0F;
        if (status & (1 << 5)) { /* S5 set with MAX-COL = 0 means 16 collisions */
            collision_count = 16;
        }
        s->stats.collisions += collision_count;
    }
}

static void i82596_update_rx_stats(I82596State *s, bool frame_ok, uint16_t status, size_t frame_len)
{
    if (frame_ok) {
        s->stats.frames_received++;
    } else {
        if (status & (1 << 11)) { /* CRC error in aligned frame */
            s->stats.crc_errors++;
        }
        if (status & (1 << 10)) { /* Alignment error */
            s->stats.alignment_errors++;
        }
        if (status & (1 << 9)) { /* No resources */
            s->stats.resource_errors++;
        }
        if (status & (1 << 8)) { /* Overrun */
            s->stats.overrun_errors++;
        }
    }
}

static void i82596_transmit(I82596State *s, uint32_t addr)
{
    uint32_t tbd_p;
    uint16_t cmd;
    int insert_crc;
    int sf_bit; /* For checking, 0 = Simplified, 1 = Flexible */
    uint16_t tcb_count = 0;
    uint16_t total_len = 0;
    bool tx_success = true;
    uint16_t tx_status = 0;

    cmd = get_uint16(addr + 2);
    assert(cmd & 0x0008);    /* check flexible mode bit (CMD_FLEX) - must be set in 32-bit modes */
    sf_bit = (cmd & 0x02) ? 1 : 0;
    insert_crc = (I596_NOCRC_INS == 0) && ((cmd & 0x10) == 0) && !I596_LOOPBACK;
    
    tcb_count = get_uint16(addr + 12) & 0x3FFF;
    
    printf("DEBUG: TCB addr=0x%08x, raw_count=0x%04x, masked_count=%d\n", 
           addr, get_uint16(addr + 12), tcb_count);
    
    if (tcb_count > sizeof(s->tx_buffer)) {
        printf("WARNING: TCB count %d exceeds buffer size %zu, truncating\n", 
               tcb_count, sizeof(s->tx_buffer));
        tcb_count = sizeof(s->tx_buffer);
    }
    
    /* Get TBD pointer */
    tbd_p = get_uint32(addr + 8);
    
    if (tcb_count > 0 && tcb_count <= 1518) {
        /* Data in the TCB starts at offset 16 */
        address_space_rw(&address_space_memory, addr + 16,
            MEMTXATTRS_UNSPECIFIED, s->tx_buffer, tcb_count, 0);
        total_len = tcb_count;
        
        trace_i82596_transmit(tcb_count, addr + 16);
        printf("DEBUG: Read %d bytes from TCB data area\n", tcb_count);
    } else if (tcb_count > 0) {
        printf("WARNING: TCB count %d is too large, ignoring TCB data\n", tcb_count);
        tcb_count = 0;  /* Don't use TCB data */
    }

    if (sf_bit == 0) {
        uint16_t tcb_value = get_uint16(addr + 16);
        uint16_t eof_bit = tcb_value & 0x8000;
        
        if (!eof_bit) {
            DBG(printf("WARNING: EOF bit not set in simplified mode, fixing automatically\n"));
            /* Set the EOF bit in the TCB count field */
            tcb_value |= 0x8000;
            set_uint16(addr + 12, tcb_value);
        }
    }
    /* process TBD in flexible mode or if TCB_COUNT=0 */
    while (tbd_p != I596_NULL) {
        uint16_t size, len;
        uint32_t tba;

        size = get_uint16(tbd_p);
        len = size & SIZE_MASK;
        tba = get_uint32(tbd_p + 8);
        trace_i82596_transmit(len, tba);

        if (s->nic && len) {
            if (total_len + len > sizeof(s->tx_buffer)) {
                printf("WARNING: TBD length %d would exceed buffer size (total=%d, buffer=%zu), truncating\n", 
                       len, total_len, sizeof(s->tx_buffer));
                len = sizeof(s->tx_buffer) - total_len;
                if (len <= 0) {
                    printf("WARNING: Buffer completely full, stopping TBD processing\n");
                    tx_success = false;
                    tx_status |= (1 << 8); /* DMA underrun */
                    break;
                }
            }
            address_space_rw(&address_space_memory, tba,
                MEMTXATTRS_UNSPECIFIED, s->tx_buffer + total_len, len, 0);
            total_len += len;
            DBG(PRINT_PKTHDR("Send", &s->tx_buffer[total_len - len]));
        }

        /* was this the last package? */
        if (size & I596_EOF) {
            break;
        }

        /* get pointer to next TBD */
        tbd_p = get_uint32(tbd_p + 4);
    }
    
    /* If we have data to send */
    if (s->nic && total_len > 0 && tx_success) {
        if (I596_NO_SRC_ADD_IN == 0) {
            memcpy(&s->tx_buffer[ETH_ALEN], s->conf.macaddr.a, ETH_ALEN);
        }

        DBG(printf("i82596_transmit: insert_crc = %d  insert SRC = %d\n",
                    insert_crc, I596_NO_SRC_ADD_IN == 0));
        
        /* Add CRC if needed */
        if (insert_crc) {
            uint32_t crc = crc32(~0, s->tx_buffer, total_len);
            crc = cpu_to_be32(crc);
            if (total_len + sizeof(crc) > sizeof(s->tx_buffer)) {
                printf("WARNING: Cannot add CRC, would exceed buffer size (total=%d, buffer=%zu)\n", 
                       total_len, sizeof(s->tx_buffer));
                tx_success = false;
                tx_status |= (1 << 8); /* DMA underrun */
            } else {
                memcpy(&s->tx_buffer[total_len], &crc, sizeof(crc));
                total_len += sizeof(crc);
            }
        }

        if (tx_success) {
            DBG(PRINT_PKTHDR("Send", &s->tx_buffer));
            DBG(printf("Sending %d bytes (crc_inserted=%d)\n", total_len, insert_crc));
            
            /* Handle loopback modes */
            switch (I596_LOOPBACK) {
            case 0:     /* no loopback, send packet */
                qemu_send_packet_raw(qemu_get_queue(s->nic), s->tx_buffer, total_len);
                break;
            default:    /* all other loopback modes: ignore! */
            case 1:     /* external loopback enabled */
                i82596_receive(qemu_get_queue(s->nic), s->tx_buffer, total_len);
                break;
            }
            
            qemu_flush_queued_packets(qemu_get_queue(s->nic));
        }
    }
    
    /* Update transmission statistics */
    i82596_update_tx_stats(s, tx_success, tx_status);
    
    uint16_t final_status = STAT_C;
    if (tx_success) {
        final_status |= STAT_OK;
    }
    final_status |= tx_status; /* Include error bits, right? */
    set_uint16(addr, final_status);
}

static void set_individual_address(I82596State *s, uint32_t addr)
{
    NetClientState *nc;
    uint8_t *m;

    nc = qemu_get_queue(s->nic);
    m = s->conf.macaddr.a;
    address_space_rw(&address_space_memory, addr + 8,
                        MEMTXATTRS_UNSPECIFIED, m, ETH_ALEN, 0);
    qemu_format_nic_info_str(nc, m);
    DBG(printf("MAC addr set to %s\n", nc->info_str));
    trace_i82596_new_mac(nc->info_str);
}

static void set_multicast_list(I82596State *s, uint32_t addr)
{
    uint16_t mc_count, i;

    memset(&s->mult[0], 0, sizeof(s->mult));
    mc_count = get_uint16(addr + 8) / ETH_ALEN;
    addr += 10;
    if (mc_count > MAX_MC_CNT) {
        mc_count = MAX_MC_CNT;
    }
    DBG(printf("Add %d multicast entries.\n", mc_count));
    for (i = 0; i < mc_count; i++) {
        uint8_t multicast_addr[ETH_ALEN];
        address_space_rw(&address_space_memory,
            addr + i * ETH_ALEN, MEMTXATTRS_UNSPECIFIED,
            multicast_addr, ETH_ALEN, 0);
        DBG(printf("Add multicast entry " MAC_FMT "\n",
                    MAC_ARG(multicast_addr)));
        unsigned mcast_idx = (net_crc32(multicast_addr, ETH_ALEN) &
                              BITS(7, 2)) >> 2;
        assert(mcast_idx < 8 * sizeof(s->mult));
        s->mult[mcast_idx >> 3] |= (1 << (mcast_idx & 7));
    }
    trace_i82596_set_multicast(mc_count);
}

void i82596_set_link_status(NetClientState *nc)
{
    I82596State *d = qemu_get_nic_opaque(nc);

    d->lnkst = nc->link_down ? 0 : 0x8000;
}

static void update_scb_status(I82596State *s)
{
    s->scb_status = (s->scb_status & 0xf000) | (s->CUS << 8) | (s->RUS << 4) | 8 /* 8: bus throttle timers loaded */;
    DBG(printf("update_scb_status 0x%04x CUS: %d, RUS: %d\n", s->scb_status, s->CUS, s->RUS));
    set_uint16(s->scb, s->scb_status);
}


static void i82596_s_reset(I82596State *s)
{
    trace_i82596_s_reset(s);
    DBG(printf("i82596_s_reset()\n"));
    s->scp = 0x00FFFFF4;
    s->scb_status = 0;
    s->CUS = CU_IDLE;
    s->RUS = RX_IDLE;
    s->cmd_p = I596_NULL;
    s->lnkst = 0x8000; /* initial link state: up */
    s->send_irq = 0;
    memset(&s->stats, 0, sizeof(s->stats));
}

static void i82596_perform_dump(I82596State *s, uint32_t dump_addr, bool is_port_dump)
{
    I82596DumpArea dump_data = {0};  /* Zero-initialize entire structure */
    
    printf("i82596 dump to address 0x%08x (size=%zu bytes)\n", 
           dump_addr, sizeof(I82596DumpArea));
    
    /* Configuration Area */
    memcpy(dump_data.config_bytes, s->config, 
           MIN(sizeof(dump_data.config_bytes), sizeof(s->config)));
    
    /* Individual Address */
    memcpy(dump_data.individual_addr, s->conf.macaddr.a, ETH_ALEN);
    
    dump_data.crc_errors = cpu_to_le32(s->stats.crc_errors);
    dump_data.alignment_errors = cpu_to_le32(s->stats.alignment_errors);
    dump_data.resource_errors = cpu_to_le32(s->stats.resource_errors);
    dump_data.overrun_errors = cpu_to_le32(s->stats.overrun_errors);
    
    /* Multicast Hash Register */
    memcpy(dump_data.multicast_hash, s->mult, sizeof(dump_data.multicast_hash));
    
    /* Current Status */
    dump_data.cu_ru_status = cpu_to_le16((s->CUS << 8) | (s->RUS & 0xFF));
    dump_data.link_status = cpu_to_le16(s->lnkst);
    dump_data.scb_address = cpu_to_le32(s->scb);
    dump_data.current_cmd_ptr = cpu_to_le32(s->cmd_p);
    dump_data.bus_control = cpu_to_le32(0x01010101);  /* Standard bus control value */
    
    /* System Configuration */
    dump_data.sysbus_config = cpu_to_le32(s->sysbus);
    dump_data.additional_control = cpu_to_le32(0x00000001);
    
    address_space_write(&address_space_memory, dump_addr, 
                       MEMTXATTRS_UNSPECIFIED, (uint8_t*)&dump_data, 
                       sizeof(I82596DumpArea));
    
    /* In case of PORT_ALTDUMP, write completion status word */
    if (is_port_dump) {
        uint16_t status_word = cpu_to_le16(STAT_C | STAT_OK); /* 0xA000 */
        address_space_write(&address_space_memory, dump_addr + sizeof(I82596DumpArea),
                           MEMTXATTRS_UNSPECIFIED, (uint8_t*)&status_word, 
                           sizeof(status_word));
        printf("i82596 PORT_ALTDUMP completed: status=0x%04x at offset 0x%zx\n", 
               STAT_C | STAT_OK, sizeof(I82596DumpArea));
    } else {
        printf("i82596 CmdDump completed\n");
    }
}

static void i82596_perform_tdr(I82596State *s, uint32_t addr)
{
    uint32_t tdr_addr = addr + 8;
    uint16_t tdr_time;
    uint16_t tdr_status = 0;
    /* 
    * Note: Loopback is not implemented in the Linux driver
    * Other drivers such as that of HPUX 10.20 use the loopback mode.
    */
    if (s->lnkst && I596_LOOPBACK) {
        /* loopback mode, report link as OK with a small echo time */
        tdr_status = 0x8000;
        tdr_time = 6;
    } else {
        if (I596_LOOPBACK == 3) { /* External loopback mode */
            tdr_status = 0x4000;
            tdr_time = 0x7FF;
        } else if (I596_LOOPBACK == 1) { /* Internal loopback mode */
            tdr_status = 0x1000;
            tdr_time = 0x20;
        } else {
            tdr_status = 0x2000;
            tdr_time = 0x3A0;
        }
    }
    /* Write TDR results back to the memory */
    tdr_time &= 0x7FF;
    set_uint16(tdr_addr, tdr_status);
    set_uint16(tdr_addr + 2, tdr_time);
    DBG(printf("TDR Command Completed: status=0x%04x, time=0x%04x\n", 
              tdr_status, tdr_time));
    s->scb_status |= SCB_STATUS_CX;
}

static void i82596_perform_diagnose(I82596State *s, uint32_t addr)
{
    /* We perform diagnosis in 3 phases accurate to the docs:
    * Phase 1: Test Statistical Counters
    * Phase 2: Test Exponential Backoff Logic
    * Phase 3: Test Timer Logic
    * Phase 4: Test Configuration Validity
    * If any test fails, we set the F bit in the status word.
    * If all tests pass, we set the OK bit.
    * Sounds good, lets go!
    */
    bool diagnose_passed = true;
    uint16_t diagnose_status = STAT_C | STAT_OK;
    printf("i82596 Diagnose: Running internal self-test\n");
    
    /* Phase 1: Test Statistical Counters */
    printf("i82596 Diagnose: Testing statistical counters...\n");
    
    uint32_t saved_crc = s->stats.crc_errors;
    uint32_t saved_align = s->stats.alignment_errors;
    uint32_t saved_resource = s->stats.resource_errors;
    uint32_t saved_overrun = s->stats.overrun_errors;
    uint32_t saved_collisions = s->stats.collisions;
    uint32_t saved_tx = s->stats.frames_transmitted;
    uint32_t saved_rx = s->stats.frames_received;
    
    s->stats.crc_errors = 0x12345678;
    s->stats.alignment_errors = 0x87654321;
    s->stats.resource_errors = 0xAAAAAAAA;
    s->stats.overrun_errors = 0x55555555;
    
    if (s->stats.crc_errors != 0x12345678 ||
        s->stats.alignment_errors != 0x87654321 ||
        s->stats.resource_errors != 0xAAAAAAAA ||
        s->stats.overrun_errors != 0x55555555) {
        printf("i82596 Diagnose: FAILED - Counter write/read test\n");
        diagnose_passed = false;
    }
    
    s->stats.crc_errors = 0xFFFFFFFF;
    s->stats.crc_errors++;
    if (s->stats.crc_errors != 0) {
        printf("i82596 Diagnose: FAILED - Counter wraparound test\n");
        diagnose_passed = false;
    }
    
    /* Phase 2: Test Exponential Backoff Logic */
    printf("i82596 Diagnose: Testing exponential backoff logic...\n");
    
    for (int collision = 0; collision < 16; collision++) {
        uint32_t backoff_slots = 1 << MIN(collision, 10); /* No idea, revert if not working?*/
        uint32_t random_factor = (qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) >> 10) & (backoff_slots - 1);
        if (random_factor >= backoff_slots) {
            printf("i82596 Diagnose: FAILED - Backoff calculation error at collision %d\n", collision);
            diagnose_passed = false;
            break;
        }
    }
    
    /* Phase 3: Test Timer Logic */
    printf("i82596 Diagnose: Testing timer logic...\n");
    
    uint16_t slot_time = (s->config[7] & 0x07) << 8 | s->config[6];
    if (slot_time == 0) {
        slot_time = 0x200; /* Default 512 bit times */
    }
    if (slot_time < 0x20 || slot_time > 0x7FF) {
        printf("i82596 Diagnose: WARNING - Unusual slot time value: 0x%04x\n", slot_time);
    }
    uint8_t ifs = s->config[5];
    if (ifs < 12) {
        printf("i82596 Diagnose: WARNING - IFS too small: %d (minimum 12)\n", ifs);
    }
    
    printf("i82596 Diagnose: Testing configuration validity...\n");
    
    uint8_t addr_len = s->config[3] & 0x07;
    if (addr_len != 6 && addr_len != 0) {
        printf("i82596 Diagnose: WARNING - Non-standard address length: %d\n", addr_len);
    }
    
    uint8_t fifo_limit = s->config[1] & 0x0F;
    if (fifo_limit < 8 || fifo_limit > 14) {
        printf("i82596 Diagnose: WARNING - Unusual FIFO limit: %d\n", fifo_limit);
    }
    
    s->stats.crc_errors = saved_crc;
    s->stats.alignment_errors = saved_align;
    s->stats.resource_errors = saved_resource;
    s->stats.overrun_errors = saved_overrun;
    s->stats.collisions = saved_collisions;
    s->stats.frames_transmitted = saved_tx;
    s->stats.frames_received = saved_rx;
    
    if (!diagnose_passed) {
        diagnose_status = STAT_C; /* Clear OK bit to indicate failure */
        diagnose_status |= (1 << 11); /* Set F bit - self-test failed */
        printf("i82596 Diagnose: FAILED - One or more tests failed :(\n");
    } else {
        printf("i82596 Diagnose: PASSED - All tests completed successfully :) Time to party!\n");
    }
    
    /* Update command status */
    set_uint16(addr, diagnose_status);
    s->scb_status |= SCB_STATUS_CX;
}

static void command_loop(I82596State *s)
{
    uint16_t cmd;
    uint16_t status;
    uint8_t byte_cnt;

    DBG(printf("STARTING COMMAND LOOP cmd_p=0x%08x\n", s->cmd_p));

    while (s->cmd_p != I596_NULL) {
        status = STAT_B;
        set_uint16(s->cmd_p, status);
        status = STAT_C | STAT_OK;

        cmd = get_uint16(s->cmd_p + 2);
        DBG(printf("Running command 0x%04x (cmd %d) at 0x%08x\n",
                cmd, cmd & 7, s->cmd_p));

        switch (cmd & 0x07) {
        case CmdNOp:
            break;
        case CmdSASetup:
            set_individual_address(s, s->cmd_p);
            break;
        case CmdConfigure:
            /* NEEDS MORE THROUGH DETAILS WITH THE DOCUMENTATION*/
            byte_cnt = get_byte(s->cmd_p + 8) & 0x0f;
            byte_cnt = MAX(byte_cnt, 4);
            byte_cnt = MIN(byte_cnt, sizeof(s->config));
            address_space_rw(&address_space_memory, s->cmd_p + 8,
                MEMTXATTRS_UNSPECIFIED, s->config, byte_cnt, 0);
            s->config[2] &= 0x82;
            s->config[2] |= 0x40;
            DBG(printf("I596_CONFIG3 = 0x%02x  LOOPBACK 0x%x\n", s->config[3], I596_LOOPBACK));
            if (I596_NO_SRC_ADD_IN == 0) {
                assert((s->config[3] & 0x07) == ETH_ALEN);
            }
            s->config[7]  &= 0xf7;
            assert(I596_CRC16_32 == 0);
            DBG(printf("I596_CRCINM = %d\n\n", I596_CRCINM));
            s->config[10] = MAX(s->config[10], 5);
            s->config[12] &= 0x40;
            s->config[13] |= 0x3f;
            s->scb_status |= SCB_STATUS_CX;
            break;
        case CmdTDR:
            i82596_perform_tdr(s, s->cmd_p);
            break;
        case CmdTx:
            i82596_transmit(s, s->cmd_p);
            break;
        case CmdMulticastList:
            set_multicast_list(s, s->cmd_p);
            break;
        case CmdDump:
            uint32_t dump_addr = get_uint32(s->cmd_p + 8);
            i82596_perform_dump(s, dump_addr, false);
            status |= STAT_OK;
            s->scb_status |= SCB_STATUS_CX;
            break;
        case CmdDiagnose:
            i82596_perform_diagnose(s, s->cmd_p);
            status = get_uint16(s->cmd_p); /* Get the status set by diagnose */
            printf("i82596 Diagnose command completed\n");
            break;
        }

        /* Update status (Exclusing diagnose which sets its own status) */
        if ((cmd & 0x07) != CmdDiagnose) {
            set_uint16(s->cmd_p, status);
        }

        s->cmd_p = get_uint32(s->cmd_p + 4);
        DBG(printf("NEXT loop addr is 0x%08x\n", s->cmd_p));
        if (s->cmd_p == 0) {
            s->cmd_p = I596_NULL;
        }

        if (cmd & CMD_EOL) {
            s->cmd_p = I596_NULL;
        }
        if (cmd & CMD_SUSP) {
            s->CUS = CU_SUSPENDED;
            printf("FIXME SUSPEND ?\n");
        }

        if (cmd & CMD_INTR) {
            s->scb_status |= SCB_STATUS_CX;
            s->send_irq = 1;
        }

        if (s->CUS == CU_SUSPENDED) {
            break;
        }
    }
    DBG(printf("FINISHED COMMAND LOOP\n"));
    qemu_flush_queued_packets(qemu_get_queue(s->nic));
}

static void i82596_flush_queue_timer(void *opaque)
{
    I82596State *s = opaque;
    if (0) {
        timer_del(s->flush_queue_timer);
        qemu_flush_queued_packets(qemu_get_queue(s->nic));
        timer_mod(s->flush_queue_timer,
              qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 1000);
    }
}

static void examine_scb(I82596State *s)
{
    uint16_t command, cuc, ruc;

    /* get the scb command word */
    command = get_uint16(s->scb + 2);
    DBG(printf("COMMAND = 0x%04x\n", command));
    cuc = (command >> 8) & 0x7;
    ruc = (command >> 4) & 0x7;
    DBG(printf("MAIN CU COMMAND 0x%04x: stat 0x%02x cuc 0x%02x ruc 0x%02x\n",
            command, command >> 12,  cuc, ruc));

    /* toggle the STAT flags in SCB status word */
    s->scb_status &= ~(command & SCB_COMMAND_ACK_MASK);

    switch (cuc) {
    case 0:     /* no change */
    case 5:
    case 6:
        break;
    case 1:     /* CUC_START */
        s->CUS = CU_ACTIVE;
        break;
    case 4:     /* CUC_ABORT */
        s->CUS = CU_IDLE;
        s->scb_status |= SCB_STATUS_CNA; /* CU left active state */
        s->send_irq = 1;
        break;
    default:
        printf("WARNING: Unknown CUC %d!\n", cuc);
    }

    switch (ruc) {
    case 0:     /* no change */
        break;
    case 1:     /* RX_START */
    case 2:     /* RX_RESUME */
        s->RUS = RX_READY;
        break;
    case 3:     /* RX_SUSPEND */
        s->RUS = RX_SUSPENDED;
        s->scb_status |= SCB_STATUS_RNR; /* RU left active state */
        s->send_irq = 1;
        break;
    case 4:     /* RX_ABORT */
        s->RUS = RX_IDLE;
        s->scb_status |= SCB_STATUS_RNR; /* RU left active state */
        s->send_irq = 1;
        break;
    default:
        printf("WARNING: Unknown RUC %d!\n", ruc);
    }

    if (command & 0x80) { /* reset bit set? */
        i82596_s_reset(s);
    }

    /* execute commands from SCBL */
    if (s->CUS == CU_ACTIVE) {
        if (s->cmd_p == I596_NULL) {
            s->cmd_p = get_uint32(s->scb + 4);
        }
        command_loop(s);
        s->CUS = CU_IDLE;
        s->send_irq = 1;
    }

    qemu_flush_queued_packets(qemu_get_queue(s->nic));
}

static void signal_ca(I82596State *s)
{
    DBG(printf("-- CA start\n"));

    /* trace_i82596_channel_attention(s); */
    if (s->scp) {
        uint32_t iscp;
        
        /* CA after reset -> do init with new scp. */
        s->sysbus = get_byte(s->scp + 3); /* big endian byte location */
        DBG(printf("SYSBUS = %08x\n", s->sysbus));
        
        if (((s->sysbus >> 1) & 0x03) != 2) {
            DBG(printf("INFO: Not in Linear Mode (mode=%d)\n", ((s->sysbus >> 1) & 0x03)));
        }
        
        if ((s->sysbus >> 7) & 1) {
            DBG(printf("INFO: Enhanced Big Endian Mode enabled\n"));
        }
        
        iscp = get_uint32(s->scp + 8);
        
        s->scb = get_uint32(iscp + 4);
        DBG(printf("ISCP = 0x%08x, SCB = 0x%08x\n", iscp, s->scb));
        
        set_byte(iscp + 1, 0);
        

        s->scb_status = SCB_STATUS_CX | SCB_STATUS_CNA;
        s->CUS = CU_IDLE;
        s->RUS = RX_IDLE;
        s->scp = 0;
        s->send_irq = 1;
        goto _cont;
    }

    /* For subsequent CAs, examine the SCB */
    examine_scb(s);

_cont:
    update_scb_status(s);

    set_uint16(s->scb + 2, 0);

    if (s->send_irq) {
        s->send_irq = 0;
        DBG(printf("Send IRQ\n"));
        qemu_set_irq(s->irq, 1);
    }
    DBG(printf("-- CA end\n"));
}

// static void i82596_ioport_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
// {
//     i82596_ioport_writew(opaque, (uint32_t)addr, (uint32_t)val);
// }

// static uint64_t i82596_ioport_read(void *opaque, hwaddr addr, unsigned size)
// {
//     return i82596_ioport_readw(opaque, (uint32_t)addr);
// }

// static const MemoryRegionOps i82596_ioport_ops = {
//     .write = i82596_ioport_write,
//     .read = i82596_ioport_read,
//     .endianness = DEVICE_LITTLE_ENDIAN,
//     .valid = {
//         .min_access_size = 4,
//         .max_access_size = 4,
//     },
// };

void i82596_ioport_writew(void *opaque, uint32_t addr, uint32_t val)
{
    I82596State *s = opaque;
    uint32_t res, tmp;
    uint32_t port_cmd = addr & PORT_BYTEMASK;
    
    printf("i82596_ioport_writew addr=0x%08x val=0x%08x, port_cmd=%d\n", 
           addr, val, port_cmd);
    
    switch (port_cmd) {
    case PORT_RESET: /* Reset */
        printf("i82596 PORT_RESET\n");
        i82596_s_reset(s);
        break;
        
    case PORT_SELFTEST:
        printf("i82596 PORT_SELFTEST at 0x%08x\n", val);
        /* Selftest: check if address+4 contains 0xFFFFFFFF, then set it to 0 */
        res = val + sizeof(uint32_t);
        tmp = get_uint32(res);
        printf("i82596 SELFTEST: checking address 0x%08x, expected 0xFFFFFFFF, got 0x%08x\n", res, tmp);
        if (tmp == I596_NULL) {
            set_uint32(res, 0);
            printf("i82596 SELFTEST completed successfully\n");
        } else {
            printf("i82596 SELFTEST failed - wrong initial value\n");
        }
        break;
        
    case PORT_ALTSCP:
        printf("i82596 PORT_ALTSCP: setting SCP to 0x%08x\n", val);
        s->scp = val;
        break;
        
    case PORT_ALTDUMP:
        printf("i82596 PORT_ALTDUMP: dumping to address 0x%08x\n", val);
        if (val & 0xF) {
            printf("i82596 PORT_ALTDUMP: Warning - dump area not 16-byte aligned (0x%08x)\n", val);
        }
        i82596_perform_dump(s, val & ~0xF, true);
        break;
        
    case PORT_CA:
        printf("i82596 PORT_CA: Channel Attention\n");
        signal_ca(s);
        break;
        
    default:
        printf("i82596: Unknown port command 0x%x (addr=0x%08x, val=0x%08x)\n", 
               port_cmd, addr, val);
        break;
    }
}

uint32_t i82596_ioport_readw(void *opaque, uint32_t addr)
{
    return -1;
}

void i82596_h_reset(void *opaque)
{
    I82596State *s = opaque;

    i82596_s_reset(s);
}

bool i82596_can_receive(NetClientState *nc)
{
    I82596State *s = qemu_get_nic_opaque(nc);

    if (s->RUS == RX_READY) {
        return false;
    }

    if (!s->lnkst) {
        return false;
    }

    if (USE_TIMER && !timer_pending(s->flush_queue_timer)) {
        return true;
    }

    return true;
}

ssize_t i82596_receive(NetClientState *nc, const uint8_t *buf, size_t sz)
{
    I82596State *s = qemu_get_nic_opaque(nc);
    uint32_t rfd_p;
    uint32_t rbd;
    uint16_t status, is_broadcast = 0;
    size_t len = sz;
    uint32_t crc;
    uint8_t *crc_ptr;
    uint8_t min_frame_buffer[MIN_BUF_SIZE + 4]; /* +4 for CRC bytes */
    static const uint8_t broadcast_macaddr[6] = {
                0xff, 0xff, 0xff, 0xff, 0xff, 0xff };

    DBG(printf("i82596_receive() start, sz = %lu\n", sz));

    /* first check if receiver is enabled */
    if (s->RUS == RX_SUSPENDED) {
        trace_i82596_receive_analysis(">>> Receiving suspended");
        return -1;
    }

    if (!s->lnkst) {
        trace_i82596_receive_analysis(">>> Link down");
        return -1;
    }

    /* Received frame smaller than configured "min frame len"? */
    if (sz < s->config[10]) {
        if (0) printf("Received frame too small, %lu vs. %u bytes\n",
            sz, s->config[10]);
        sz = 60; /* return -1; */
    }

    DBG(printf("Received %lu bytes\n", sz));

    if (I596_PROMISC || I596_LOOPBACK) {
        /* In promiscuous: we receive all */
        trace_i82596_receive_analysis(
                ">>> packet received in promiscuous mode");
    } else {
        if (!memcmp(buf,  broadcast_macaddr, 6)) {
            /* broadcast address */
            if (I596_BC_DISABLE) {
                trace_i82596_receive_analysis(">>> broadcast packet rejected");

                return len;
            }

            trace_i82596_receive_analysis(">>> broadcast packet received");
            is_broadcast = 1;

        } else if (buf[0] & 0x01) {
            /* multicast */
            if (!I596_MC_ALL) {
                trace_i82596_receive_analysis(">>> multicast packet rejected");

                return len;
            }

            int mcast_idx = (net_crc32(buf, ETH_ALEN) & BITS(7, 2)) >> 2;
            assert(mcast_idx < 8 * sizeof(s->mult));

            if (!(s->mult[mcast_idx >> 3] & (1 << (mcast_idx & 7)))) {
                trace_i82596_receive_analysis(">>> multicast address mismatch");

                return len;
            }

            trace_i82596_receive_analysis(">>> multicast packet received");
            is_broadcast = 1;

        } else if (!memcmp(s->conf.macaddr.a, buf, 6)) {

            /* match */
            trace_i82596_receive_analysis(
                    ">>> physical address matching packet received");

        } else {

            trace_i82596_receive_analysis(">>> unknown packet");

            return len;
        }
    }

    /* if too small buffer, then expand it to minimum Ethernet frame size */
    if (len < MIN_BUF_SIZE) {
        memcpy(min_frame_buffer, buf, len);
        memset(min_frame_buffer + len, 0, MIN_BUF_SIZE - len);
        buf = min_frame_buffer;
        len = MIN_BUF_SIZE;
    }

    /* Calculate the ethernet checksum (4 bytes) */
    if (I596_CRCINM && !I596_LOOPBACK) {
        len += 4;
        crc = crc32(~0, buf, sz);
        crc = cpu_to_be32(crc);
        crc_ptr = (uint8_t *) &crc;
    }

    /* Find an available Receive Frame Descriptor */
    rfd_p = get_uint32(s->scb + 8);
    do {
        assert(rfd_p && rfd_p != I596_NULL);
        status = get_uint16(rfd_p+0);
        /* if rfd is filled, get next one from link addr */
        if (status & STAT_OK)
            rfd_p = get_uint32(rfd_p+4);
    } while (status & STAT_OK);

    trace_i82596_receive_packet(len);
    DBG(PRINT_PKTHDR("Receive", buf));

    while (len) {
        uint16_t command;
        uint32_t next_rfd;
        uint32_t rba;
        uint16_t rba_size;
        uint32_t actual_count_ptr;

        DBG(printf("Receive: rfd is 0x%08x, len = %lu\n", rfd_p, len));
        command = get_uint16(rfd_p + 2);
        assert(command & CMD_FLEX); /* assert Flex Mode, according to docs */

        DBG(printf("Receive: EL= %d, S(uspend) = %d\n", 
             (command & CMD_EOL)?1:0, (command & CMD_SUSP)?1:0));

        /* get first Receive Buffer Descriptor Address */
        rbd = get_uint32(rfd_p + 8);
        assert(rbd && rbd != I596_NULL);

        rba = rfd_p + 16;       /* data is behind the length field */
        rba_size = get_uint16(rfd_p + 14); /* count of additional bytes in rfd */
        actual_count_ptr = rfd_p + 12;

        while (len) {
            uint16_t num, actual_count;

            DBG(printf("rba is at 0x%x, rba_size = %d, cnt_ptr 0x%08x\n", 
                 rba, rba_size, actual_count_ptr));

            /* store number of received bytes first */
            num = rba_size & SIZE_MASK;
            if (num > len) {
                num = len;
            }
            actual_count = num;
            if (num == len) {
                actual_count |= I596_EOF; /* set EOF BIT */
            }

            if (num) {
                actual_count |= 0x4000; /* set F BIT - frame received without error */
                set_uint16(actual_count_ptr, actual_count); /* write actual count with flags */
            
                if (rba & 0x3) {
                    /* For unaligned addresses, we copy data in a way that preserves alignment */
                    uint8_t aligned_buffer[PKT_BUF_SZ_MAX] __attribute__((aligned(4)));
                    memcpy(aligned_buffer, (void *)buf, num);
                    address_space_rw(&address_space_memory, rba & ~0x3,
                        MEMTXATTRS_UNSPECIFIED, aligned_buffer, (num + (rba & 0x3) + 3) & ~0x3, 1);
                } else {
                    address_space_rw(&address_space_memory, rba,
                        MEMTXATTRS_UNSPECIFIED, (void *)buf, num, 1);
                }
            }
            rba += num;
            buf += num;
            len -= num;
            if (len == 0 && I596_CRCINM && !I596_LOOPBACK) { /* copy crc */
                address_space_rw(&address_space_memory, rba - 4,
                    MEMTXATTRS_UNSPECIFIED, crc_ptr, 4, 1);
            }

            if (len == 0) { // do not get next rbd
                break;
            }

            if (rba_size & I596_EOF) /* last entry */
                break;

            DBG(printf("Receive: rbd is 0x%08x\n", rbd));
            rba_size = get_uint16(rbd + 12);
            rba = get_uint32(rbd + 8);
            actual_count_ptr = rbd + 0;
            assert(rba_size != 0);

            /* get next rbd */
            rbd = get_uint32(rbd + 4);
        }

        /* Housekeeping, see pg. 18 of 82596CA documentation */
        next_rfd = get_uint32(rfd_p + 4);
        assert(next_rfd && next_rfd != I596_NULL);
        set_uint32(next_rfd + 8, rbd);

        status = STAT_C | STAT_OK | is_broadcast;
        set_uint16(rfd_p, status);

        if (command & CMD_SUSP) {  /* suspend after command? */
            s->RUS = RX_SUSPENDED;
            s->scb_status |= SCB_STATUS_RNR; /* RU left active state */
        }
        if (command & CMD_EOL) /* was it last Frame Descriptor? */
            break;

        assert(len == 0);
    }

    assert(len == 0);

    s->scb_status |= SCB_STATUS_FR; /* set "RU finished receiving frame" bit. */
    update_scb_status(s);

    i82596_update_rx_stats(s, true, status, sz);
    /* send IRQ that we received data */
    qemu_set_irq(s->irq, 1);

    DBG(printf("i82596_receive() end sz = %lu\n", sz));
    return sz;
}


const VMStateDescription vmstate_i82596 = {
    .name = "i82596",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16(lnkst, I82596State),
        VMSTATE_TIMER_PTR(flush_queue_timer, I82596State),
        VMSTATE_END_OF_LIST()
    }
};

void i82596_common_init(DeviceState *dev, I82596State *s, NetClientInfo *info)
{
    if (s->conf.macaddr.a[0] == 0) {
        qemu_macaddr_default_if_unset(&s->conf.macaddr);
    }
    s->nic = qemu_new_nic(info, &s->conf, object_get_typename(OBJECT(dev)),
                dev->id, &dev->mem_reentrancy_guard, s);
    qemu_format_nic_info_str(qemu_get_queue(s->nic), s->conf.macaddr.a);

    if (USE_TIMER) {
        s->flush_queue_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                    i82596_flush_queue_timer, s);
    }
    s->lnkst = 0x8000; /* initial link state: up */
    
    // memory_region_init_io(&s->ioport, OBJECT(dev), &i82596_ioport_ops, s,
    //                      "i82596-ioport", I82596_IOPORT_SIZE);
}
