/*
 * QEMU Intel i82596 (Apricot) emulation
 *
 * Copyright (c) 2019 Helge Deller <deller@gmx.de>
 * Actively being developed by Soumyajyotii Ssarkar <soumyajyotisarkar23@gmail.com>
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
 #include "exec/address-spaces.h"
 #include "qemu/module.h"
 #include "trace.h"
 #include "i82596.h"
 #include <zlib.h> /* for crc32 */
 
 #define ENABLE_DEBUG 1
 
 #if defined(ENABLE_DEBUG)
 #define DBG(x)          x
 #else
 #define DBG(x)          do { } while (0)
 #endif
 
 #define USE_TIMER       0
 
 #define BITS(n, m) (((0xffffffffU << (31 - n)) >> (31 - n + m)) << m)
 
 #define PKT_BUF_SZ      1536
 #define MAX_MC_CNT      64
 #define MIN_BUF_SIZE    60
 
 #define ISCP_BUSY       0x0001
 
 #define I596_NULL       ((uint32_t)0xffffffff)
 
 #define SCB_STATUS_CX   0x8000 /* CU finished command with I bit */
 #define SCB_STATUS_FR   0x4000 /* RU finished receiving a frame */
 #define SCB_STATUS_CNA  0x2000 /* CU left active state */
 #define SCB_STATUS_RNR  0x1000 /* RU left active state */
 
 #define SCB_COMMAND_ACK_MASK \
         (SCB_STATUS_CX | SCB_STATUS_FR | SCB_STATUS_CNA | SCB_STATUS_RNR)
 
 #define CU_IDLE         0
 #define CU_SUSPENDED    1
 #define CU_ACTIVE       2
 
 #define RX_IDLE         0
 #define RX_SUSPENDED    1
 #define RX_NO_RESOURCES 2
 #define RX_READY        4
 
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

 #define ETHER_TYPE_LEN 2       /* Standard Ethernet type/length field size */
 #define ETHER_MAC_ADDR_LEN 6   /* MAC address length */
 #define ETHER_HDR_LEN 14       /* 6(dst) + 6(src) + 2(type/len) */
 #define ETHER_CRC_LEN 4        /* FCS/CRC field length */
 #define ETHER_MIN_LEN 60       /* Minimum frame body without CRC */

 /* various flags in the chip config registers */
 #define I596_PREFETCH      (s->config[0] & 0x80)
 #define I596_NO_SRC_ADD_IN (s->config[3] & 0x08) /* if 1, do not insert MAC in Tx Packet */
 #define I596_LOOPBACK      (s->config[3] >> 6)     /* loopback mode, 3 = external loopback */
 #define I596_PROMISC       (s->config[8] & 0x01)
 #define I596_BC_DISABLE    (s->config[8] & 0x02)   /* broadcast disable */
 #define I596_NOCRC_INS     (s->config[8] & 0x08)   /* do not append CRC to Tx frame */
 #define I596_CRC16_32      (s->config[8] & 0x10)   /* CRC-16 or CRC-32 */
 #define I596_CRCINM        (s->config[11] & 0x04) /* Rx CRC appended in memory */
 #define I596_MC_ALL        (s->config[11] & 0x20)
 #define I596_MULTIIA       (s->config[13] & 0x40) 
 
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
 
 static void i82596_transmit(I82596State *s, uint32_t addr)
 {
     uint32_t tdb_p; /* Transmit Buffer Descriptor */
     uint16_t cmd, tcb_count = 0;
     int insert_crc;
     
     cmd = get_uint16(addr + 2);
     int sf_mode = (cmd & CMD_FLEX) ? 0 : 1; /* 0=Flexible, 1=Simplified */
     
     /* Get the TBD pointer */
     tdb_p = get_uint32(addr + 8);
     
     /* check NC bit and possibly insert CRC */
     insert_crc = (I596_NOCRC_INS == 0) && ((cmd & 0x10) == 0) && !I596_LOOPBACK;
     
     if (!sf_mode) {
         /* Entering Flexible mode: check TCB count field (offset 8 in 32-bit mode) */
         uint16_t eof_flag = 0;
         tcb_count = get_uint16(addr + 10) & SIZE_MASK;
         eof_flag = get_uint16(addr + 10) & I596_EOF;
         
         /* If TCB contains data (TCB_COUNT > 0), read it first */
         if (tcb_count > 0) {
             uint32_t tcb_data_addr = addr + 16; /* Data starts after TCB header */
             
             /* Check buffer size for safety instead of using assert */
             if (tcb_count > sizeof(s->tx_buffer)) {
                 warn_report("i82596: TCB count %d exceeds buffer size %zu", 
                            tcb_count, sizeof(s->tx_buffer));
                 tcb_count = sizeof(s->tx_buffer); /* Truncate to available space */
             }
             
             /* Copy data from TCB */
             address_space_rw(&address_space_memory, tcb_data_addr,
                 MEMTXATTRS_UNSPECIFIED, s->tx_buffer, tcb_count, 0);
                 
             /* Insert MAC if needed - with bounds checking */
             if (I596_NO_SRC_ADD_IN == 0 && (ETH_ALEN + ETH_ALEN <= sizeof(s->tx_buffer))) {
                 memcpy(&s->tx_buffer[ETH_ALEN], s->conf.macaddr.a, ETH_ALEN);
             }
             
             /* Calculate CRC if needed - with bounds checking */
             if (insert_crc) {
                 if (tcb_count + sizeof(uint32_t) <= sizeof(s->tx_buffer)) {
                     uint32_t crc = crc32(~0, s->tx_buffer, tcb_count);
                     crc = cpu_to_be32(crc);
                     memcpy(&s->tx_buffer[tcb_count], &crc, sizeof(crc));
                     tcb_count += sizeof(crc);
                 } else {
                     warn_report("i82596: Buffer too small to append CRC");
                 }
             }
            
            /* If EOF flag is set, this is the entire frame - send it */
            if (eof_flag || tdb_p == I596_NULL) {
                DBG(PRINT_PKTHDR("Send from TCB", &s->tx_buffer));
                DBG(printf("Sending %d bytes from TCB\n", tcb_count));
                
                switch (I596_LOOPBACK) {
                case 0:     /* no loopback, send packet */
                    qemu_send_packet_raw(qemu_get_queue(s->nic), s->tx_buffer, tcb_count);
                    break;
                case 1:     /* external loopback enabled */
                default:    /* all other loopback modes: ignore! */
                    i82596_receive(qemu_get_queue(s->nic), s->tx_buffer, tcb_count);
                    break;
                }
                qemu_flush_queued_packets(qemu_get_queue(s->nic));
                return; /* Done with this frame */
            }
        }
    }
    
    /* Process TBDs for remaining data (or all data in non-Simplified mode) */
    if (tdb_p != I596_NULL) {
        uint16_t offset = 0;
        
        /* In Flexible mode with TCB data, we need to append to existing data */
        if (!sf_mode && tcb_count > 0) {
            offset = tcb_count;
        }
        
        /* Process the buffer descriptors */
        while (tdb_p != I596_NULL) {
            uint16_t size, len;
            uint32_t tba;
            
            size = get_uint16(tdb_p);
            len = size & SIZE_MASK;
            tba = get_uint32(tdb_p + 8);
            trace_i82596_transmit(len, tba);
            
            if (s->nic && len) {
                /* Ensure we don't overflow the buffer */
                assert(offset + len <= sizeof(s->tx_buffer));
                
                /* Copy data from buffer */
                address_space_rw(&address_space_memory, tba,
                    MEMTXATTRS_UNSPECIFIED, s->tx_buffer + offset, len, 0);
                
                /* Insert MAC in the first buffer if needed and no TCB data */
                if (offset == 0 && I596_NO_SRC_ADD_IN == 0) {
                    memcpy(&s->tx_buffer[ETH_ALEN], s->conf.macaddr.a, ETH_ALEN);
                }
                offset += len;
            }
            
            /* Was this the last package? */
            if (size & I596_EOF) {
                if (insert_crc) {
                    uint32_t crc = crc32(~0, s->tx_buffer, offset);
                    crc = cpu_to_be32(crc);
                    memcpy(&s->tx_buffer[offset], &crc, sizeof(crc));
                    offset += sizeof(crc);
                }
                
                DBG(PRINT_PKTHDR("Send", &s->tx_buffer));
                DBG(printf("Sending %d bytes (crc_inserted=%d)\n", offset, insert_crc));
                
                switch (I596_LOOPBACK) {
                case 0:     /* no loopback, send packet */
                    qemu_send_packet_raw(qemu_get_queue(s->nic), s->tx_buffer, offset);
                    break;
                case 1:     /* external loopback enabled */
                default:    /* all other loopback modes: ignore! */
                    i82596_receive(qemu_get_queue(s->nic), s->tx_buffer, offset);
                    break;
                }
                
                qemu_flush_queued_packets(qemu_get_queue(s->nic));
                break;
            }
            
            /* Get next buffer pointer */
            tdb_p = get_uint32(tdb_p + 4);
        }
    } else if (sf_mode) {
        /* In Simplified mode with no TBD, all data is in the TCB */
        uint16_t len;
        uint32_t tcb_data_addr = addr + 16; /* Data starts after TCB header */
        
        /* In Simplified mode, SIZE field at offset 12 indicates total data size */
        len = get_uint16(addr + 12) & SIZE_MASK;
        
        if (s->nic && len) {
            /* Make sure buffer is large enough */
            uint16_t new_len = len + 4; /* +4 for CRC */
            assert(new_len <= sizeof(s->tx_buffer));
            
            /* Copy data from TCB */
            address_space_rw(&address_space_memory, tcb_data_addr,
                MEMTXATTRS_UNSPECIFIED, s->tx_buffer, len, 0);
            
            /* Insert MAC if needed */
            if (I596_NO_SRC_ADD_IN == 0) {
                memcpy(&s->tx_buffer[ETH_ALEN], s->conf.macaddr.a, ETH_ALEN);
            }
            
            DBG(printf("i82596_transmit simplified: insert_crc = %d insert SRC = %d\n",
                    insert_crc, I596_NO_SRC_ADD_IN == 0));
            
            if (insert_crc) {
                uint32_t crc = crc32(~0, s->tx_buffer, len);
                crc = cpu_to_be32(crc);
                memcpy(&s->tx_buffer[len], &crc, sizeof(crc));
                len += sizeof(crc);
            }
            
            DBG(PRINT_PKTHDR("Send Simplified", &s->tx_buffer));
            DBG(printf("Sending %d bytes (simplified mode)\n", len));
            
            switch (I596_LOOPBACK) {
            case 0:     /* no loopback, send packet */
                qemu_send_packet_raw(qemu_get_queue(s->nic), s->tx_buffer, len);
                break;
            case 1:     /* external loopback enabled */
            default:    /* all other loopback modes: ignore! */
                i82596_receive(qemu_get_queue(s->nic), s->tx_buffer, len);
                break;
            }
        }
        
        qemu_flush_queued_packets(qemu_get_queue(s->nic));
     }
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
         address_space_rw(&address_space_memory, addr + i * ETH_ALEN, 
            MEMTXATTRS_UNSPECIFIED, multicast_addr, ETH_ALEN, 0);
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
     s->scb_status = (s->scb_status & 0xf000) | (s->cu_status << 8) | (s->rx_status << 4) | 8; /* 8: bus throttle timers loaded */
     DBG(printf("update_scb_status 0x%04x CUS: %d, RUS: %d\n", s->scb_status, s->cu_status, s->rx_status));
     set_uint16(s->scb, s->scb_status);
 }

 static void i82596_s_reset(I82596State *s)
 {
     trace_i82596_s_reset(s);
     DBG(printf("i82596_s_reset()\n"));
     s->scp = 0x00FFFFF4;
     s->scb_status = 0;
     s->cu_status = CU_IDLE;
     s->rx_status = RX_SUSPENDED;
     s->cmd_p = I596_NULL;
     s->lnkst = 0x8000; /* initial link state: up */
     s->ca = s->ca_active = 0;
     s->send_irq = 0;
 }
 
 static void command_loop(I82596State *s)
 {
     uint16_t cmd;
     uint16_t status;
     uint8_t byte_cnt;
 
     DBG(printf("STARTING COMMAND LOOP cmd_p=0x%08x\n", s->cmd_p));
 
     while (s->cmd_p != I596_NULL) {
         /* set status */
         status = STAT_B;
         set_uint16(s->cmd_p, status);
         status = STAT_C | STAT_OK; /* update, but write later */
 
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
             byte_cnt = get_byte(s->cmd_p + 8) & 0x0f;
             byte_cnt = MAX(byte_cnt, 4);
             byte_cnt = MIN(byte_cnt, sizeof(s->config));
             /* copy byte_cnt max. */
             address_space_rw(&address_space_memory, s->cmd_p + 8,
                 MEMTXATTRS_UNSPECIFIED, s->config, byte_cnt, 0);
             /* config byte according to page 35ff */
             s->config[2] &= 0x82; /* mask valid bits */
             s->config[2] |= 0x40;
             DBG(printf("I596_CONFIG3 = 0x%02x  LOOPBACK 0x%x\n", s->config[3], I596_LOOPBACK));
             if (I596_NO_SRC_ADD_IN == 0) {
                 assert((s->config[3] & 0x07) == ETH_ALEN);
             }
             s->config[7]  &= 0xf7; /* clear zero bit */
             assert(I596_CRC16_32 == 0); /* only CRC-32 implemented */
             DBG(printf("I596_CRCINM = %d\n\n", I596_CRCINM));
             s->config[10] = MAX(s->config[10], 5); /* min frame length */
             s->config[12] &= 0x40; /* only full duplex field valid */
             s->config[13] |= 0x3f; /* set ones in byte 13 */
             s->scb_status |= SCB_STATUS_CX;
             break;
         case CmdTDR:
             /* get signal LINK */
             set_uint32(s->cmd_p + 8, s->lnkst);
             break;
         case CmdTx:
             i82596_transmit(s, s->cmd_p);
             break;
         case CmdMulticastList:
             set_multicast_list(s, s->cmd_p);
             break;
         case CmdDump:
         case CmdDiagnose:
             printf("FIXME Command %d !!\n", cmd & 7);
             g_assert_not_reached();
         }
 
         /* update status */
         set_uint16(s->cmd_p, status);
 
         s->cmd_p = get_uint32(s->cmd_p + 4); /* get link address */
         DBG(printf("NEXT loop addr is 0x%08x\n", s->cmd_p));
         if (s->cmd_p == 0) {
             s->cmd_p = I596_NULL;
         }
 
         /* Stop when last command of the list. */
         if (cmd & CMD_EOL) {
             s->cmd_p = I596_NULL;
         }
         /* Suspend after doing cmd? */
         if (cmd & CMD_SUSP) {
             s->cu_status = CU_SUSPENDED;
             printf("FIXME SUSPEND !!\n");
         }
 
         /* Interrupt after doing cmd? */
         if (cmd & CMD_INTR) {
             s->scb_status |= SCB_STATUS_CX;
             s->send_irq = 1;
         }
 
         if (s->cu_status == CU_SUSPENDED) {
             break;
         }
     }
     DBG(printf("FINISHED COMMAND LOOP\n"));
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
     uint16_t command, cuc, ruc, c;
 
     /* get the scb command word */
     command = get_uint16(s->scb + 2);
     DBG(printf("COMMAND = 0x%04x\n", command));
     cuc = (command >> 8) & 0x7;
     ruc = (command >> 4) & 0x7;
     DBG(printf("MAIN CU COMMAND 0x%04x: stat 0x%02x cuc 0x%02x ruc 0x%02x\n",
             command, command >> 12,  cuc, ruc));
 
     /* toggle the STAT flags in SCB status word */
     c = command & (SCB_STATUS_CX | SCB_STATUS_FR | SCB_STATUS_CNA | SCB_STATUS_RNR);
     s->scb_status &= ~c;
 
     switch (cuc) {
     case 0:     /* no change */
     case 5:
     case 6:
         break;
     case 1:     /* CUC_START */
         s->cu_status = CU_ACTIVE;
         break;
     case 4:     /* CUC_ABORT */
         s->cu_status = CU_IDLE;
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
         s->rx_status = RX_READY;
         break;
     case 3:     /* RX_SUSPEND */
         s->rx_status = RX_SUSPENDED;
         s->scb_status |= SCB_STATUS_RNR; /* RU left active state */
         s->send_irq = 1;
         break;
     case 4:     /* RX_ABORT */
         s->rx_status = RX_IDLE;
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
     if (s->cu_status == CU_ACTIVE) {
         if (s->cmd_p == I596_NULL) {
             s->cmd_p = get_uint32(s->scb + 4);
         }
         command_loop(s);
         s->cu_status = CU_IDLE;
         s->send_irq = 1;
     }
 
     qemu_flush_queued_packets(qemu_get_queue(s->nic));
 }
 
 static void signal_ca(I82596State *s)
{
    DBG(printf("-- CA start\n"));

    trace_i82596_channel_attention(s);
    
    if (s->scp) {
        uint32_t iscp;
        uint8_t sysbus;
        uint8_t mode;
        
        /* After reset, initialize with the new SCP */
        sysbus = get_byte(s->scp + 3); /* big endian */
        DBG(printf("SYSBUS = 0x%02x\n", sysbus));
        
        /* Extract operation mode from SYSBUS byte */
        mode = (sysbus >> 1) & 0x03;
        
        /* Check for supported modes:
         * 0 = 82586 mode
         * 1 = 32-bit segmented mode
         * 2 = Linear mode
         */
        if (mode != 2) { /* MODE_LINEAR */
            warn_report("i82596: Only Linear Mode (2) is currently implemented! Mode = %d", mode);
        }
        
        /* Check for enhanced big endian mode (C-step feature) */
        if (sysbus >> 7) {
            warn_report("i82596: Enhanced Big Endian Mode not supported");
        }
        
        /* Get ISCP address from SCP */
        iscp = get_uint32(s->scp + 8);
        
        /* Get SCB address from ISCP */
        s->scb = get_uint32(iscp + 4);
        DBG(printf("ISCP = 0x%08x, SCB = 0x%08x\n", iscp, s->scb));
        
        /* Clear BUSY flag in ISCP as per documentation */
        set_byte(iscp + 1, 0);
        
        /* Per documentation: sets CX and CNA to 1 in the SCB, resets CU and RU states */
        s->scb_status = SCB_STATUS_CX | SCB_STATUS_CNA;
        s->cu_status = CU_IDLE;
        s->rx_status = RX_IDLE;
        s->scp = 0;
        s->send_irq = 1;
    } else {
        /* Process commands in the SCB if no initialization is happening */
        examine_scb(s);
    }
    
    /* Update SCB status and clear command word */
    update_scb_status(s);
    set_uint16(s->scb + 2, 0);
    
    /* Send interrupt if requested */
    if (s->send_irq) {
        s->send_irq = 0;
        DBG(printf("Send IRQ\n"));
        qemu_set_irq(s->irq, 1);
    }
    
    DBG(printf("-- CA end\n"));
}
 
 void i82596_ioport_writew(void *opaque, uint32_t addr, uint32_t val)
 {
     I82596State *s = opaque;
     uint32_t res, tmp;
     DBG(printf("i82596_ioport_writew addr=0x%08x val=0x%04x\n", addr, val));
     switch (addr) {
     case PORT_RESET: /* Reset */
         i82596_s_reset(s);
         break;
     case PORT_SELFTEST:
         res = val + sizeof(uint32_t);
         tmp = get_uint32(res); /* should be -1 */
         DBG(printf("i82596 SELFTEST at 0x%04x val 0x%04x requested.\n", res, tmp));
         assert(tmp == I596_NULL);
         set_uint32(res, 0); /* set to zero */
         break;
     case PORT_ALTSCP:
         DBG(printf("i82596 ALTSCP requested.\n"));
         s->scp = val;
         break;
     case PORT_ALTDUMP:
         printf("i82596 PORT_ALTDUMP not implemented yet.\n");
         break;
     case PORT_CA:
         signal_ca(s);
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
 
     if (s->rx_status != RX_READY) {
         return false;
     }
 
     /* Link down? */
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
    uint8_t buf1[ETHER_MIN_LEN + ETHER_CRC_LEN]; /* 64 bytes - standard minimum Ethernet frame */
    static const uint8_t broadcast_macaddr[6] = {
                0xff, 0xff, 0xff, 0xff, 0xff, 0xff };

    DBG(printf("i82596_receive() start, sz = %lu\n", sz));

    /* first check if receiver is enabled */
    if (s->rx_status == RX_SUSPENDED) {
        trace_i82596_receive_analysis(">>> Receiving suspended");
        return -1;
    }

    if (!s->lnkst) {
        trace_i82596_receive_analysis(">>> Link down");
        return -1;
    }

    /* Received frame smaller than configured "min frame len"? */
    if (sz < s->config[10]) {
        DBG(printf("Received frame too small, %lu vs. %u bytes\n",
            sz, s->config[10]));
        sz = 60; /* return -1; */
    }
    
    DBG(printf("Received %lu bytes\n", sz));

    /* Address filtering logic - unchanged */
    if (I596_PROMISC || I596_LOOPBACK) {
        /* promiscuous: receive all */
        trace_i82596_receive_analysis(
                ">>> packet received in promiscuous mode");
    } else {
        /* Broadcast check - unchanged */
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
            /* Replace assertion with runtime check */
            if (mcast_idx >= 8 * sizeof(s->mult)) {
                warn_report("i82596: Multicast index out of range");
                return len;
            }

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

    /* if too small buffer, then expand it safely */
    if (len < MIN_BUF_SIZE + VLAN_HLEN) {
        /* Safety check to prevent buffer overflow */
        size_t copy_len = MIN(len, sizeof(buf1));
        if (copy_len < len) {
            warn_report("i82596: Packet too large for buffer (%zu > %zu)", 
                       len, sizeof(buf1));
        }
        
        memcpy(buf1, buf, copy_len);
        memset(buf1 + copy_len, 0, sizeof(buf1) - copy_len);
        buf = buf1;
        if (len < MIN_BUF_SIZE) {
            len = MIN_BUF_SIZE;
        }
    }

    /* Calculate the ethernet checksum (4 bytes) */
    if (I596_CRCINM && !I596_LOOPBACK) {
        len += 4;
        crc = crc32(~0, buf, sz);
        crc = cpu_to_be32(crc);
        crc_ptr = (uint8_t *) &crc;
    }

    /* Get initial Receive Frame Descriptor with validation */
    rfd_p = get_uint32(s->scb + 8);
    if (!rfd_p || rfd_p == I596_NULL) {
        warn_report("i82596: Invalid RFD pointer");
        s->scb_status |= SCB_STATUS_RNR;
        s->rx_status = RX_NO_RESOURCES;
        return -1;
    }

    do {
        status = get_uint16(rfd_p+0);
        /* if rfd is filled, get next one from link addr */
        if (status & STAT_OK) {
            uint32_t next_rfd_p = get_uint32(rfd_p + 4);
            if (!next_rfd_p || next_rfd_p == I596_NULL) {
                warn_report("i82596: Invalid next RFD pointer");
                return -1;
            }
            rfd_p = next_rfd_p;
        }
    } while (status & STAT_OK);

    trace_i82596_receive_packet(len);
    DBG(PRINT_PKTHDR("Receive", buf));

    while (len) {
        uint16_t command, rba_size;
        uint32_t next_rfd, rba, actual_count_ptr;

        DBG(printf("Receive: rfd is 0x%08x, len = %lu\n", rfd_p, len));
        command = get_uint16(rfd_p + 2);
        int sf_mode = (command & CMD_FLEX) ? 0 : 1; /* 0=Flexible, 1=Simplified */
    
        DBG(printf("Receive: EL=%d, S(uspend)=%d, Mode=%s\n", 
             (command & CMD_EOL) ? 1 : 0, (command & CMD_SUSP) ? 1 : 0,
             sf_mode ? "Simplified" : "Flexible"));
    
        /* Get first RBD in Flexible mode, ignored in Simplified */
        rbd = get_uint32(rfd_p + 8);
        if (sf_mode) {
            /* Simplified mode - all data goes directly into RFD */
            rba = rfd_p + 16; /* Data starts after RFD header */
            rba_size = get_uint16(rfd_p + 12) & SIZE_MASK;
            actual_count_ptr = rfd_p + 12;
            
            /* Store number of received bytes */
            uint16_t num = rba_size;
            if (num > len) {
                num = len;
            }
            uint16_t actual_count = num | 0x4000; /* Set F bit */
            
            if (num == len) {
                actual_count |= I596_EOF; /* Set EOF bit if this is the last buffer */
            }
            
            set_uint16(actual_count_ptr, actual_count);
            
            /* Copy data to memory */
            if (num > 0) {
                address_space_rw(&address_space_memory, rba,
                    MEMTXATTRS_UNSPECIFIED, (void *)buf, num, 1);
                    
                /* Copy CRC if needed */
                if (len == num && I596_CRCINM && !I596_LOOPBACK) {
                    address_space_rw(&address_space_memory, rba + num - 4,
                        MEMTXATTRS_UNSPECIFIED, crc_ptr, 4, 1);
                }
            }
            
            /* Update pointers and remaining length */
            buf += num;
            len -= num;
            
            /* If buffer was too small, this is a no resources error */
            if (len > 0) {
                /* Set no resources error bit */
                status |= (1 << 9);
                s->scb_status |= SCB_STATUS_RNR;
                s->rx_status = RX_NO_RESOURCES;
            }
        } else {
            /* Flexible mode - data can be in both RFD and RBDs */
            if (rbd == I596_NULL) {
                DBG(printf("Flexible mode with NULL RBD pointer\n"));
                /* Without RBDs, we can't store the data */
                s->scb_status |= SCB_STATUS_RNR;
                s->rx_status = RX_NO_RESOURCES;
                return -1;
            }
            
            /* Possibly store first bytes in RFD */
            rba = rfd_p + 16;
            rba_size = get_uint16(rfd_p + 14) & SIZE_MASK;
            actual_count_ptr = rfd_p + 12;
            
            /* Continue with existing implementation for Flexible mode */
            while (len) {
                uint16_t num, actual_count;
                
                DBG(printf("rba is at 0x%x, rba_size = %d, cnt_ptr 0x%08x\n", 
                      rba, rba_size, actual_count_ptr));
                
                /* Store number of received bytes first */
                num = rba_size & SIZE_MASK;
                if (num > len) {
                    num = len;
                }
                actual_count = num;
                if (num == len) {
                    actual_count |= I596_EOF; /* set EOF BIT */
                }
                
                if (num) {
                    actual_count |= 0x4000; /* set F BIT */
                    set_uint16(actual_count_ptr, actual_count);
                    
                    address_space_rw(&address_space_memory, rba,
                        MEMTXATTRS_UNSPECIFIED, (void *)buf, num, 1);
                }
                
                rba += num;
                buf += num;
                len -= num;
                
                if (len == 0 && I596_CRCINM && !I596_LOOPBACK) {
                    address_space_rw(&address_space_memory, rba - 4,
                        MEMTXATTRS_UNSPECIFIED, crc_ptr, 4, 1);
                }
                
                if (len == 0) {
                    break; /* done with this frame */
                }
                
                if (rba_size & I596_EOF) {
                    break; /* last entry */
                }
                
                /* Get next RBD with validation */
                DBG(printf("Receive: rbd is 0x%08x\n", rbd));
                rba_size = get_uint16(rbd + 12);
                if (rba_size == 0) {
                    warn_report("i82596: RBD size is zero");
                    s->scb_status |= SCB_STATUS_RNR;
                    s->rx_status = RX_NO_RESOURCES;
                    return -1;
                }
                
                rba = get_uint32(rbd + 8);
                actual_count_ptr = rbd + 0;
                
                rbd = get_uint32(rbd + 4);
            }
        }
        
        /* Housekeeping, see pg. 18 */
        next_rfd = get_uint32(rfd_p + 4);
        if (!next_rfd || next_rfd == I596_NULL) {
            warn_report("i82596: Invalid next RFD pointer in housekeeping");
            s->scb_status |= SCB_STATUS_RNR;
            s->rx_status = RX_NO_RESOURCES;
            return -1;
        }
        
        set_uint32(next_rfd + 8, rbd);

        status = STAT_C | STAT_OK | is_broadcast;
        set_uint16(rfd_p, status);

        if (command & CMD_SUSP) {  /* suspend after command? */
            s->rx_status = RX_SUSPENDED;
            s->scb_status |= SCB_STATUS_RNR; /* RU left active state */
            break;
        }
        
        if (command & CMD_EOL) /* was it last Frame Descriptor? */
            break;

        /* Check if we still have data to process */
        if (len != 0) {
            warn_report("i82596: Frame data not completely processed (remaining: %zu bytes)", len);
        }
    }
    /* If we have data left, set no resources error */
    if (len != 0) {
        warn_report("i82596: Frame data not completely processed (remaining: %zu bytes)", len);
    }
 
     s->scb_status |= SCB_STATUS_FR; /* set "RU finished receiving frame" bit. */
     update_scb_status(s);
 
    /* send IRQ that we received data */
    qemu_set_irq(s->irq, 1);

    DBG(printf("i82596_receive() end sz = %lu\n", sz));
    if (0) {
        DBG(printf("Checking:\n"));
        rfd_p = get_uint32(s->scb + 8); /* get Receive Frame Descriptor */
        DBG(printf("Next Receive: rfd is %08x\n", rfd_p));
        rfd_p = get_uint32(rfd_p + 4); /* get Next Receive Frame Descriptor */
        DBG(printf("Next Receive: rfd is %08x\n", rfd_p));
        /* get first Receive Buffer Descriptor Address */
        rbd = get_uint32(rfd_p + 8);
        DBG(printf("Next Receive: rbd is %08x\n", rbd));
     }
 
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
 }
 