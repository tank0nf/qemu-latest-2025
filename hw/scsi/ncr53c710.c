/*
 * QEMU NCR53C710 SCSI Host Bus Adapter emulation
 *
 * Copyright (c) 2024 QEMU contributors
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/pci/pci.h"
#include "hw/scsi/scsi.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "sysemu/dma.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "trace.h"
#include "qom/object.h"

/* Debug options */
//#define DEBUG_NCR53C710
//#define DEBUG_NCR53C710_REG
//#define DEBUG_WEIRD_CASES   /* my personal flag for tracking strange chip behavior */

#ifdef DEBUG_NCR53C710
#define DPRINTF(fmt, ...) \
    do { printf("NCR53C710: " fmt , ## __VA_ARGS__); } while (0)
/* Sometimes I need to track specific sequences */
#define DBG_SPECIAL(fmt, ...) \
    do { printf("### NCR53C710 [%s]: " fmt, __func__ , ## __VA_ARGS__); } while (0)
#define BADF(fmt, ...) \
    do { printf("NCR53C710: ERROR: " fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...) do {} while(0)
#define DBG_SPECIAL(fmt, ...) do {} while(0)
#define BADF(fmt, ...) \
    do { qemu_log_mask(LOG_GUEST_ERROR, "NCR53C710: ERROR: " fmt , ## __VA_ARGS__); } while (0)
#endif

 
 /* Flag set if this is a tagged command */
 #define NCR53C710_TAG_VALID (1 << 16)
 
 /* Register definitions */
 /* SCSI Control Register 0 (SCNTL0) */
 #define NCR53C710_SCNTL0_TRG     0x01    /* Target mode */
 #define NCR53C710_SCNTL0_AAP     0x02    /* Assert ATN on parity error */
 #define NCR53C710_SCNTL0_EPG     0x04    /* Enable parity generation */
 #define NCR53C710_SCNTL0_EPC     0x08    /* Enable parity checking */
 #define NCR53C710_SCNTL0_WATN    0x10    /* Select with ATN */
 #define NCR53C710_SCNTL0_START   0x20    /* Start sequence */
 
 /* SCSI Control Register 1 (SCNTL1) */
 #define NCR53C710_SCNTL1_RCV     0x01    /* Reserved */
 #define NCR53C710_SCNTL1_SND     0x02    /* Reserved */
 #define NCR53C710_SCNTL1_AESP    0x04    /* Assert even SCSI parity */
 #define NCR53C710_SCNTL1_RST     0x08    /* Assert SCSI RST */
 #define NCR53C710_SCNTL1_CON     0x10    /* Connected */
 #define NCR53C710_SCNTL1_ESR     0x20    /* Enable selection/reselection */
 #define NCR53C710_SCNTL1_ADB     0x40    /* Assert data bus */
 #define NCR53C710_SCNTL1_EXC     0x80    /* Extra clock cycle of data setup */
 
 /* Interrupt Status Register (ISTAT) */
 #define NCR53C710_ISTAT_DIP      0x01    /* DMA interrupt pending */
 #define NCR53C710_ISTAT_SIP      0x02    /* SCSI interrupt pending */
 #define NCR53C710_ISTAT_CON      0x08    /* Connected */
 #define NCR53C710_ISTAT_SIGP     0x20    /* Signal process */
 #define NCR53C710_ISTAT_RST      0x40    /* Reset */
 #define NCR53C710_ISTAT_ABRT     0x80    /* Abort */
 
 /* SCSI Status Register 0 (SSTAT0) */
 #define NCR53C710_SSTAT0_PAR     0x01    /* Parity error */
 #define NCR53C710_SSTAT0_RST     0x02    /* SCSI RST received */
 #define NCR53C710_SSTAT0_UDC     0x04    /* Unexpected disconnect */
 #define NCR53C710_SSTAT0_SGE     0x08    /* SCSI gross error */
 #define NCR53C710_SSTAT0_SEL     0x10    /* Selected or reselected */
 #define NCR53C710_SSTAT0_STO     0x20    /* SCSI timeout */
 #define NCR53C710_SSTAT0_FCMP    0x40    /* Function complete */
 #define NCR53C710_SSTAT0_MA      0x80    /* Phase mismatch or ATNI active */
 
 /* DMA Status Register (DSTAT) */
 #define NCR53C710_DSTAT_IID      0x01    /* Illegal instruction detected */
 #define NCR53C710_DSTAT_WTD      0x02    /* Watchdog timeout detected */
 #define NCR53C710_DSTAT_SIR      0x04    /* SCRIPT interrupt instruction received */
 #define NCR53C710_DSTAT_SSI      0x08    /* SCRIPT step interrupt */
 #define NCR53C710_DSTAT_ABRT     0x10    /* Aborted */
 #define NCR53C710_DSTAT_BF       0x20    /* Bus fault */
 #define NCR53C710_DSTAT_DFE      0x80    /* DMA FIFO empty */
 
 /* SCSI Output Control Latch (SOCL) */
 #define NCR53C710_SOCL_IO        0x01    /* I/O */
 #define NCR53C710_SOCL_CD        0x02    /* C/D */
 #define NCR53C710_SOCL_MSG       0x04    /* MSG */
 #define NCR53C710_SOCL_ATN       0x08    /* ATN */
 #define NCR53C710_SOCL_SEL       0x10    /* SEL */
 #define NCR53C710_SOCL_BSY       0x20    /* BSY */
 #define NCR53C710_SOCL_ACK       0x40    /* ACK */
 #define NCR53C710_SOCL_REQ       0x80    /* REQ */
 
 /* SCSI Bus Control Lines (SBCL) */
 #define NCR53C710_SBCL_IO        0x01    /* I/O */
 #define NCR53C710_SBCL_CD        0x02    /* C/D */
 #define NCR53C710_SBCL_MSG       0x04    /* MSG */
 #define NCR53C710_SBCL_ATN       0x08    /* ATN */
 #define NCR53C710_SBCL_SEL       0x10    /* SEL */
 #define NCR53C710_SBCL_BSY       0x20    /* BSY */
 #define NCR53C710_SBCL_ACK       0x40    /* ACK */
 #define NCR53C710_SBCL_REQ       0x80    /* REQ */
 
 /* DMA Control (DCNTL) */
 #define NCR53C710_DCNTL_COM      0x01    /* 53C700 compatibility */
 #define NCR53C710_DCNTL_IRQD     0x02    /* IRQ disable */
 #define NCR53C710_DCNTL_STD      0x04    /* Start DMA operation */
 #define NCR53C710_DCNTL_IRQM     0x08    /* IRQ mode */
 #define NCR53C710_DCNTL_SSM      0x10    /* Single-step mode */
 #define NCR53C710_DCNTL_PFEN     0x20    /* Prefetch enable */
 #define NCR53C710_DCNTL_PFF      0x40    /* Prefetch flush */
 #define NCR53C710_DCNTL_CLSE     0x80    /* Cache line size enable */
 
 /* SCSI phase values */
 #define NCR53C710_PHASE_DO       0       /* Data out */
 #define NCR53C710_PHASE_DI       1       /* Data in */
 #define NCR53C710_PHASE_CMD      2       /* Command */
 #define NCR53C710_PHASE_ST       3       /* Status */
 #define NCR53C710_PHASE_MO       6       /* Message out */
 #define NCR53C710_PHASE_MI       7       /* Message in */
 #define NCR53C710_PHASE_MASK     7       /* Mask for phase bits */
 
 /* Structure to track SCSI requests */
 typedef struct NCR53C710Request {
     SCSIRequest *req;
     uint32_t tag;
     uint32_t dma_len;
     uint8_t *dma_buf;
     uint32_t pending;
     int out;
     QTAILQ_ENTRY(NCR53C710Request) next;
 } NCR53C710Request;
 
 /* Main device state structure */
 typedef struct NCR53C710State {
     /*< private >*/
     PCIDevice parent_obj;
     /*< public >*/
 
     int carry;              /* Carry flag for some operations */
     int status;             /* SCSI status */
     int msg_action;         /* Action for MSG IN phase end */
     int msg_len;            /* MSG data length */
     uint8_t msg[NCR53C710_MAX_MSGIN_LEN]; /* Message data buffer */
     int waiting;            /* 0: idle, 1: waiting for reselect, 2: during DMA, 3: DMA in progress */
     SCSIBus bus;            /* SCSI bus */
     int current_lun;        /* Current LUN */
     uint32_t select_tag;    /* Tag during selection */
     int command_complete;   /* Command completion flag */
     QTAILQ_HEAD(, NCR53C710Request) queue; /* Request queue */
     NCR53C710Request *current; /* Current request */
 
     /* IRQ */
     qemu_irq irq;
 
     /* Registers */
     uint32_t dsa;           /* Data structure address */
     uint32_t temp;          /* Temporary register */
     uint32_t dnad;          /* DMA next address for data */
     uint32_t dbc;           /* DMA byte counter */
     uint8_t istat;          /* Interrupt status */
     uint8_t dcmd;           /* DMA command */
     uint8_t dstat;          /* DMA status */
     uint8_t dien;           /* DMA interrupt enable */
     uint8_t sien0;          /* SCSI interrupt enable 0 */
     uint8_t ctest2;         /* Chip test 2 */
     uint8_t ctest3;         /* Chip test 3 */
     uint8_t ctest4;         /* Chip test 4 */
     uint8_t ctest5;         /* Chip test 5 */
     uint32_t dsp;           /* DMA SCRIPTS pointer */
     uint32_t dsps;          /* DMA SCRIPTS pointer save */
     uint8_t dmode;          /* DMA mode */
     uint8_t dcntl;          /* DMA control */
     uint8_t scntl0;         /* SCSI control 0 */
     uint8_t scntl1;         /* SCSI control 1 */
     uint8_t sstat0;         /* SCSI status 0 */
     uint8_t sstat1;         /* SCSI status 1 */
     uint8_t sstat2;         /* SCSI status 2 */
     uint8_t scid;           /* SCSI chip ID */
     uint8_t sxfer;          /* SCSI transfer */
     uint8_t socl;           /* SCSI output control latch */
     uint8_t sdid;           /* SCSI destination ID */
     uint8_t sfbr;           /* SCSI first byte received */
     uint8_t sidl;           /* SCSI input data latch */
     uint32_t sbc;           /* SCSI byte counter */
     uint32_t scratch;       /* General purpose scratch pad */
     uint8_t sbr;            /* SCSI bus register */
     uint8_t ctest0;         /* Chip test 0 */
     uint8_t ctest1;         /* Chip test 1 */
     uint8_t ctest6;         /* Chip test 6 */
     uint8_t ctest7;         /* Chip test 7 */
     uint8_t ctest8;         /* Chip test 8 */
     uint8_t lcrc;           /* Longitudinal parity */
     uint8_t dwt;            /* DMA watchdog timer */
     uint8_t sbcl;           /* SCSI bus control lines */
     uint8_t script_active;  /* SCRIPT is currently executing */
 } NCR53C710State;
 
 /* SYSBUS device state structure */
 #define TYPE_NCR53C710 "ncr53c710"
 OBJECT_DECLARE_SIMPLE_TYPE(NCR53C710SysBusDeviceState, NCR53C710)
 
 struct NCR53C710SysBusDeviceState {
     /*< private >*/
     SysBusDevice parent_obj;
     /*< public >*/
 
     MemoryRegion mmio;
     NCR53C710State state;
 };
 
 /* Function prototypes */
 static void ncr53c710_soft_reset(NCR53C710State *s);
 static void ncr53c710_update_irq(NCR53C710State *s);
 static void ncr53c710_execute_script(NCR53C710State *s);
 static void ncr53c710_script_dma_interrupt(NCR53C710State *s, int stat);
 static void ncr53c710_script_scsi_interrupt(NCR53C710State *s, int stat0);
 static uint8_t ncr53c710_reg_readb(NCR53C710State *s, int offset);
 static void ncr53c710_reg_writeb(NCR53C710State *s, int offset, uint8_t val);
 
 /* DMA read from system memory */
 static void ncr53c710_dma_read(NCR53C710State *s, dma_addr_t addr,
                                void *buf, dma_addr_t len)
 {
     address_space_read(&address_space_memory, addr, MEMTXATTRS_UNSPECIFIED, 
                        buf, len);
 }
 
 /* DMA write to system memory */
 static void ncr53c710_dma_write(NCR53C710State *s, dma_addr_t addr,
                                 const void *buf, dma_addr_t len)
 {
     address_space_write(&address_space_memory, addr, MEMTXATTRS_UNSPECIFIED, 
                         buf, len);
 }
 
 /* Read a 32-bit word from system memory (with endianness conversion) */
 static inline uint32_t ncr53c710_read_dword(NCR53C710State *s, uint32_t addr)
 {
     uint32_t buf;
 
     ncr53c710_dma_read(s, addr, &buf, 4);
     DPRINTF("read_dword addr %#08x => %#08x\n", addr, le32_to_cpu(buf));
     return le32_to_cpu(buf);
 }
 
 /* Stop SCRIPTS execution */
 static void ncr53c710_stop_script(NCR53C710State *s)
 {
     DPRINTF("Stopping script execution\n");
     s->script_active = 0;
 }
 
 /* Set external IRQ line */
 static void ncr53c710_set_irq(NCR53C710State *s, int level)
 {
     DPRINTF("Setting IRQ to %d\n", level);
     qemu_set_irq(s->irq, level);
 }
 
 /* Update the IRQ status based on pending interrupts */
 static void ncr53c710_update_irq(NCR53C710State *s)
 {
     int level;
     NCR53C710Request *p;
 
     level = 0;
     if (s->dstat) {
         if (s->dstat & s->dien) {
             level = 1;
         }
         s->istat |= NCR53C710_ISTAT_DIP;
     } else {
         s->istat &= ~NCR53C710_ISTAT_DIP;
     }
 
     if (s->sstat0) {
         if (s->sstat0 & s->sien0) {
             level = 1;
         }
         s->istat |= NCR53C710_ISTAT_SIP;
     } else {
         s->istat &= ~NCR53C710_ISTAT_SIP;
     }
 
     DPRINTF("Update IRQ level %d dstat %02x sstat0 %02x\n", 
            level, s->dstat, s->sstat0);
     
     ncr53c710_set_irq(s, level);
 
     /* Check for pending reselections */
     if (!level && !(s->scntl1 & NCR53C710_SCNTL1_CON)) {
         DPRINTF("Handled IRQs & disconnected, looking for pending processes\n");
         QTAILQ_FOREACH(p, &s->queue, next) {
             if (p->pending) {
                 ncr53c710_reselect(s, p);
                 break;
             }
         }
     }
 }
 
 /* Raise a SCSI interrupt */
 static void ncr53c710_script_scsi_interrupt(NCR53C710State *s, int stat0)
 {
     uint32_t mask0;
 
     DPRINTF("SCSI Interrupt %#02x prev %#02x\n", stat0, s->sstat0);
     s->sstat0 |= stat0;
     
     /* Stop processor on fatal or unmasked interrupt */
     mask0 = s->sien0 | ~(NCR53C710_SSTAT0_FCMP | NCR53C710_SSTAT0_SEL);
     
     /* As a special hack, don't stop processing when raising STO */
     /* Instead continue execution and stop at the next insn that accesses the SCSI bus */
     if (s->sstat0 & mask0) {
         ncr53c710_stop_script(s);
     }
     
     ncr53c710_update_irq(s);
 }
 
 /* Raise a DMA interrupt */
 static void ncr53c710_script_dma_interrupt(NCR53C710State *s, int stat)
 {
     DPRINTF("DMA Interrupt 0x%x prev 0x%x\n", stat, s->dstat);
     s->dstat |= stat;
     ncr53c710_update_irq(s);
     ncr53c710_stop_script(s);
 }
 
 /* Set the SCSI phase */
 static inline void ncr53c710_set_phase(NCR53C710State *s, int phase)
 {
     s->sstat2 = (s->sstat2 & ~NCR53C710_PHASE_MASK) | phase;
     s->ctest0 &= ~1;
     if (phase == NCR53C710_PHASE_DI) {
         s->ctest0 |= 1;
     }
     s->sbcl &= ~NCR53C710_SBCL_REQ;
 }
 
 /* Handle phase mismatch */
 static void ncr53c710_bad_phase(NCR53C710State *s, int out, int new_phase)
 {
     /* Trigger a phase mismatch */
     DPRINTF("Phase mismatch interrupt\n");
     ncr53c710_script_scsi_interrupt(s, NCR53C710_SSTAT0_MA);
     ncr53c710_stop_script(s);
     ncr53c710_set_phase(s, new_phase);
     s->sbcl |= NCR53C710_SBCL_REQ;
 }
 
 /* Resume script execution after DMA operation */
 static void ncr53c710_resume_script(NCR53C710State *s)
 {
     if (s->waiting != 2) {
         s->waiting = 0;
         ncr53c710_execute_script(s);
     } else {
         s->waiting = 0;
     }
 }
 
 /* Disconnect from the SCSI bus */
 static void ncr53c710_disconnect(NCR53C710State *s)
 {
     s->scntl1 &= ~NCR53C710_SCNTL1_CON;
     s->sstat2 &= ~NCR53C710_PHASE_MASK;
 }
 
 /* Handle selection of non-existent target */
 static void ncr53c710_bad_selection(NCR53C710State *s, uint32_t id)
 {
     DPRINTF("Selected absent target %d\n", id);
     ncr53c710_script_scsi_interrupt(s, NCR53C710_SSTAT0_STO);
     ncr53c710_disconnect(s);
 }
 
 /* Convert SCSI ID to number */
 static int ncr53c710_id_to_num(int id)
 {
     int num = 0;
     while (id > 1) {
         num++;
         id >>= 1;
     }
     if (num > 7) {
         num = -1;  /* Invalid ID */
     }
     return num;
 }
 
 /* Initiate a SCSI DMA transfer */
 static void ncr53c710_do_dma(NCR53C710State *s, int out)
 {
     uint32_t count;
     dma_addr_t addr;
     SCSIDevice *dev;
 
     assert(s->current);
     if (!s->current->dma_len) {
         /* Wait until data is available */
         DPRINTF("DMA no data available\n");
         return;
     }
 
     dev = s->current->req->dev;
     assert(dev);
 
     count = s->dbc;
     if (count > s->current->dma_len) {
         count = s->current->dma_len;
     }
 
     addr = s->dnad;
     DPRINTF("DMA addr=0x" DMA_ADDR_FMT " len=%d\n", addr, count);
     
     s->dnad += count;
     s->dbc -= count;
     
     if (s->current->dma_buf == NULL) {
         s->current->dma_buf = scsi_req_get_buf(s->current->req);
     }
     
     if (out) {
         /* Data from system memory to device */
         ncr53c710_dma_read(s, addr, s->current->dma_buf, count);
     } else {
         /* Data from device to system memory */
         ncr53c710_dma_write(s, addr, s->current->dma_buf, count);
     }
     
     s->current->dma_len -= count;
     if (s->current->dma_len == 0) {
         s->current->dma_buf = NULL;
         scsi_req_continue(s->current->req);
     } else {
         s->current->dma_buf += count;
         ncr53c710_resume_script(s);
     }
 }
 
 /* Add a command to the queue */
 static void ncr53c710_queue_command(NCR53C710State *s)
 {
     NCR53C710Request *p = s->current;
 
     DPRINTF("Queueing tag=0x%x\n", p->tag);
     assert(s->current != NULL);
     assert(s->current->dma_len == 0);
     
     QTAILQ_INSERT_TAIL(&s->queue, s->current, next);
     s->current = NULL;
 
     p->pending = 0;
     p->out = (s->sstat2 & NCR53C710_PHASE_MASK) == NCR53C710_PHASE_DO;
 }
 
 /* Queue a byte for a MSG IN phase */
 static void ncr53c710_add_msg_byte(NCR53C710State *s, uint8_t data)
 {
     if (s->msg_len >= NCR53C710_MAX_MSGIN_LEN) {
         BADF("MSG IN data too long\n");
     } else {
         DPRINTF("MSG IN 0x%02x\n", data);
         s->msg[s->msg_len++] = data;
     }
 }
 
 /* Perform reselection to continue a command */
 static void ncr53c710_reselect(NCR53C710State *s, NCR53C710Request *p)
 {
     int id;
 
     assert(s->current == NULL);
     QTAILQ_REMOVE(&s->queue, p, next);
     s->current = p;
 
     id = (p->tag >> 8) & 0xf;
     /* LSI53C700 Family Compatibility, see NCR53C710 4-73 */
     if (!(s->dcntl & NCR53C710_DCNTL_COM)) {
         s->sfbr = 1 << (id & 0x7);
     }
     s->lcrc = 0;
     
     DPRINTF("Reselected target %d\n", id);
     s->scntl1 |= NCR53C710_SCNTL1_CON;
     ncr53c710_set_phase(s, NCR53C710_PHASE_MI);
     s->msg_action = p->out ? 2 : 3;
     s->current->dma_len = p->pending;
     
     ncr53c710_add_msg_byte(s, 0x80); /* Identify message */
     if (s->current->tag & NCR53C710_TAG_VALID) {
         ncr53c710_add_msg_byte(s, 0x20); /* Simple queue tag */
         ncr53c710_add_msg_byte(s, p->tag & 0xff);
     }
 
     ncr53c710_script_scsi_interrupt(s, NCR53C710_SSTAT0_SEL);
 }
 
 /* Find a request by tag */
 static NCR53C710Request *ncr53c710_find_by_tag(NCR53C710State *s, uint32_t tag)
 {
     NCR53C710Request *p;
 
     QTAILQ_FOREACH(p, &s->queue, next) {
         if (p->tag == tag) {
             return p;
         }
     }
 
     return NULL;
 }
 
 /* Free a request */
 static void ncr53c710_request_free(NCR53C710State *s, NCR53C710Request *p)
 {
     if (p == s->current) {
         s->current = NULL;
     } else {
         QTAILQ_REMOVE(&s->queue, p, next);
     }
     g_free(p);
 }
 
 /* Cancel request callback */
 static void ncr53c710_request_cancelled(SCSIRequest *req)
 {
     NCR53C710SysBusDeviceState *m = NCR53C710(req->bus->qbus.parent);
     NCR53C710State *s = &m->state;
     NCR53C710Request *p = (NCR53C710Request *)req->hba_private;
 
     if (p) {
         req->hba_private = NULL;
         ncr53c710_request_free(s, p);
     }
     scsi_req_unref(req);
 }
 
 /* Queue a request for later execution */
 static int ncr53c710_queue_req(NCR53C710State *s, SCSIRequest *req, uint32_t len)
 {
     NCR53C710Request *p = (NCR53C710Request *)req->hba_private;
 
     if (p->pending) {
         BADF("Multiple IO pending for request %p\n", p);
     }
     p->pending = len;
     
     /* Reselect if waiting for it, or if bus is free with no pending interrupts */
     if (s->waiting == 1 ||
         (!(s->scntl1 & NCR53C710_SCNTL1_CON) &&
          !(s->istat & (NCR53C710_ISTAT_SIP | NCR53C710_ISTAT_DIP)))) {
         /* Reselect device */
         ncr53c710_reselect(s, p);
         return 0;
     } else {
         DPRINTF("Queueing IO tag=0x%x\n", p->tag);
         p->pending = len;
         return 1;
     }
 }
 
 /* Command complete callback */
 static void ncr53c710_command_complete(SCSIRequest *req, uint32_t status, size_t resid)
 {
     NCR53C710SysBusDeviceState *m = NCR53C710(req->bus->qbus.parent);
     NCR53C710State *s = &m->state;
     int out;
 
     out = (s->sstat2 & NCR53C710_PHASE_MASK) == NCR53C710_PHASE_DO;
     DPRINTF("Command complete status=%d\n", (int)status);
     s->lcrc = 0;
     s->status = status;
     s->command_complete = 2;
     
     if (s->waiting && s->dbc != 0) {
         /* Raise phase mismatch for short transfers */
         ncr53c710_bad_phase(s, out, NCR53C710_PHASE_ST);
     } else {
         ncr53c710_set_phase(s, NCR53C710_PHASE_ST);
     }
 
     if (req->hba_private == s->current) {
         req->hba_private = NULL;
         ncr53c710_request_free(s, s->current);
         scsi_req_unref(req);
     }
     ncr53c710_resume_script(s);
 }
 
 /* Transfer data callback */
 static void ncr53c710_transfer_data(SCSIRequest *req, uint32_t len)
 {
     NCR53C710SysBusDeviceState *m = NCR53C710(req->bus->qbus.parent);
     NCR53C710State *s = &m->state;
     int out;
 
     assert(req->hba_private);
     if (s->waiting == 1 || req->hba_private != s->current ||
         (!(s->scntl1 & NCR53C710_SCNTL1_CON))) {
         if (ncr53c710_queue_req(s, req, len)) {
             return;
         }
     }
 
     out = (s->sstat2 & NCR53C710_PHASE_MASK) == NCR53C710_PHASE_DO;
 
     /* Host adapter (re)connected */
     DPRINTF("Data ready tag=0x%x len=%d\n", req->tag, len);
     s->current->dma_len = len;
     s->command_complete = 1;
     
     if (s->waiting) {
         if (s->waiting == 1 || s->dbc == 0) {
             ncr53c710_resume_script(s);
         } else {
             ncr53c710_do_dma(s, out);
         }
     }
 }
 
 /* Execute SCSI command */
 static void ncr53c710_do_command(NCR53C710State *s)
 {
     SCSIDevice *dev;
     uint8_t buf[16];
     uint32_t id;
     int n;
 
     DPRINTF("Send command len=%d\n", s->dbc);
     if (s->dbc > 16) {
         s->dbc = 16;
     }
     ncr53c710_dma_read(s, s->dnad, buf, s->dbc);
     DPRINTF("Send command len=%d %02x.%02x.%02x.%02x.%02x.%02x\n", 
             s->dbc, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);
     s->sfbr = buf[0];
     s->command_complete = 0;
 
     id = (s->select_tag >> 8) & 0xff;
     s->lcrc = id; /* Store target ID */
     dev = scsi_device_find(&s->bus, 0, ncr53c710_id_to_num(id), s->current_lun);
     if (!dev) {
         ncr53c710_bad_selection(s, id);
         return;
     }
 
     assert(s->current == NULL);
     s->current = g_new0(NCR53C710Request, 1);
     s->current->tag = s->select_tag;
     s->current->req = scsi_req_new(dev, s->current->tag, 
                                    s->current_lun, buf, s->current);
 
     n = scsi_req_enqueue(s->current->req);
     if (n) {
         if (n > 0) {
             ncr53c710_set_phase(s, NCR53C710_PHASE_DI);
         } else if (n < 0) {
             ncr53c710_set_phase(s, NCR53C710_PHASE_DO);
         }
         scsi_req_continue(s->current->req);
     }
     
     if (!s->command_complete) {
         if (n) {
             /* Command did not complete immediately so disconnect */
             ncr53c710_add_msg_byte(s, 2); /* SAVE DATA POINTER */
             ncr53c710_add_msg_byte(s, 4); /* DISCONNECT */
             /* Wait data */
             ncr53c710_set_phase(s, NCR53C710_PHASE_MI);
             s->msg_action = 1;
             ncr53c710_queue_command(s);
         } else {
             /* Wait command complete */
             ncr53c710_set_phase(s, NCR53C710_PHASE_DI);
         }
     }
 }
 
 /* Handle status phase */
 static void ncr53c710_do_status(NCR53C710State *s)
 {
     uint8_t status;
     DPRINTF("Get status len=%d status=%d\n", s->dbc, s->status);
     if (s->dbc != 1) {
         BADF("Bad Status move\n");
     }
     s->dbc = 1;
     status = s->status;
     s->sfbr = status;
     ncr53c710_dma_write(s, s->dnad, &status, 1);
     ncr53c710_set_phase(s, NCR53C710_PHASE_MI);
     s->msg_action = 1;
     ncr53c710_add_msg_byte(s, 0); /* COMMAND COMPLETE */
 }
 
 /* Handle message in phase */
 static void ncr53c710_do_msgin(NCR53C710State *s)
 {
     int len;
     DPRINTF("Message in len=%d/%d\n", s->dbc, s->msg_len);
     s->sfbr = s->msg[0];
     len = s->msg_len;
     if (len > s->dbc) {
         len = s->dbc;
     }
     ncr53c710_dma_write(s, s->dnad, s->msg, len);
     /* Linux drivers rely on the last byte being in the SIDL */
     s->sidl = s->msg[len - 1];
     s->msg_len -= len;
     
     if (s->msg_len) {
         memmove(s->msg, s->msg + len, s->msg_len);
     } else {
         /* Switch to next phase based on msg_action */
         switch (s->msg_action) {
         case 0:
             ncr53c710_set_phase(s, NCR53C710_PHASE_CMD);
             break;
         case 1:
             ncr53c710_disconnect(s);
             break;
         case 2:
             ncr53c710_set_phase(s, NCR53C710_PHASE_DO);
             break;
         case 3:
             ncr53c710_set_phase(s, NCR53C710_PHASE_DI);
             break;
         default:
             abort();
         }
     }
 }
 
 /* Read the next byte during a MSGOUT phase */
 static uint8_t ncr53c710_get_msgbyte(NCR53C710State *s)
 {
     uint8_t data;
     ncr53c710_dma_read(s, s->dnad, &data, 1);
     s->dnad++;
     s->dbc--;
     return data;
 }
 
 /* Skip the next n bytes during a MSGOUT phase */
 static void ncr53c710_skip_msgbytes(NCR53C710State *s, unsigned int n)
 {
     s->dnad += n;
     s->dbc  -= n;
 }
 
 /* Handle message out phase */
 static void ncr53c710_do_msgout(NCR53C710State *s)
 {
     uint8_t msg;
     int len;
     uint32_t current_tag;
     NCR53C710Request *current_req, *p, *p_next;
 
     if (s->current) {
         current_tag = s->current->tag;
         current_req = s->current;
     } else {
         current_tag = s->select_tag;
         current_req = ncr53c710_find_by_tag(s, current_tag);
     }
 
     DPRINTF("MSG out len=%d\n", s->dbc);
     while (s->dbc) {
         msg = ncr53c710_get_msgbyte(s);
         s->sfbr = msg;
 
         switch (msg) {
         case 0x04:
             DPRINTF("MSG: Disconnect\n");
             ncr53c710_disconnect(s);
             break;
         case 0x08:
             DPRINTF("MSG: No Operation\n");
             ncr53c710_set_phase(s, NCR53C710_PHASE_CMD);
             break;
         case 0x01:
             len = ncr53c710_get_msgbyte(s);
             msg = ncr53c710_get_msgbyte(s);
             DPRINTF("Extended message 0x%x (len %d)\n", msg, len);
             switch (msg) {
             case 1:
                 DPRINTF("SDTR (ignored)\n");
                 ncr53c710_skip_msgbytes(s, 2);
                 break;
             case 3:
                 DPRINTF("WDTR (ignored)\n");
                 ncr53c710_skip_msgbytes(s, 1);
                 break;
             default:
                 goto bad;
             }
             break;
         case 0x20: /* SIMPLE queue */
             s->select_tag |= ncr53c710_get_msgbyte(s) | NCR53C710_TAG_VALID;
             DPRINTF("SIMPLE queue tag=0x%x\n", s->select_tag & 0xff);
             break;
         case 0x21: /* HEAD of queue */
             BADF("HEAD queue not implemented\n");
             s->select_tag |= ncr53c710_get_msgbyte(s) | NCR53C710_TAG_VALID;
             break;
         case 0x22: /* ORDERED queue */
             BADF("ORDERED queue not implemented\n");
             s->select_tag |= ncr53c710_get_msgbyte(s) | NCR53C710_TAG_VALID;
             break;
         case 0x0d:
             /* The ABORT TAG message clears the current I/O process only */
             DPRINTF("MSG: ABORT TAG tag=0x%x\n", current_tag);
             if (current_req) {
                 scsi_req_cancel(current_req->req);
             }
             ncr53c710_disconnect(s);
             break;
         case 0x06:
         case 0x0e:
         case 0x0c:
             /* ABORT/CLEAR QUEUE/BUS DEVICE RESET messages */
             if (msg == 0x06) {
                 DPRINTF("MSG: ABORT tag=0x%x\n", current_tag);
             } else if (msg == 0x0e) {
                 DPRINTF("MSG: CLEAR QUEUE tag=0x%x\n", current_tag);
             } else if (msg == 0x0c) {
                 DPRINTF("MSG: BUS DEVICE RESET tag=0x%x\n", current_tag);
             }
 
             /* Clear the current I/O process */
             if (s->current) {
                 scsi_req_cancel(s->current->req);
             }
 
             /* Clear all queued commands for the current device */
             QTAILQ_FOREACH_SAFE(p, &s->queue, next, p_next) {
                 if ((p->tag & 0x0000ff00) == (current_tag & 0x0000ff00)) {
                     scsi_req_cancel(p->req);
                 }
             }
 
             ncr53c710_disconnect(s);
             break;
         default:
             if ((msg & 0x80) == 0) {
                 goto bad;
             }
             s->current_lun = msg & 7;
             DPRINTF("Select LUN %d\n", s->current_lun);
             ncr53c710_set_phase(s, NCR53C710_PHASE_CMD);
             break;
         }
     }
     return;
 
 bad:
     BADF("Unimplemented message 0x%02x\n", msg);
     ncr53c710_set_phase(s, NCR53C710_PHASE_MI);
     ncr53c710_add_msg_byte(s, 7); /* MESSAGE REJECT */
     s->msg_action = 0;
 }
 
 /* Memory copy operation */
 #define NCR53C710_BUF_SIZE 4096
 static void ncr53c710_memcpy(NCR53C710State *s, uint32_t dest, uint32_t src, int count)
 {
     int n;
     uint8_t buf[NCR53C710_BUF_SIZE];
 
     DPRINTF("memcpy dest 0x%08x src 0x%08x count %d\n", dest, src, count);
     while (count) {
         n = (count > NCR53C710_BUF_SIZE) ? NCR53C710_BUF_SIZE : count;
         ncr53c710_dma_read(s, src, buf, n);
         ncr53c710_dma_write(s, dest, buf, n);
         src += n;
         dest += n;
         count -= n;
     }
 }
 
 /* Wait for reselection state */
 static void ncr53c710_wait_reselect(NCR53C710State *s)
 {
     NCR53C710Request *p;
 
     DPRINTF("Wait Reselect\n");
 
     QTAILQ_FOREACH(p, &s->queue, next) {
         if (p->pending) {
             ncr53c710_reselect(s, p);
             break;
         }
     }
     if (s->current == NULL) {
         s->waiting = 1;
     }
 }
 
 /* Soft reset of the chip */
 static void ncr53c710_soft_reset(NCR53C710State *s)
 {
     DPRINTF("Soft reset\n");
     s->carry = 0;
     s->msg_action = 0;
     s->msg_len = 0;
     s->waiting = 0;
     s->dsa = 0;
     s->dnad = 0;
     s->dbc = 0;
     s->temp = 0;
     s->scratch = 0;
     /* Reset bit does not reset */
     s->istat &= 0x40;
     s->dcmd = 0x40;
     s->dstat = NCR53C710_DSTAT_DFE;
     s->dien = 0;
     s->sien0 = 0;
     s->ctest2 = 0;
     s->ctest3 = 0;
     s->ctest4 = 0;
     s->ctest5 = 0;
     s->dsp = 0;
     s->dsps = 0;
     s->dmode = 0;
     s->dcntl = 0;
     s->scntl0 = 0xc0;
     s->scntl1 = 0;
     s->sstat0 = 0;
     s->sstat1 = 0;
     s->sstat2 = 0;
     s->scid = 0x80;
     s->sxfer = 0;
     s->socl = 0;
     s->sdid = 0;
     s->sidl = 0;
     s->sbc = 0;
     s->sbr = 0;
     s->ctest0 = 0;
     s->ctest1 = 0;
     s->ctest6 = 0;
     s->ctest7 = 0;
     s->ctest8 = 0;
     s->lcrc = 0;
     s->dwt = 0;
     s->sbcl = 0;
     s->script_active = 0;
 
     assert(!s->current);
 }
 
 /* Register read handlers */
 static uint8_t ncr53c710_reg_readb(NCR53C710State *s, int offset)
 {
     uint8_t val;
     
 #ifdef DEBUG_NCR53C710_REG
     DPRINTF("Read reg %02x\n", offset);
 #endif
     
     switch (offset) {
     case 0x00: /* SCNTL0: SCSI control 0 */
         val = s->scntl0;
         break;
     case 0x01: /* SCNTL1: SCSI control 1 */
         val = s->scntl1;
         break;
     case 0x02: /* SDID: SCSI destination ID */
         val = s->sdid;
         break;
     case 0x03: /* SIEN: SCSI interrupt enable */
         val = s->sien0;
         break;
     case 0x04: /* SCID: SCSI chip ID */
         val = s->scid;
         break;
     case 0x05: /* SXFER: SCSI transfer */
         val = s->sxfer;
         break;
     case 0x06: /* SODL: SCSI output data latch */
         val = 0;
         break;
     case 0x07: /* SOCL: SCSI output control latch */
         val = s->socl;
         break;
     case 0x08: /* SFBR: SCSI first byte received */
         val = s->sfbr;
         break;
     case 0x09: /* SIDL: SCSI input data latch */
         val = s->sidl;
         break;
     case 0x0a: /* SBDL: SCSI bus data lines */
         /* In real hardware this reads the actual SCSI bus values.
          * We just return 0 as we don't implement the bus details.
          */
         val = 0;
         break;
     case 0x0b: /* SBCL: SCSI bus control lines */
         /* Similar to SBDL, we'd read the actual control line status here */
         val = s->sstat2 & 0x07; /* Return phase bits */
         if (s->scntl1 & NCR53C710_SCNTL1_CON) {
             val |= s->sbcl;
             if (s->socl & NCR53C710_SOCL_ATN) {
                 val |= NCR53C710_SBCL_ATN;
             }
         }
         break;
     case 0x0c: /* DSTAT: DMA status */
         val = s->dstat | NCR53C710_DSTAT_DFE;
         s->dstat = 0; /* Register is cleared on read */
         ncr53c710_update_irq(s);
         break;
     case 0x0d: /* SSTAT0: SCSI status 0 */
         val = s->sstat0;
         s->sstat0 = 0; /* Register is cleared on read */
         ncr53c710_update_irq(s);
         break;
     case 0x0e: /* SSTAT1: SCSI status 1 */
         val = s->sstat1;
         break;
     case 0x0f: /* SSTAT2: SCSI status 2 */
         val = s->sstat2;
         break;
     case 0x10: /* DSA: Data structure address (LSB) */
         val = s->dsa & 0xff;
         break;
     case 0x11: /* DSA: Data structure address */
         val = (s->dsa >> 8) & 0xff;
         break;
     case 0x12: /* DSA: Data structure address */
         val = (s->dsa >> 16) & 0xff;
         break;
     case 0x13: /* DSA: Data structure address (MSB) */
         val = (s->dsa >> 24) & 0xff;
         break;
     case 0x14: /* CTEST0: Chip test 0 */
         val = s->ctest0;
         break;
     case 0x15: /* CTEST1: Chip test 1 */
         val = 0xf0; /* FIFO empty */
         break;
     case 0x16: /* CTEST2: Chip test 2 */
         val = s->ctest2 | NCR53C710_CTEST2_DACK;
         if (s->istat & NCR53C710_ISTAT_SIGP) {
             s->istat &= ~NCR53C710_ISTAT_SIGP;
             val |= 0x40; /* SIGP bit */
         }
         break;
     case 0x17: /* CTEST3: Chip test 3 */
         val = s->ctest3;
         break;
     case 0x18: /* CTEST4: Chip test 4 */
         val = s->ctest4;
         break;
     case 0x19: /* CTEST5: Chip test 5 */
         val = s->ctest5;
         break;
     case 0x1a: /* CTEST6: Chip test 6 */
         val = s->ctest6;
         break;
     case 0x1b: /* CTEST7: Chip test 7 */
         val = s->ctest7;
         break;
     case 0x1c: /* TEMP: Temporary stack (LSB) */
         val = s->temp & 0xff;
         break;
     case 0x1d: /* TEMP: Temporary stack */
         val = (s->temp >> 8) & 0xff;
         break;
     case 0x1e: /* TEMP: Temporary stack */
         val = (s->temp >> 16) & 0xff;
         break;
     case 0x1f: /* TEMP: Temporary stack (MSB) */
         val = (s->temp >> 24) & 0xff;
         break;
     case 0x20: /* DFIFO: DMA FIFO */
         val = 0; /* Always empty in our emulation */
         break;
     case 0x21: /* ISTAT: Interrupt status */
         val = s->istat;
         break;
     case 0x22: /* CTEST8: Chip test 8 */
         val = s->ctest8;
         break;
     case 0x23: /* LCRC: Longitudinal parity */
         val = s->lcrc;
         break;
     case 0x24: /* DBC: DMA byte counter (LSB) */
         val = s->dbc & 0xff;
         break;
     case 0x25: /* DBC: DMA byte counter */
         val = (s->dbc >> 8) & 0xff;
         break;
     case 0x26: /* DBC: DMA byte counter (MSB) */
         val = (s->dbc >> 16) & 0xff;
         break;
     case 0x27: /* DCMD: DMA command */
         val = s->dcmd;
         break;
     case 0x28: /* DNAD: DMA next address for data (LSB) */
         val = s->dnad & 0xff;
         break;
     case 0x29: /* DNAD: DMA next address for data */
         val = (s->dnad >> 8) & 0xff;
         break;
     case 0x2a: /* DNAD: DMA next address for data */
         val = (s->dnad >> 16) & 0xff;
         break;
     case 0x2b: /* DNAD: DMA next address for data (MSB) */
         val = (s->dnad >> 24) & 0xff;
         break;
     case 0x2c: /* DSP: DMA SCRIPTS pointer (LSB) */
         val = s->dsp & 0xff;
         break;
     case 0x2d: /* DSP: DMA SCRIPTS pointer */
         val = (s->dsp >> 8) & 0xff;
         break;
     case 0x2e: /* DSP: DMA SCRIPTS pointer */
         val = (s->dsp >> 16) & 0xff;
         break;
     case 0x2f: /* DSP: DMA SCRIPTS pointer (MSB) */
         val = (s->dsp >> 24) & 0xff;
         break;
     case 0x30: /* DSPS: DMA SCRIPTS pointer save (LSB) */
         val = s->dsps & 0xff;
         break;
     case 0x31: /* DSPS: DMA SCRIPTS pointer save */
         val = (s->dsps >> 8) & 0xff;
         break;
     case 0x32: /* DSPS: DMA SCRIPTS pointer save */
         val = (s->dsps >> 16) & 0xff;
         break;
     case 0x33: /* DSPS: DMA SCRIPTS pointer save (MSB) */
         val = (s->dsps >> 24) & 0xff;
         break;
     case 0x34: /* SCRATCH: General purpose scratch pad (LSB) */
         val = s->scratch & 0xff;
         break;
     case 0x35: /* SCRATCH: General purpose scratch pad */
         val = (s->scratch >> 8) & 0xff;
         break;
     case 0x36: /* SCRATCH: General purpose scratch pad */
         val = (s->scratch >> 16) & 0xff;
         break;
     case 0x37: /* SCRATCH: General purpose scratch pad (MSB) */
         val = (s->scratch >> 24) & 0xff;
         break;
     case 0x38: /* DMODE: DMA mode */
         val = s->dmode;
         break;
     case 0x39: /* DIEN: DMA interrupt enable */
         val = s->dien;
         break;
     case 0x3a: /* DWT: DMA watchdog timer */
         val = s->dwt;
         break;
     case 0x3b: /* DCNTL: DMA control */
         val = s->dcntl;
         break;
     case 0x3c: /* ADDER: Sum output of internal adder (LSB) */
     case 0x3d: /* ADDER: Sum output of internal adder */
     case 0x3e: /* ADDER: Sum output of internal adder */
     case 0x3f: /* ADDER: Sum output of internal adder (MSB) */
         /* This is a read-only register showing the result of various operations */
         val = 0;
         break;
     default:
         BADF("Read from unknown register offset 0x%x\n", offset);
         val = 0;
         break;
     }
     
     return val;
 }
 
 /* Register write handlers */
 static void ncr53c710_reg_writeb(NCR53C710State *s, int offset, uint8_t val)
 {
 #ifdef DEBUG_NCR53C710_REG
     DPRINTF("Write reg %02x = %02x\n", offset, val);
 #endif
 
     switch (offset) {
     case 0x00: /* SCNTL0: SCSI control 0 */
         s->scntl0 = val;
         if (val & NCR53C710_SCNTL0_START) {
             DPRINTF("Start sequence requested\n");
             /* On real hardware, this would initiate a selection sequence */
         }
         break;
     case 0x01: /* SCNTL1: SCSI control 1 */
         s->scntl1 = val;
         if (val & NCR53C710_SCNTL1_RST) {
             if (!(s->sstat0 & NCR53C710_SSTAT0_RST)) {
                 /* Assert SCSI bus reset */
                 /* In a real system, this would reset all SCSI devices on the bus */
                 s->sstat0 |= NCR53C710_SSTAT0_RST;
                 ncr53c710_script_scsi_interrupt(s, NCR53C710_SSTAT0_RST);
             }
         } else {
             s->sstat0 &= ~NCR53C710_SSTAT0_RST;
         }
         break;
     case 0x02: /* SDID: SCSI destination ID */
         s->sdid = val;
         break;
     case 0x03: /* SIEN: SCSI interrupt enable */
         s->sien0 = val;
         ncr53c710_update_irq(s);
         break;
     case 0x04: /* SCID: SCSI chip ID */
         s->scid = val;
         break;
     case 0x05: /* SXFER: SCSI transfer */
         s->sxfer = val;
         break;
     case 0x06: /* SODL: SCSI output data latch */
         /* This register is used to manually send data on the SCSI bus */
         /* We don't emulate the low-level SCSI bus so this is a no-op */
         break;
     case 0x07: /* SOCL: SCSI output control latch */
         s->socl = val;
         break;
     case 0x08: /* SFBR: SCSI first byte received */
         /* This is typically read-only, but can be written by SCRIPTS */
         s->sfbr = val;
         break;
     case 0x09: /* SIDL: SCSI input data latch */
     case 0x0a: /* SBDL: SCSI bus data lines */
     case 0x0b: /* SBCL: SCSI bus control lines */
         /* These are read-only status registers */
         break;
     case 0x0c: /* DSTAT: DMA status */
     case 0x0d: /* SSTAT0: SCSI status 0 */
     case 0x0e: /* SSTAT1: SCSI status 1 */
     case 0x0f: /* SSTAT2: SCSI status 2 */
         /* These are read-only status registers */
         break;
     case 0x10: /* DSA: Data structure address (LSB) */
         s->dsa = (s->dsa & ~0xff) | val;
         break;
     case 0x11: /* DSA: Data structure address */
         s->dsa = (s->dsa & ~0xff00) | (val << 8);
         break;
     case 0x12: /* DSA: Data structure address */
         s->dsa = (s->dsa & ~0xff0000) | (val << 16);
         break;
     case 0x13: /* DSA: Data structure address (MSB) */
         s->dsa = (s->dsa & ~0xff000000) | (val << 24);
         break;
     case 0x14: /* CTEST0: Chip test 0 */
         s->ctest0 = (val & 0xfe) | (s->ctest0 & 1);
         break;
     case 0x15: /* CTEST1: Chip test 1 */
         /* Read-only register */
         break;
     case 0x16: /* CTEST2: Chip test 2 */
         /* Read-only register */
         break;
     case 0x17: /* CTEST3: Chip test 3 */
         s->ctest3 = val;
         break;
     case 0x18: /* CTEST4: Chip test 4 */
         s->ctest4 = val;
         break;
     case 0x19: /* CTEST5: Chip test 5 */
         s->ctest5 = val;
         break;
     case 0x1a: /* CTEST6: Chip test 6 */
         s->ctest6 = val;
         break;
     case 0x1b: /* CTEST7: Chip test 7 */
         s->ctest7 = val;
         break;
     case 0x1c: /* TEMP: Temporary stack (LSB) */
         s->temp = (s->temp & ~0xff) | val;
         break;
     case 0x1d: /* TEMP: Temporary stack */
         s->temp = (s->temp & ~0xff00) | (val << 8);
         break;
     case 0x1e: /* TEMP: Temporary stack */
         s->temp = (s->temp & ~0xff0000) | (val << 16);
         break;
     case 0x1f: /* TEMP: Temporary stack (MSB) */
         s->temp = (s->temp & ~0xff000000) | (val << 24);
         break;
     case 0x20: /* DFIFO: DMA FIFO */
         /* Writing to this register clears the FIFO */
         break;
     case 0x21: /* ISTAT: Interrupt status */
         s->istat = (s->istat & 0x0f) | (val & 0xf0);
         if (val & NCR53C710_ISTAT_ABRT) {
             ncr53c710_script_dma_interrupt(s, NCR53C710_DSTAT_ABRT);
         }
         if (s->waiting == 1 && (val & NCR53C710_ISTAT_SIGP)) {
             DPRINTF("Woken by SIGP\n");
             s->waiting = 0;
             s->dsp = s->dnad;
             ncr53c710_execute_script(s);
         }
         if (val & NCR53C710_ISTAT_RST) {
             ncr53c710_soft_reset(s);
         }
         break;
     case 0x22: /* CTEST8: Chip test 8 */
         s->ctest8 = val;
         break;
     case 0x23: /* LCRC: Longitudinal parity */
         s->lcrc = val;
         break;
     case 0x24: /* DBC: DMA byte counter (LSB) */
         s->dbc = (s->dbc & ~0xff) | val;
         break;
     case 0x25: /* DBC: DMA byte counter */
         s->dbc = (s->dbc & ~0xff00) | (val << 8);
         break;
     case 0x26: /* DBC: DMA byte counter (MSB) */
         s->dbc = (s->dbc & ~0xff0000) | (val << 16);
         break;
     case 0x27: /* DCMD: DMA command */
         s->dcmd = val;
         break;
     case 0x28: /* DNAD: DMA next address for data (LSB) */
         s->dnad = (s->dnad & ~0xff) | val;
         break;
     case 0x29: /* DNAD: DMA next address for data */
         s->dnad = (s->dnad & ~0xff00) | (val << 8);
         break;
     case 0x2a: /* DNAD: DMA next address for data */
         s->dnad = (s->dnad & ~0xff0000) | (val << 16);
         break;
     case 0x2b: /* DNAD: DMA next address for data (MSB) */
         s->dnad = (s->dnad & ~0xff000000) | (val << 24);
         break;
     case 0x2c: /* DSP: DMA SCRIPTS pointer (LSB) */
         s->dsp = (s->dsp & ~0xff) | val;
         break;
     case 0x2d: /* DSP: DMA SCRIPTS pointer */
         s->dsp = (s->dsp & ~0xff00) | (val << 8);
         break;
     case 0x2e: /* DSP: DMA SCRIPTS pointer */
         s->dsp = (s->dsp & ~0xff0000) | (val << 16);
         break;
     case 0x2f: /* DSP: DMA SCRIPTS pointer (MSB) */
         s->dsp = (s->dsp & ~0xff000000) | (val << 24);
         if ((s->dmode & NCR53C710_DMODE_MAN) == 0) {
             s->waiting = 0;
             ncr53c710_execute_script(s);
         }
         break;
     case 0x30: /* DSPS: DMA SCRIPTS pointer save (LSB) */
         s->dsps = (s->dsps & ~0xff) | val;
         break;
     case 0x31: /* DSPS: DMA SCRIPTS pointer save */
         s->dsps = (s->dsps & ~0xff00) | (val << 8);
         break;
     case 0x32: /* DSPS: DMA SCRIPTS pointer save */
         s->dsps = (s->dsps & ~0xff0000) | (val << 16);
         break;
     case 0x33: /* DSPS: DMA SCRIPTS pointer save (MSB) */
         s->dsps = (s->dsps & ~0xff000000) | (val << 24);
         break;
     case 0x34: /* SCRATCH: General purpose scratch pad (LSB) */
         s->scratch = (s->scratch & ~0xff) | val;
         break;
     case 0x35: /* SCRATCH: General purpose scratch pad */
         s->scratch = (s->scratch & ~0xff00) | (val << 8);
         break;
     case 0x36: /* SCRATCH: General purpose scratch pad */
         s->scratch = (s->scratch & ~0xff0000) | (val << 16);
         break;
     case 0x37: /* SCRATCH: General purpose scratch pad (MSB) */
         s->scratch = (s->scratch & ~0xff000000) | (val << 24);
         break;
     case 0x38: /* DMODE: DMA mode */
         s->dmode = val;
         break;
     case 0x39: /* DIEN: DMA interrupt enable */
         s->dien = val;
         ncr53c710_update_irq(s);
         break;
     case 0x3a: /* DWT: DMA watchdog timer */
         s->dwt = val;
         break;
     case 0x3b: /* DCNTL: DMA control */
         s->dcntl = val & ~(NCR53C710_DCNTL_PFF | NCR53C710_DCNTL_STD);
         if ((val & NCR53C710_DCNTL_STD) && (s->dmode & NCR53C710_DMODE_MAN))
             ncr53c710_execute_script(s);
         break;
     case 0x3c: /* ADDER: Sum output of internal adder (LSB) */
     case 0x3d: /* ADDER: Sum output of internal adder */
     case 0x3e: /* ADDER: Sum output of internal adder */
     case 0x3f: /* ADDER: Sum output of internal adder (MSB) */
         /* Read-only registers */
         break;
     default:
         BADF("Write to unknown register offset 0x%x\n", offset);
         break;
     }
 }
 
 /* The core SCRIPTS execution engine */
 static void ncr53c710_execute_script(NCR53C710State *s)
 {
     uint32_t insn;
     uint32_t addr, addr_high;
     int opcode;
     int insn_processed = 0;
 
     s->script_active = 1;
 again:
     insn_processed++;
     if (insn_processed > 10000 && !s->waiting) {
         /* Some Windows drivers cause infinite loops. Break out after 
            too many instructions to avoid hanging the emulator. */
         DPRINTF("NCR53C710: Too many instructions executed, breaking loop\n");
         ncr53c710_script_scsi_interrupt(s, NCR53C710_SSTAT0_UDC);
         ncr53c710_disconnect(s);
         return;
     }
 
     /* Fetch the instruction */
     insn = ncr53c710_read_dword(s, s->dsp);
     addr = ncr53c710_read_dword(s, s->dsp + 4);
     DPRINTF("SCRIPT execution at dsp=%08x opcode %08x arg %08x\n", s->dsp, insn, addr);
     
     /* Store the address part in the DSPS register */
     s->dsps = addr;
     
     /* Store the command part in the DCMD register */
     s->dcmd = insn >> 24;
     
     /* Move the instruction pointer forward */
     s->dsp += 8;
     
     /* Process the instruction based on its type */
     switch (insn >> 30) {
     case 0: /* Block move instruction */
         if (s->sstat0 & NCR53C710_SSTAT0_STO) {
             DPRINTF("Delayed select timeout\n");
             ncr53c710_stop_script(s);
             break;
         }
         
         /* Get the byte count from the lower 24 bits */
         s->dbc = insn & 0xffffff;
         
         /* Handle indirect addressing modes */
         if (insn & (1 << 29)) {
             /* Indirect addressing */
             addr = ncr53c710_read_dword(s, addr);
         } else if (insn & (1 << 28)) {
             /* Table indirect addressing */
             uint32_t buf[2];
             int32_t offset;
             
             /* 32-bit Table indirect */
             offset = sextract32(addr, 0, 24);
             ncr53c710_dma_read(s, s->dsa + offset, buf, 8);
             /* Byte count is stored in bits 0:23 only */
             s->dbc = le32_to_cpu(buf[0]) & 0xffffff;
             addr = le32_to_cpu(buf[1]);
         }
         
         /* Check if the phase matches what's expected */
         if ((s->sstat2 & PHASE_MASK) != ((insn >> 24) & 7)) {
             DPRINTF("Phase mismatch - got %d, expected %d\n", 
                    s->sstat2 & PHASE_MASK, (insn >> 24) & 7);
             ncr53c710_script_scsi_interrupt(s, NCR53C710_SSTAT0_MA);
             s->sbcl |= NCR53C710_SBCL_REQ;
             break;
         }
         
         /* Set up the DMA address */
         s->dnad = addr;
         
         /* Process based on current phase */
         switch (s->sstat2 & PHASE_MASK) {
         case PHASE_DO:
             s->waiting = 2;
             ncr53c710_do_dma(s, 1);
             if (s->waiting)
                 s->waiting = 3;
             break;
         case PHASE_DI:
             s->waiting = 2;
             ncr53c710_do_dma(s, 0);
             if (s->waiting)
                 s->waiting = 3;
             break;
         case PHASE_CMD:
             ncr53c710_do_command(s);
             break;
         case PHASE_ST:
             ncr53c710_do_status(s);
             break;
         case PHASE_MO:
             ncr53c710_do_msgout(s);
             break;
         case PHASE_MI:
             ncr53c710_do_msgin(s);
             break;
         default:
             BADF("Unimplemented phase %d\n", s->sstat2 & PHASE_MASK);
         }
         
         /* Update the byte counter registers */
         s->ctest5 = (s->ctest5 & 0xfc) | ((s->dbc >> 8) & 3);
         s->sbc = s->dbc;
         break;
         
     case 1: /* I/O or R/W instruction */
         opcode = (insn >> 27) & 7;
         if (opcode < 5) {
             uint32_t id;
             
             /* Get the ID from the instruction or indirectly */
             if (insn & (1 << 25)) {
                 id = ncr53c710_read_dword(s, s->dsa + sextract32(insn, 0, 24));
             } else {
                 id = insn;
             }
             id = (id >> 16) & 0xff;
             
             /* Handle relative addressing */
             if (insn & (1 << 26)) {
                 addr = s->dsp + sextract32(addr, 0, 24);
             }
             
             /* Set up the address */
             s->dnad = addr;
             
             /* Process the specific I/O operation */
             switch (opcode) {
             case 0: /* Select */
                 s->sdid = id;
                 if (s->scntl1 & NCR53C710_SCNTL1_CON) {
                     DPRINTF("Already reselected, jumping to alternative address\n");
                     s->dsp = s->dnad;
                     break;
                 }
                 s->sstat1 |= NCR53C710_SSTAT1_WOA;
                 
                 /* Check if the device exists */
                 if (!scsi_device_find(&s->bus, 0, ncr53c710_id_to_num(id), 0)) {
                     ncr53c710_bad_selection(s, id);
                     break;
                 }
                 
                 DPRINTF("Selected target ID %d%s\n",
                        id, insn & (1 << 24) ? " with ATN" : "");
                 
                 /* Record the selected device ID */
                 s->select_tag = id << 8;
                 s->scntl1 |= NCR53C710_SCNTL1_CON;
                 
                 /* Set the appropriate phase based on ATN condition */
                 if (insn & (1 << 24)) {
                     s->socl |= NCR53C710_SOCL_ATN;
                     ncr53c710_set_phase(s, PHASE_MO);
                 } else {
                     ncr53c710_set_phase(s, PHASE_CMD);
                 }
                 break;
                 
             case 1: /* Disconnect */
                 DPRINTF("Wait Disconnect\n");
                 s->scntl1 &= ~NCR53C710_SCNTL1_CON;
                 break;
                 
             case 2: /* Wait Reselect */
                 ncr53c710_wait_reselect(s);
                 break;
                 
             case 3: /* Set */
                 DPRINTF("Set%s%s%s%s\n",
                        insn & (1 << 3) ? " ATN" : "",
                        insn & (1 << 6) ? " ACK" : "",
                        insn & (1 << 9) ? " TM" : "",
                        insn & (1 << 10) ? " CC" : "");
                 
                 if (insn & (1 << 3)) {
                     s->socl |= NCR53C710_SOCL_ATN;
                     ncr53c710_set_phase(s, PHASE_MO);
                 }
                 
                 if (insn & (1 << 9)) {
                     BADF("Target mode not implemented\n");
                 }
                 
                 if (insn & (1 << 10))
                     s->carry = 1;
                 break;
                 
             case 4: /* Clear */
                 DPRINTF("Clear%s%s%s%s\n",
                        insn & (1 << 3) ? " ATN" : "",
                        insn & (1 << 6) ? " ACK" : "",
                        insn & (1 << 9) ? " TM" : "",
                        insn & (1 << 10) ? " CC" : "");
                 
                 if (insn & (1 << 3)) {
                     s->socl &= ~NCR53C710_SOCL_ATN;
                 }
                 
                 if (insn & (1 << 10))
                     s->carry = 0;
                 break;
             }
         } else {
             /* Register operations */
             uint8_t op0, op1, data8;
             int reg;
             int operator;
             
             /* Decode the instruction */
             reg = ((insn >> 16) & 0x7f) | (insn & 0x80);
             data8 = (insn >> 8) & 0xff;
             opcode = (insn >> 27) & 7;
             operator = (insn >> 24) & 7;
             
             DPRINTF("Register op: reg 0x%x, operator %d, data8=0x%02x, sfbr=0x%02x%s\n", 
                    reg, operator, data8, s->sfbr,
                    (insn & (1 << 23)) ? " SFBR" : "");
             
             /* Get operands based on instruction type */
             op0 = op1 = 0;
             switch (opcode) {
             case 5: /* From SFBR */
                 op0 = s->sfbr;
                 op1 = data8;
                 break;
             case 6: /* To SFBR */
                 if (operator)
                     op0 = ncr53c710_reg_readb(s, reg);
                 op1 = data8;
                 break;
             case 7: /* Read-modify-write */
                 if (operator)
                     op0 = ncr53c710_reg_readb(s, reg);
                 if (insn & (1 << 23)) {
                     op1 = s->sfbr;
                 } else {
                     op1 = data8;
                 }
                 break;
             }
             
             /* Perform the operation */
             switch (operator) {
             case 0: /* Move */
                 op0 = op1;
                 break;
             case 1: /* Shift left */
                 op1 = op0 >> 7;
                 op0 = (op0 << 1) | s->carry;
                 s->carry = op1;
                 break;
             case 2: /* OR */
                 op0 |= op1;
                 break;
             case 3: /* XOR */
                 op0 ^= op1;
                 break;
             case 4: /* AND */
                 op0 &= op1;
                 break;
             case 5: /* Shift right */
                 op1 = op0 & 1;
                 op0 = (op0 >> 1) | (s->carry << 7);
                 s->carry = op1;
                 break;
             case 6: /* ADD */
                 op0 += op1;
                 s->carry = op0 < op1;
                 break;
             case 7: /* ADC (add with carry) */
                 op0 += op1 + s->carry;
                 if (s->carry)
                     s->carry = op0 <= op1;
                 else
                     s->carry = op0 < op1;
                 break;
             }
             
             /* Store the result */
             switch (opcode) {
             case 5: /* From SFBR */
             case 7: /* Read-modify-write */
                 ncr53c710_reg_writeb(s, reg, op0);
                 break;
             case 6: /* To SFBR */
                 s->sfbr = op0;
                 break;
             }
         }
         break;
         
     case 2: /* Transfer Control (Jump/Call) */
         {
             int cond = 1;
             int jmp = (insn & (1 << 19)) != 0;
             
             if (insn == 0) {
                 /* Handle NOP (all zeroes) */
                 s->dsp += 0;
                 goto again;
             }
             
             if (s->sstat0 & NCR53C710_SSTAT0_STO) {
                 DPRINTF("Delayed select timeout\n");
                 ncr53c710_stop_script(s);
                 break;
             }
             
             /* Process the conditions */
             if ((insn & (1 << 21)) && (s->carry != jmp))
                 cond = 0;
             
             if ((insn & (1 << 17)) && 
                 ((s->sstat2 & PHASE_MASK) != ((insn >> 24) & 7)))
                 cond = 0;
             
             if ((insn & (1 << 18))) {
                 uint8_t mask = (~insn >> 8) & 0xff;
                 if ((s->sfbr & mask) != (insn & mask))
                     cond = 0;
             }
             
             /* If condition is true, process the control transfer */
             if (cond == jmp) {
                 if (insn & (1 << 23)) {
                     /* Relative address */
                     addr_high = s->dsp + sextract32(addr, 0, 24);
                 } else {
                     addr_high = addr;
                 }
                 
                 switch ((insn >> 27) & 7) {
                 case 0: /* Jump */
                     DPRINTF("Jump to 0x%08x\n", addr_high);
                     s->dsp = addr_high;
                     break;
                 case 1: /* Call */
                     DPRINTF("Call to 0x%08x\n", addr_high);
                     s->temp = s->dsp;
                     s->dsp = addr_high;
                     break;
                 case 2: /* Return */
                     DPRINTF("Return to 0x%08x\n", s->temp);
                     s->dsp = s->temp;
                     break;
                 case 3: /* Interrupt */
                     DPRINTF("Generate interrupt 0x%08x\n", s->dsps);
                     if (insn & (1 << 20)) {
                         ncr53c710_update_irq(s);
                     } else {
                         ncr53c710_script_dma_interrupt(s, NCR53C710_DSTAT_SIR);
                     }
                     break;
                 default:
                     DPRINTF("Illegal transfer control\n");
                     ncr53c710_script_dma_interrupt(s, NCR53C710_DSTAT_IID);
                     break;
                 }
             } else {
                 DPRINTF("Condition failed for control transfer\n");
             }
         }
         break;
         
     case 3: /* Memory Move or Load/Store */
         if ((insn & (1 << 29)) == 0) {
             /* Memory to memory move */
             uint32_t dest;
             
             /* Get the destination address */
             dest = ncr53c710_read_dword(s, s->dsp);
             s->dsp += 4;
             
             /* Perform the memory copy */
             ncr53c710_memcpy(s, dest, addr, insn & 0xffffff);
         } else {
             /* Memory to/from register transfer */
             uint8_t data[7];
             int reg;
             int n;
             int i;
             
             /* Handle indirect addressing */
             if (insn & (1 << 28)) {
                 addr = s->dsa + sextract32(addr, 0, 24);
             }
             
             n = (insn & 7);
             reg = (insn >> 16) & 0xff;
             
             if (insn & (1 << 24)) {
                 /* Load - Memory to Register */
                 ncr53c710_dma_read(s, addr, data, n);
                 DPRINTF("Load reg 0x%x size %d addr 0x%08x\n", reg, n, addr);
                 for (i = 0; i < n; i++) {
                     ncr53c710_reg_writeb(s, reg + i, data[i]);
                 }
             } else {
                 /* Store - Register to Memory */
                 DPRINTF("Store reg 0x%x size %d addr 0x%08x\n", reg, n, addr);
                 for (i = 0; i < n; i++) {
                     data[i] = ncr53c710_reg_readb(s, reg + i);
                 }
                 ncr53c710_dma_write(s, addr, data, n);
             }
         }
         break;
     }
     
     /* Continue execution if script is active and not waiting */
     if (s->script_active && !s->waiting) {
         if (s->dcntl & NCR53C710_DCNTL_SSM) {
             /* Single-step mode - generate interrupt */
             ncr53c710_script_dma_interrupt(s, NCR53C710_DSTAT_SSI);
         } else {
             goto again;
         }
     }
     
     DPRINTF("SCRIPTS execution stopped\n");
 }
 
 /* Memory region operations for register access */
 static uint64_t ncr53c710_mmio_read(void *opaque, hwaddr addr, unsigned size)
 {
     NCR53C710SysBusDeviceState *s = opaque;
     uint8_t val;
     
     if (size != 1) {
         return 0;
     }
     
     val = ncr53c710_reg_readb(&s->state, addr);
     
 #ifdef DEBUG_NCR53C710_REG
     DPRINTF("Read reg %02x = %02x\n", (unsigned)addr, val);
 #endif
     
     return val;
 }
 
 static void ncr53c710_mmio_write(void *opaque, hwaddr addr, uint64_t val,
                              unsigned size)
 {
     NCR53C710SysBusDeviceState *s = opaque;
     
     if (size != 1) {
         return;
     }
     
 #ifdef DEBUG_NCR53C710_REG
     DPRINTF("Write reg %02x = %02x\n", (unsigned)addr, (uint8_t)val);
 #endif
     
     ncr53c710_reg_writeb(&s->state, addr, val);
 }
 
 static const MemoryRegionOps ncr53c710_mmio_ops = {
     .read = ncr53c710_mmio_read,
     .write = ncr53c710_mmio_write,
     .endianness = DEVICE_NATIVE_ENDIAN,
     .impl = {
         .min_access_size = 1,
         .max_access_size = 1,
     },
 };
 
 /* Initialize the SCSI device */
 static void ncr53c710_scsi_realize(SysBusDevice *sbd, Error **errp)
 {
     NCR53C710SysBusDeviceState *s = NCR53C710(sbd);
     DeviceState *dev = DEVICE(s);
     
     QTAILQ_INIT(&s->state.queue);
     
     memory_region_init_io(&s->mmio, OBJECT(s), &ncr53c710_mmio_ops,
                           s, "ncr53c710", 0x100);
     
     sysbus_init_mmio(sbd, &s->mmio);
     sysbus_init_irq(sbd, &s->state.irq);
     
     scsi_bus_new(&s->state.bus, sizeof(s->state.bus),
                  dev, &ncr53c710_scsi_bus_info, NULL);
 }
 
 static void ncr53c710_reset(DeviceState *dev)
 {
     NCR53C710SysBusDeviceState *s = NCR53C710(dev);
     
     DPRINTF("NCR53C710 Reset\n");
     ncr53c710_soft_reset(&s->state);
 }
 
 /* SCSI Bus Info */
 static const SCSIBusInfo ncr53c710_scsi_bus_info = {
     .tcq = true,
     .max_target = NCR53C710_MAX_DEVS,
     .max_lun = 7,
     
     .transfer_data = ncr53c710_transfer_data,
     .complete = ncr53c710_command_complete,
     .cancel = ncr53c710_request_cancelled
 };
 
 /* VMState definition for device state */
 static const VMStateDescription vmstate_ncr53c710_state = {
     .name = "ncr53c710",
     .version_id = 1,
     .minimum_version_id = 1,
     .fields = (VMStateField[]) {
         VMSTATE_INT32(carry, NCR53C710State),
         VMSTATE_INT32(status, NCR53C710State),
         VMSTATE_INT32(msg_action, NCR53C710State),
         VMSTATE_INT32(msg_len, NCR53C710State),
         VMSTATE_BUFFER(msg, NCR53C710State),
         VMSTATE_INT32(waiting, NCR53C710State),
         VMSTATE_INT32(current_lun, NCR53C710State),
         VMSTATE_UINT32(select_tag, NCR53C710State),
         VMSTATE_INT32(command_complete, NCR53C710State),
         
         VMSTATE_UINT32(dsa, NCR53C710State),
         VMSTATE_UINT32(temp, NCR53C710State),
         VMSTATE_UINT32(dnad, NCR53C710State),
         VMSTATE_UINT32(dbc, NCR53C710State),
         VMSTATE_UINT8(istat, NCR53C710State),
         VMSTATE_UINT8(dcmd, NCR53C710State),
         VMSTATE_UINT8(dstat, NCR53C710State),
         VMSTATE_UINT8(dien, NCR53C710State),
         VMSTATE_UINT8(sien0, NCR53C710State),
         VMSTATE_UINT8(ctest2, NCR53C710State),
         VMSTATE_UINT8(ctest3, NCR53C710State),
         VMSTATE_UINT8(ctest4, NCR53C710State),
         VMSTATE_UINT8(ctest5, NCR53C710State),
         VMSTATE_UINT32(dsp, NCR53C710State),
         VMSTATE_UINT32(dsps, NCR53C710State),
         VMSTATE_UINT8(dmode, NCR53C710State),
         VMSTATE_UINT8(dcntl, NCR53C710State),
         VMSTATE_UINT8(scntl0, NCR53C710State),
         VMSTATE_UINT8(scntl1, NCR53C710State),
         VMSTATE_UINT8(sstat0, NCR53C710State),
         VMSTATE_UINT8(sstat1, NCR53C710State),
         VMSTATE_UINT8(sstat2, NCR53C710State),
         VMSTATE_UINT8(scid, NCR53C710State),
         VMSTATE_UINT8(sxfer, NCR53C710State),
         VMSTATE_UINT8(socl, NCR53C710State),
         VMSTATE_UINT8(sdid, NCR53C710State),
         VMSTATE_UINT8(sfbr, NCR53C710State),
         VMSTATE_UINT8(sidl, NCR53C710State),
         VMSTATE_UINT32(sbc, NCR53C710State),
         VMSTATE_UINT32(scratch, NCR53C710State),
         VMSTATE_UINT8(sbr, NCR53C710State),
         VMSTATE_UINT8(ctest0, NCR53C710State),
         VMSTATE_UINT8(ctest1, NCR53C710State),
         VMSTATE_UINT8(ctest6, NCR53C710State),
         VMSTATE_UINT8(ctest7, NCR53C710State),
         VMSTATE_UINT8(ctest8, NCR53C710State),
         VMSTATE_UINT8(lcrc, NCR53C710State),
         VMSTATE_UINT8(dwt, NCR53C710State),
         VMSTATE_UINT8(sbcl, NCR53C710State),
         VMSTATE_UINT8(script_active, NCR53C710State),
         VMSTATE_END_OF_LIST()
     },
 };
 
 static void ncr53c710_class_init(ObjectClass *klass, void *data)
 {
     DeviceClass *dc = DEVICE_CLASS(klass);
     SysBusDeviceClass *sbc = SYS_BUS_DEVICE_CLASS(klass);
 
     sbc->realize = ncr53c710_scsi_realize;
     dc->reset = ncr53c710_reset;
     dc->vmsd = &vmstate_ncr53c710_state;
     set_bit(DEVICE_CATEGORY_STORAGE, dc->categories);
 }
 
 static const TypeInfo ncr53c710_info = {
     .name          = TYPE_NCR53C710,
     .parent        = TYPE_SYS_BUS_DEVICE,
     .instance_size = sizeof(NCR53C710SysBusDeviceState),
     .class_init    = ncr53c710_class_init,
 };
 
 /* Register the device type */
 static void ncr53c710_register_types(void)
 {
     type_register_static(&ncr53c710_info);
 }
 
 type_init(ncr53c710_register_types)
 