/*
 * QEMU NCR53C710 SCSI Host Bus Adapter emulation tests
 *
 * Copyright (c) 2025 Soumyajyotii Sarkar<soumyajyotisarkar23@gmail.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "hw/hw.h"
// #include "hw/scsi/ncr53c710.h"
#include "hw/qdev-properties.h"
#include "qapi/error.h"
#include "qemu/main-loop.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qtest/qtest.h"
#include "libqtest.h"
#include "libqos/libqos.h"
#include "libqos/pci.h"
#include "libqos/malloc.h"

/* Mock SCSIBus for testing */
typedef struct MockSCSIBus {
    SCSIBus bus;
    int command_count;
    int data_transfers;
    uint8_t last_command[16];
    uint8_t status_code;
} MockSCSIBus;

/* Test state */
typedef struct NCR53C710TestState {
    QTestState *qts;
    uint64_t mmio_base;
    int irq_received;
    int irq_count;
} NCR53C710TestState;

/* Setup test environment */
static NCR53C710TestState *ncr53c710_test_init(void)
{
    NCR53C710TestState *s = g_new0(NCR53C710TestState, 1);
    
    /* Start QEMU with the NCR53C710 device */
    s->qts = qtest_init("-device ncr53c710,id=scsi");
    s->mmio_base = 0x1000; /* Use a standard memory base for tests */
    
    return s;
}

/* Cleanup test environment */
static void ncr53c710_test_quit(NCR53C710TestState *s)
{
    qtest_quit(s->qts);
    g_free(s);
}

/* Helper functions for register access */
static uint8_t ncr_readb(NCR53C710TestState *s, hwaddr offset)
{
    return qtest_readb(s->qts, s->mmio_base + offset);
}

static void ncr_writeb(NCR53C710TestState *s, hwaddr offset, uint8_t value)
{
    qtest_writeb(s->qts, s->mmio_base + offset, value);
}

static uint32_t ncr_read_reg32(NCR53C710TestState *s, hwaddr reg_offset)
{
    uint32_t val = 0;
    val |= ncr_readb(s, reg_offset);
    val |= ncr_readb(s, reg_offset + 1) << 8;
    val |= ncr_readb(s, reg_offset + 2) << 16;
    val |= ncr_readb(s, reg_offset + 3) << 24;
    return val;
}

static void ncr_write_reg32(NCR53C710TestState *s, hwaddr reg_offset, uint32_t value)
{
    ncr_writeb(s, reg_offset, value & 0xFF);
    ncr_writeb(s, reg_offset + 1, (value >> 8) & 0xFF);
    ncr_writeb(s, reg_offset + 2, (value >> 16) & 0xFF);
    ncr_writeb(s, reg_offset + 3, (value >> 24) & 0xFF);
}

/* Mock IRQ handler */
static void ncr_irq_handler(void *opaque, int irq, int level)
{
    NCR53C710TestState *s = opaque;
    if (level) {
        s->irq_received = 1;
        s->irq_count++;
    } else {
        s->irq_received = 0;
    }
}

/* Test register read/write */
static void test_registers(void)
{
    NCR53C710TestState *s = ncr53c710_test_init();
    uint8_t test_values[] = {0x55, 0xAA, 0x00, 0xFF, 0x01};
    uint8_t read_value;
    int i;
    
    g_print("Testing register read/write operations\n");
    
    /* Test SCNTL0 register (read/write) */
    for (i = 0; i < sizeof(test_values); i++) {
        ncr_writeb(s, 0x00, test_values[i]);
        read_value = ncr_readb(s, 0x00);
        g_assert_cmpint(read_value, ==, test_values[i]);
    }
    
    /* Test SCNTL1 register (read/write) */
    for (i = 0; i < sizeof(test_values); i++) {
        ncr_writeb(s, 0x01, test_values[i]);
        read_value = ncr_readb(s, 0x01);
        g_assert_cmpint(read_value, ==, test_values[i]);
    }
    
    /* Test DSA register (32-bit register) */
    ncr_write_reg32(s, 0x10, 0x12345678);
    g_assert_cmpint(ncr_read_reg32(s, 0x10), ==, 0x12345678);
    
    /* Test TEMP register (32-bit register) */
    ncr_write_reg32(s, 0x1c, 0xABCDEF01);
    g_assert_cmpint(ncr_read_reg32(s, 0x1c), ==, 0xABCDEF01);
    
    /* Test DSTAT (read-only register - should clear on read) */
    ncr_writeb(s, 0x0c, 0xAA); /* Write should not affect value */
    read_value = ncr_readb(s, 0x0c);
    /* Only NCR53C710_DSTAT_DFE (0x80) should be set by default */
    g_assert_cmpint(read_value, ==, 0x80);
    read_value = ncr_readb(s, 0x0c);
    /* Reading should clear the register, but DFE is always set */
    g_assert_cmpint(read_value, ==, 0x80);
    
    ncr53c710_test_quit(s);
}

/* Test chip reset */
static void test_chip_reset(void)
{
    NCR53C710TestState *s = ncr53c710_test_init();
    
    g_print("Testing chip reset\n");
    
    /* Modify some registers */
    ncr_writeb(s, 0x00, 0x55); /* SCNTL0 */
    ncr_writeb(s, 0x01, 0xAA); /* SCNTL1 */
    ncr_write_reg32(s, 0x10, 0x12345678); /* DSA */
    
    /* Trigger a reset */
    ncr_writeb(s, 0x21, NCR53C710_ISTAT_RST); /* Set the reset bit in ISTAT */
    
    /* Verify registers are reset to default values */
    g_assert_cmpint(ncr_readb(s, 0x00), ==, 0xc0); /* SCNTL0 reset value */
    g_assert_cmpint(ncr_readb(s, 0x01), ==, 0x00); /* SCNTL1 reset value */
    g_assert_cmpint(ncr_readb(s, 0x04), ==, 0x80); /* SCID reset value */
    g_assert_cmpint(ncr_read_reg32(s, 0x10), ==, 0x00); /* DSA reset value */
    
    ncr53c710_test_quit(s);
}

/* Test DMA operations */
static void test_dma_operations(void)
{
    NCR53C710TestState *s = ncr53c710_test_init();
    QOSState *qs;
    QGuestAllocator *alloc;
    uint64_t src_addr, dest_addr;
    uint32_t test_data[4] = {0xDEADBEEF, 0x12345678, 0xABCDEF01, 0x87654321};
    
    g_print("Testing DMA operations\n");
    
    /* Setup memory for DMA operations */
    qs = qtest_spapr_boot();
    alloc = qs->alloc;
    src_addr = guest_alloc(alloc, sizeof(test_data));
    dest_addr = guest_alloc(alloc, sizeof(test_data));
    
    /* Write test data to source buffer */
    qtest_memwrite(s->qts, src_addr, test_data, sizeof(test_data));
    
    /* Clear destination buffer */
    qtest_memset(s->qts, dest_addr, 0, sizeof(test_data));
    
    /* Set up DMA registers for memory-to-memory copy */
    ncr_write_reg32(s, 0x2c, 0x00000000); /* DSP: Point to memory-move script */
    /* SCRIPTS memory-to-memory move (instruction will be loaded via DMA) */
    uint32_t script[3] = {
        0xC0000010,      /* Block move, 16 bytes */
        src_addr,        /* Source address */
        dest_addr        /* Destination address (usually loaded at DSP+8) */
    };
    uint64_t script_addr = guest_alloc(alloc, sizeof(script));
    qtest_memwrite(s->qts, script_addr, script, sizeof(script));
    
    /* Write the script address to DSP and start execution */
    ncr_write_reg32(s, 0x2c, script_addr);
    
    /* Let the virtual machine run a bit to complete the DMA operation */
    qtest_clock_step(s->qts, 1000000);
    
    /* Check that data was copied correctly */
    uint32_t result[4];
    qtest_memread(s->qts, dest_addr, result, sizeof(result));
    
    g_assert_cmpmem(result, sizeof(result), test_data, sizeof(test_data));
    
    /* Free allocated memory */
    guest_free(alloc, src_addr);
    guest_free(alloc, dest_addr);
    guest_free(alloc, script_addr);
    
    qtest_shutdown(qs);
    ncr53c710_test_quit(s);
}

/* Test interrupt generation */
static void test_interrupt_generation(void)
{
    NCR53C710TestState *s = ncr53c710_test_init();
    
    g_print("Testing interrupt generation\n");
    
    /* Register IRQ handler */
    qtest_irq_intercept_in(s->qts, "ncr53c710");
    
    /* Enable interrupts */
    ncr_writeb(s, 0x39, 0xFF); /* DIEN - DMA interrupt enable */
    ncr_writeb(s, 0x03, 0xFF); /* SIEN0 - SCSI interrupt enable 0 */
    
    /* Generate a DMA interrupt */
    ncr_writeb(s, 0x21, NCR53C710_ISTAT_ABRT); /* Set ABRT bit in ISTAT */
    
    /* Verify interrupt was received */
    g_assert_true(s->irq_received);
    g_assert_cmpint(s->irq_count, ==, 1);
    
    /* Check that DSTAT register shows the correct interrupt cause */
    uint8_t dstat = ncr_readb(s, 0x0c);
    g_assert_true(dstat & NCR53C710_DSTAT_ABRT);
    
    /* Clear interrupts by reading the status register (already done above) */
    /* Verify interrupt was cleared */
    g_assert_false(s->irq_received);
    
    ncr53c710_test_quit(s);
}

/* Test SCSI selection and command execution */
static void test_scsi_selection(void)
{
    NCR53C710TestState *s = ncr53c710_test_init();
    QOSState *qs;
    QGuestAllocator *alloc;
    uint64_t cmd_addr, script_addr;
    
    g_print("Testing SCSI selection and command execution\n");
    
    /* Setup memory for SCSI command */
    qs = qtest_spapr_boot();
    alloc = qs->alloc;
    
    /* SCSI INQUIRY command */
    uint8_t inquiry_cmd[6] = {0x12, 0x00, 0x00, 0x00, 0x24, 0x00};
    cmd_addr = guest_alloc(alloc, sizeof(inquiry_cmd));
    qtest_memwrite(s->qts, cmd_addr, inquiry_cmd, sizeof(inquiry_cmd));
    
    /* SCRIPTS for selection and command execution */
    uint32_t script[] = {
        0x47000000 | (1 << 16),   /* Select target 0 with ATN */
        0x00000000,               /* Jump address (not used) */
        0x06000000 | sizeof(inquiry_cmd), /* Move command bytes from memory to SCSI bus */
        cmd_addr,                 /* Command buffer address */
    };
    script_addr = guest_alloc(alloc, sizeof(script));
    qtest_memwrite(s->qts, script_addr, script, sizeof(script));
    
    /* Write the script address to DSP and start execution */
    ncr_write_reg32(s, 0x2c, script_addr);
    
    /* Let the virtual machine run to execute the script */
    qtest_clock_step(s->qts, 1000000);
    
    /* Check that selection was performed - SDID should contain the target ID */
    g_assert_cmpint(ncr_readb(s, 0x02), ==, 0x01); /* SDID = target 0 */
    
    /* Check that we're connected */
    g_assert_cmpint(ncr_readb(s, 0x01) & NCR53C710_SCNTL1_CON, !=, 0);
    
    /* Free allocated memory */
    guest_free(alloc, cmd_addr);
    guest_free(alloc, script_addr);
    
    qtest_shutdown(qs);
    ncr53c710_test_quit(s);
}

/* Run all tests */
int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    
    g_test_add_func("/ncr53c710/registers", test_registers);
    g_test_add_func("/ncr53c710/reset", test_chip_reset);
    g_test_add_func("/ncr53c710/dma", test_dma_operations);
    g_test_add_func("/ncr53c710/interrupts", test_interrupt_generation);
    g_test_add_func("/ncr53c710/scsi_selection", test_scsi_selection);

    return g_test_run();
}
/* Test watchdog timer functionality */
static void test_watchdog_timer(void)
{
    NCR53C710TestState *s = ncr53c710_test_init();
    
    g_print("Testing watchdog timer functionality\n");
    
    /* Enable DMA watchdog interrupt */
    ncr_writeb(s, 0x39, NCR53C710_DSTAT_WTD); /* DIEN - Enable watchdog interrupt */
    
    /* Set watchdog timer to a small value */
    ncr_writeb(s, 0x3a, 1); /* DWT - Set watchdog timer to 1 */
    
    /* Register IRQ handler */
    qtest_irq_intercept_in(s->qts, "ncr53c710");
    
    /* Start a script that would take a long time (this will trigger the watchdog) */
    /* Create an infinite loop script that simply jumps to itself */
    uint64_t script_addr;
    QOSState *qs = qtest_spapr_boot();
    QGuestAllocator *alloc = qs->alloc;
    
    script_addr = guest_alloc(alloc, 16);
    uint32_t script[2] = {
        0x80000000,      /* Jump instruction - opcode 2, subtype 0 */
        script_addr      /* Jump to self (infinite loop) */
    };
    qtest_memwrite(s->qts, script_addr, script, sizeof(script));
    
    /* Write the script address to DSP and start execution */
    ncr_write_reg32(s, 0x2c, script_addr);
    
    /* Let the virtual machine run to trigger the watchdog */
    qtest_clock_step(s->qts, 1000000);
    
    /* Verify that the watchdog timeout occurred */
    g_assert_true(s->irq_received);
    g_assert_cmpint(ncr_readb(s, 0x0c) & NCR53C710_DSTAT_WTD, !=, 0);
    
    /* Cleanup */
    guest_free(alloc, script_addr);
    qtest_shutdown(qs);
    ncr53c710_test_quit(s);
}

/* Test SCSI phase transitions */
static void test_scsi_phase_transitions(void)
{
    NCR53C710TestState *s = ncr53c710_test_init();
    
    g_print("Testing SCSI phase transitions\n");
    
    /* Start with a selection phase */
    ncr_writeb(s, 0x01, NCR53C710_SCNTL1_CON); /* Mark as connected */
    
    /* Test various phase transitions and verify they're reflected in SSTAT2 */
    
    /* Command phase */
    QOSState *qs = qtest_spapr_boot();
    QGuestAllocator *alloc = qs->alloc;
    uint64_t script_addr = guest_alloc(alloc, 32);
    
    /* Script to transition through multiple phases:
     * 1. Select target
     * 2. Message out (identify)
     * 3. Command phase
     * 4. Data in phase
     * 5. Status phase
     * 6. Message in phase
     */
    uint32_t script[] = {
        0x47000001,      /* Select target 0 with ATN */
        0x00000000,      /* Alternate address (not used) */
        0x06000001,      /* Move message out: identify message */
        script_addr + 28, /* Identify message location */
        0x06000006,      /* Move CDB (INQUIRY command) */
        script_addr + 32, /* Command bytes location */
        0x01000100,      /* Move data in */
        0x00000000,      /* Data buffer (not important for test) */
    };
    
    /* Identify message (LUN 0) */
    uint8_t identify_msg = 0x80;
    
    /* Simple INQUIRY command */
    uint8_t inquiry_cmd[6] = {0x12, 0x00, 0x00, 0x00, 0x24, 0x00};
    
    /* Write data to guest memory */
    qtest_memwrite(s->qts, script_addr, script, sizeof(script));
    qtest_memwrite(s->qts, script_addr + 28, &identify_msg, 1);
    qtest_memwrite(s->qts, script_addr + 32, inquiry_cmd, sizeof(inquiry_cmd));
    
    /* Write the script address to DSP */
    ncr_write_reg32(s, 0x2c, script_addr);
    
    /* Let the VM run for a bit */
    qtest_clock_step(s->qts, 1000000);
    
    /* Verify that we went through the expected phases */
    /* Note: In a real test, we'd need a way to track all phases, here
     * we just verify the final state */
    g_assert_cmpint(ncr_readb(s, 0x01) & NCR53C710_SCNTL1_CON, !=, 0);
    
    /* Cleanup */
    guest_free(alloc, script_addr);
    qtest_shutdown(qs);
    ncr53c710_test_quit(s);
}

/* Test SCRIPT error handling */
static void test_script_error_handling(void)
{
    NCR53C710TestState *s = ncr53c710_test_init();
    
    g_print("Testing SCRIPT error handling\n");
    
    /* Enable DMA interrupts for illegal instruction */
    ncr_writeb(s, 0x39, NCR53C710_DSTAT_IID); /* DIEN - Enable illegal instruction interrupt */
    
    /* Register IRQ handler */
    qtest_irq_intercept_in(s->qts, "ncr53c710");
    
    /* Create an invalid SCRIPT instruction */
    QOSState *qs = qtest_spapr_boot();
    QGuestAllocator *alloc = qs->alloc;
    uint64_t script_addr = guest_alloc(alloc, 8);
    
    /* Invalid instruction type (0xF in bits 30-31) */
    uint32_t invalid_script[2] = {
        0xF0000000,      /* Invalid instruction type */
        0x00000000       /* Argument */
    };
    qtest_memwrite(s->qts, script_addr, invalid_script, sizeof(invalid_script));
    
    /* Write the script address to DSP and start execution */
    ncr_write_reg32(s, 0x2c, script_addr);
    
    /* Let the VM run */
    qtest_clock_step(s->qts, 100000);
    
    /* Verify that illegal instruction was detected */
    g_assert_true(s->irq_received);
    g_assert_cmpint(ncr_readb(s, 0x0c) & NCR53C710_DSTAT_IID, !=, 0);
    
    /* Reset the chip */
    ncr_writeb(s, 0x21, NCR53C710_ISTAT_RST);
    
    /* Test invalid transfer control operation */
    uint32_t invalid_tc_script[2] = {
        0x88000000,      /* Transfer control with invalid operation (bits 27-29 = 0x4) */
        0x00000000       /* Target address */
    };
    qtest_memwrite(s->qts, script_addr, invalid_tc_script, sizeof(invalid_tc_script));
    
    /* Clear previous IRQ state */
    s->irq_received = 0;
    s->irq_count = 0;
    
    /* Write the script address to DSP again */
    ncr_write_reg32(s, 0x2c, script_addr);
    
    /* Let the VM run */
    qtest_clock_step(s->qts, 100000);
    
    /* Verify that illegal instruction was detected again */
    g_assert_true(s->irq_received);
    g_assert_cmpint(ncr_readb(s, 0x0c) & NCR53C710_DSTAT_IID, !=, 0);
    
    /* Cleanup */
    guest_free(alloc, script_addr);
    qtest_shutdown(qs);
    ncr53c710_test_quit(s);
}

/* Test message handling */
static void test_message_handling(void)
{
    NCR53C710TestState *s = ncr53c710_test_init();
    QOSState *qs;
    QGuestAllocator *alloc;
    uint64_t script_addr, msg_buf_addr;
    
    g_print("Testing SCSI message handling\n");
    
    /* Setup memory for messages and scripts */
    qs = qtest_spapr_boot();
    alloc = qs->alloc;
    
    /* Test various message types */
    
    /* 1. Simple message out - ABORT */
    script_addr = guest_alloc(alloc, 64);
    msg_buf_addr = guest_alloc(alloc, 16);
    
    /* ABORT message */
    uint8_t abort_msg = 0x06;
    qtest_memwrite(s->qts, msg_buf_addr, &abort_msg, 1);
    
    /* Script to select target and send ABORT message */
    uint32_t abort_script[4] = {
        0x47000001,      /* Select target 0 with ATN */
        0x00000000,      /* Alternative address (not used) */
        0x06000001,      /* Move 1 byte message from memory to SCSI bus in message out phase */
        msg_buf_addr     /* Message buffer address */
    };
    qtest_memwrite(s->qts, script_addr, abort_script, sizeof(abort_script));
    
    /* Write the script address to DSP and start execution */
    ncr_write_reg32(s, 0x2c, script_addr);
    
    /* Let the VM run */
    qtest_clock_step(s->qts, 100000);
    
    /* Verify that the bus disconnected (ABORT should disconnect) */
    g_assert_cmpint(ncr_readb(s, 0x01) & NCR53C710_SCNTL1_CON, ==, 0);
    
    /* 2. Extended message - SDTR (Synchronous Data Transfer Request) */
    uint8_t sdtr_msg[5] = {0x01, 0x03, 0x01, 0x19, 0x0F}; /* Extended message, SDTR, period=25, offset=15 */
    qtest_memwrite(s->qts, msg_buf_addr, sdtr_msg, 5);
    
    /* Script to select target and send SDTR message */
    uint32_t sdtr_script[4] = {
        0x47000001,      /* Select target 0 with ATN */
        0x00000000,      /* Alternative address (not used) */
        0x06000005,      /* Move 5 bytes message from memory to SCSI bus */
        msg_buf_addr     /* Message buffer address */
    };
    qtest_memwrite(s->qts, script_addr, sdtr_script, sizeof(sdtr_script));
    
    /* Reset connection state */
    ncr_writeb(s, 0x01, 0); /* Clear SCNTL1 */
    
    /* Write the script address to DSP and start execution */
    ncr_write_reg32(s, 0x2c, script_addr);
    
    /* Let the VM run */
    qtest_clock_step(s->qts, 100000);
    
    /* Test should proceed without errors */
    
    /* 3. Test message rejection with invalid message */
    uint8_t invalid_msg = 0x0F; /* Invalid message code */
    qtest_memwrite(s->qts, msg_buf_addr, &invalid_msg, 1);
    
    /* Script to select target and send invalid message */
    uint32_t invalid_msg_script[4] = {
        0x47000001,      /* Select target 0 with ATN */
        0x00000000,      /* Alternative address (not used) */
        0x06000001,      /* Move 1 byte message from memory to SCSI bus */
        msg_buf_addr     /* Message buffer address */
    };
    qtest_memwrite(s->qts, script_addr, invalid_msg_script, sizeof(invalid_msg_script));
    
    /* Reset connection state */
    ncr_writeb(s, 0x01, 0); /* Clear SCNTL1 */
    
    /* Write the script address to DSP and start execution */
    ncr_write_reg32(s, 0x2c, script_addr);
    
    /* Let the VM run */
    qtest_clock_step(s->qts, 100000);
    
    /* The device should have moved to MESSAGE IN phase with MESSAGE REJECT */
    g_assert_cmpint(ncr_readb(s, 0x0f) & NCR53C710_PHASE_MASK, ==, NCR53C710_PHASE_MI);
    
    /* Cleanup */
    guest_free(alloc, script_addr);
    guest_free(alloc, msg_buf_addr);
    qtest_shutdown(qs);
    ncr53c710_test_quit(s);
}

/* Test synchronous transfers */
static void test_sync_transfers(void)
{
    NCR53C710TestState *s = ncr53c710_test_init();
    
    g_print("Testing synchronous transfers\n");
    
    /* Set up synchronous transfer parameters in SXFER register */
    /* TP (bits 7-5): Transfer period = 4 (100ns) */
    /* MO (bits 4-0): Max offset = 8 */
    uint8_t sxfer_val = (4 << 5) | 8;
    ncr_writeb(s, 0x05, sxfer_val); /* SXFER register */
    
    /* Verify SXFER was set correctly */
    g_assert_cmpint(ncr_readb(s, 0x05), ==, sxfer_val);
    
    /* Create a test script that performs a data transfer */
    QOSState *qs = qtest_spapr_boot();
    QGuestAllocator *alloc = qs->alloc;
    
    uint64_t script_addr = guest_alloc(alloc, 32);
    uint64_t data_addr = guest_alloc(alloc, 512);
    
    /* Fill test data buffer */
    uint8_t test_data[512];
    for (int i = 0; i < 512; i++) {
        test_data[i] = i & 0xFF;
    }
    qtest_memwrite(s->qts, data_addr, test_data, sizeof(test_data));
    
    /* Selection + data-out transfer script */
    uint32_t script[6] = {
        0x47000001,      /* Select target 0 with ATN */
        0x00000000,      /* Alternative address (not used) */
        /* Message out phase: Identify message */
        0x06000001,      /* Move 1 byte in message out phase */
        data_addr,       /* Address of identify message */
        /* Data out phase: Transfer data */
        0x00000200,      /* Move 512 bytes in data out phase */
        data_addr        /* Data buffer address */
    };
    qtest_memwrite(s->qts, script_addr, script, sizeof(script));
    
    /* Connect and set phase to message out first */
    ncr_writeb(s, 0x01, NCR53C710_SCNTL1_CON); /* Mark as connected */
    ncr_writeb(s, 0x0f, NCR53C710_PHASE_MO);   /* Set phase to message out */
    
    /* Start the script */
    ncr_write_reg32(s, 0x2c, script_addr);
    
    /* Let the VM run */
    qtest_clock_step(s->qts, 100000);
    
    /* Test should execute without errors */
    
    /* Cleanup */
    guest_free(alloc, script_addr);
    guest_free(alloc, data_addr);
    qtest_shutdown(qs);
    ncr53c710_test_quit(s);
}

/* Test IRQ masking functionality */
static void test_irq_masking(void)
{
    NCR53C710TestState *s = ncr53c710_test_init();
    
    g_print("Testing IRQ masking functionality\n");
    
    /* Register IRQ handler */
    qtest_irq_intercept_in(s->qts, "ncr53c710");
    
    /* 1. Test with IRQs enabled */
    ncr_writeb(s, 0x39, 0xFF);     /* DIEN - Enable all DMA interrupts */
    ncr_writeb(s, 0x03, 0xFF);     /* SIEN0 - Enable all SCSI interrupts */
    
    /* Generate DMA abort interrupt */
    ncr_writeb(s, 0x21, NCR53C710_ISTAT_ABRT);
    
    /* Verify interrupt occurred */
    g_assert_true(s->irq_received);
    g_assert_cmpint(s->irq_count, ==, 1);
    g_assert_cmpint(ncr_readb(s, 0x0c) & NCR53C710_DSTAT_ABRT, !=, 0);
    
    /* Clear interrupt by reading DSTAT */
    uint8_t dstat = ncr_readb(s, 0x0c);
    s->irq_received = 0;
    s->irq_count = 0;
    
    /* 2. Test with IRQs disabled via DIEN */
    ncr_writeb(s, 0x39, 0x00);     /* DIEN - Disable all DMA interrupts */
    ncr_writeb(s, 0x03, 0xFF);     /* SIEN0 - Enable all SCSI interrupts */
    
    /* Generate DMA abort interrupt again */
    ncr_writeb(s, 0x21, NCR53C710_ISTAT_ABRT);
    
    /* Verify no interrupt was generated despite ABRT being set */
    g_assert_false(s->irq_received);
    g_assert_cmpint(s->irq_count, ==, 0);
    g_assert_cmpint(ncr_readb(s, 0x0c) & NCR53C710_DSTAT_ABRT, !=, 0);
    
    /* 3. Test with global IRQ disable (IRQD in DCNTL) */
    ncr_writeb(s, 0x39, 0xFF);     /* DIEN - Enable all DMA interrupts */
    ncr_writeb(s, 0x3b, NCR53C710_DCNTL_IRQD);  /* DCNTL - Set IRQ disable bit */
    
    /* Clear previous state */
    dstat = ncr_readb(s, 0x0c);
    s->irq_received = 0;
    
    /* Generate DMA abort interrupt again */
    ncr_writeb(s, 0x21, NCR53C710_ISTAT_ABRT);
    
    /* Verify no interrupt was generated despite ABRT being set and DIEN enabled */
    g_assert_false(s->irq_received);
    g_assert_cmpint(ncr_readb(s, 0x0c) & NCR53C710_DSTAT_ABRT, !=, 0);
    
    ncr53c710_test_quit(s);
}

/* Test bus reset functionality */
static void test_bus_reset(void)
{
    NCR53C710TestState *s = ncr53c710_test_init();
    
    g_print("Testing SCSI bus reset functionality\n");
    
    /* Register IRQ handler */
    qtest_irq_intercept_in(s->qts, "ncr53c710");
    
    /* Enable reset interrupt */
    ncr_writeb(s, 0x03, NCR53C710_SSTAT0_RST); /* Enable reset interrupt */
    
    /* Assert SCSI RST signal */
    ncr_writeb(s, 0x01, NCR53C710_SCNTL1_RST);
    
    /* Verify that RST bit is set in SSTAT0 */
    g_assert_cmpint(ncr_readb(s, 0x0d) & NCR53C710_SSTAT0_RST, !=, 0);
    
    /* Verify interrupt was generated */
    g_assert_true(s->irq_received);
    
    /* Deassert RST signal */
    ncr_writeb(s, 0x01, 0);
    
    /* Clear interrupt by reading SSTAT0 */
    uint8_t sstat0 = ncr_readb(s, 0x0d);
    
    ncr53c710_test_quit(s);
}

/* Test register preservation during SCRIPTS execution */
static void test_register_preservation(void)
{
    NCR53C710TestState *s = ncr53c710_test_init();
    QOSState *qs = qtest_spapr_boot();
    QGuestAllocator *alloc = qs->alloc;
    uint64_t script_addr;
    
    g_print("Testing register preservation during SCRIPTS execution\n");
    
    /* Set initial register values */
    ncr_writeb(s, 0x08, 0xAA);      /* SFBR - can be modified by SCRIPTS */
    ncr_write_reg32(s, 0x34, 0x12345678); /* SCRATCH - should be preserved */
    
    /* Create a script that modifies SFBR but should preserve SCRATCH */
    script_addr = guest_alloc(alloc, 16);
    uint32_t script[2] = {
        0xC8000001,      /* Move from reg SFBR with AND operation */
        0x000000FF       /* AND with 0xFF (no change, just to test) */
    };
    qtest_memwrite(s->qts, script_addr, script, sizeof(script));
    
    /* Execute script */
    ncr_write_reg32(s, 0x2c, script_addr);
    
    /* Let VM run */
    qtest_clock_step(s->qts, 100000);
    
    /* Verify SCRATCH register wasn't affected */
    g_assert_cmpint(ncr_read_reg32(s, 0x34), ==, 0x12345678);
    
    /* Cleanup */
    guest_free(alloc, script_addr);
    qtest_shutdown(qs);
    ncr53c710_test_quit(s);
}

/* Test prefetch buffer functionality */
static void test_prefetch_buffer(void)
{
    NCR53C710TestState *s = ncr53c710_test_init();
    
    g_print("Testing prefetch buffer functionality\n");
    
    /* Enable prefetch in DCNTL */
    ncr_writeb(s, 0x3b, NCR53C710_DCNTL_PFEN);
    
    /* Create a script with several instructions */
    QOSState *qs = qtest_spapr_boot();
    QGuestAllocator *alloc = qs->alloc;
    uint64_t script_addr = guest_alloc(alloc, 32);
    
    /* Sequential instructions that should benefit from prefetch */
    uint32_t script[8] = {
        0x98000000,      /* NOP (Transfer Control with false condition) */
        0x00000000,      /* Address (unused) */
        0x98000000,      /* NOP */
        0x00000000,      /* Address (unused) */
        0x98000000,      /* NOP */
        0x00000000,      /* Address (unused) */
        0x98000000,      /* NOP */
        0x00000000       /* Address (unused) */
    };
    qtest_memwrite(s->qts, script_addr, script, sizeof(script));
    
    /* Execute script */
    ncr_write_reg32(s, 0x2c, script_addr);
    
    /* Let VM run */
    qtest_clock_step(s->qts, 100000);
    
    /* Now test prefetch flush */
    ncr_writeb(s, 0x3b, NCR53C710_DCNTL_PFEN | NCR53C710_DCNTL_PFF);
    
    /* Execute script again */
    ncr_write_reg32(s, 0x2c, script_addr);
    
    /* Let VM run */
    qtest_clock_step(s->qts, 100000);
    
    /* Cleanup */
    guest_free(alloc, script_addr);
    qtest_shutdown(qs);
    ncr53c710_test_quit(s);
}

/* Test single-step mode */
static void test_single_step_mode(void)
{
    NCR53C710TestState *s = ncr53c710_test_init();
    
    g_print("Testing single-step mode\n");
    
    /* Register IRQ handler */
    qtest_irq_intercept_in(s->qts, "ncr53c710");
    
    /* Enable single-step interrupt */
    ncr_writeb(s, 0x39, NCR53C710_DSTAT_SSI);
    
    /* Enable single-step mode */
    ncr_writeb(s, 0x3b, NCR53C710_DCNTL_SSM);
    
    /* Create a simple script with multiple instructions */
    QOSState *qs = qtest_spapr_boot();
    QGuestAllocator *alloc = qs->alloc;
    uint64_t script_addr = guest_alloc(alloc, 16);
    
    uint32_t script[4] = {
        0x98000000,      /* NOP (Transfer Control with false condition) */
        0x00000000,      /* Address (unused) */
        0x98000000,      /* Second NOP */
        0x00000000       /* Address (unused) */
    };
    qtest_memwrite(s->qts, script_addr, script, sizeof(script));
    
    /* Reset IRQ count */
    s->irq_count = 0;
    
    /* Execute script */
    ncr_write_reg32(s, 0x2c, script_addr);
    
    /* Let VM run */
    qtest_clock_step(s->qts, 100000);
    
    /* Verify we got an interrupt after the first instruction */
    g_assert_cmpint(s->irq_count, ==, 1);
    g_assert_true(ncr_readb(s, 0x0c) & NCR53C710_DSTAT_SSI);
    
    /* Clear interrupt */
    uint8_t dstat = ncr_readb(s, 0x0c);
    
    /* Continue execution by writing DCNTL_STD */
    ncr_writeb(s, 0x3b, NCR53C710_DCNTL_SSM | NCR53C710_DCNTL_STD);
    
    /* Let VM run */
    qtest_clock_step(s->qts, 100000);
    
    /* Verify we got a second interrupt */
    g_assert_cmpint(s->irq_count, ==, 2);
    
    /* Cleanup */
    guest_free(alloc, script_addr);
    qtest_shutdown(qs);
    ncr53c710_test_quit(s);
}

/* Test selection timeout */
static void test_selection_timeout(void)
{
    NCR53C710TestState *s = ncr53c710_test_init();
    
    g_print("Testing selection timeout\n");
    
    /* Register IRQ handler */
    qtest_irq_intercept_in(s->qts, "ncr53c710");
    
    /* Enable selection timeout interrupt */
    ncr_writeb(s, 0x03, NCR53C710_SSTAT0_STO);
    
    /* Create script to select non-existent target */
    QOSState *qs = qtest_spapr_boot();
    QGuestAllocator *alloc = qs->alloc;
    uint64_t script_addr = guest_alloc(alloc, 8);
    
    uint32_t script[2] = {
        0x47000080,      /* Select target 7 (unlikely to exist in test) */
        0x00000000       /* Alternative address (not used) */
    };
    qtest_memwrite(s->qts, script_addr, script, sizeof(script));
    
    /* Execute script */
    ncr_write_reg32(s, 0x2c, script_addr);
    
    /* Let VM run */
    qtest_clock_step(s->qts, 100000);
    
    /* Verify selection timeout occurred */
    g_assert_true(s->irq_received);
    g_assert_cmpint(ncr_readb(s, 0x0d) & NCR53C710_SSTAT0_STO, !=, 0);
    
    /* Cleanup */
    guest_free(alloc, script_addr);
    qtest_shutdown(qs);
    ncr53c710_test_quit(s);
}

/* Test SCSI FIFO status */
static void test_scsi_fifo_status(void)
{
    NCR53C710TestState *s = ncr53c710_test_init();
    
    g_print("Testing SCSI FIFO status\n");
    
    /* Set up a data transfer to check FIFO status */
    QOSState *qs = qtest_spapr_boot();
    QGuestAllocator *alloc = qs->alloc;
    uint64_t data_addr = guest_alloc(alloc, 64);
    uint64_t script_addr = guest_alloc(alloc, 16);
    
    /* Create test data */
    uint8_t test_data[64];
    memset(test_data, 0xAA, sizeof(test_data));
    qtest_memwrite(s->qts, data_addr, test_data, sizeof(test_data));
    
    /* Create script to perform data transfer */
    uint32_t script[4] = {
        0x47000001,      /* Select target 0 with ATN */
        0x00000000,      /* Alternative address (not used) */
        0x00000040,      /* Data out phase transfer, 64 bytes */
        data_addr        /* Data buffer address */
    };
    qtest_memwrite(s->qts, script_addr, script, sizeof(script));
    
    /* Set connected and data phase */
    ncr_writeb(s, 0x01, NCR53C710_SCNTL1_CON);
    ncr_writeb(s, 0x0f, NCR53C710_PHASE_DO);
    
    /* Start script */
    ncr_write_reg32(s, 0x2c, script_addr);
    
    /* Let VM run */
    qtest_clock_step(s->qts, 1000);
    
    /* Check SSTAT2 for FIFO count (should be non-zero during transfer) */
    uint8_t sstat2 = ncr_readb(s, 0x0f);
    g_print("SSTAT2 FIFO count: 0x%x\n", (sstat2 >> 4) & 0xF);
    
    /* Cleanup */
    guest_free(alloc, script_addr);
    guest_free(alloc, data_addr);
    qtest_shutdown(qs);
    ncr53c710_test_quit(s);
}


int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    
    /* Existing tests */
    g_test_add_func("/ncr53c710/registers", test_registers);
    g_test_add_func("/ncr53c710/reset", test_chip_reset);
    g_test_add_func("/ncr53c710/dma", test_dma_operations);
    g_test_add_func("/ncr53c710/interrupts", test_interrupt_generation);
    g_test_add_func("/ncr53c710/scsi_selection", test_scsi_selection);
    g_test_add_func("/ncr53c710/watchdog", test_watchdog_timer);
    g_test_add_func("/ncr53c710/phase_transitions", test_scsi_phase_transitions);
    g_test_add_func("/ncr53c710/script_errors", test_script_error_handling);
    g_test_add_func("/ncr53c710/message_handling", test_message_handling);
    g_test_add_func("/ncr53c710/sync_transfers", test_sync_transfers);
    g_test_add_func("/ncr53c710/irq_masking", test_irq_masking);
    g_test_add_func("/ncr53c710/bus_reset", test_bus_reset);
    g_test_add_func("/ncr53c710/register_preservation", test_register_preservation);
    g_test_add_func("/ncr53c710/prefetch_buffer", test_prefetch_buffer);
    g_test_add_func("/ncr53c710/single_step", test_single_step_mode);
    g_test_add_func("/ncr53c710/selection_timeout", test_selection_timeout);
    g_test_add_func("/ncr53c710/scsi_fifo", test_scsi_fifo_status);

    return g_test_run();
}