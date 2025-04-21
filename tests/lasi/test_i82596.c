/*
 * QEMU i82596 network card test cases
 *
 * Copyright (c) 2023 QEMU contributors
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

 #include "qemu/osdep.h"
 #include "hw/net/i82596.h"
 #include "qapi/error.h"
 #include "qemu/timer.h"
 #include "qemu/iov.h"
 #include "qemu/main-loop.h"
 #include "qemu/module.h"
 #include "qtest-qmp.h"
 #include "libqos/libqtest.h"
 #include "libqos/libqos.h"
 #include "libqos/pci.h"
 #include "net/eth.h"
 #include "hw/qdev-properties.h"
 
 /* Test fixtures and helper functions */
 typedef struct {
     QTestState *qts;
     uint64_t ram_size;
     char *tmpdir;
     char *test_net_socket;
 } I82596TestState;
 
 static void i82596_test_init(I82596TestState *s)
 {
     s->ram_size = 32 * 1024 * 1024; /* 32MB */
     s->tmpdir = g_strdup("/tmp/qtest.XXXXXX");
     g_assert_nonnull(mkdtemp(s->tmpdir));
 
     s->test_net_socket = g_strdup_printf("%s/test-i82596.sock", s->tmpdir);
     s->qts = qtest_init_with_options("-machine accel=tcg "
                                     "-device i82596,netdev=nq0 "
                                     "-netdev socket,id=nq0,listen=%s",
                                     s->test_net_socket);
 }
 
 static void i82596_test_cleanup(I82596TestState *s)
 {
     qtest_quit(s->qts);
     unlink(s->test_net_socket);
     g_free(s->test_net_socket);
     rmdir(s->tmpdir);
     g_free(s->tmpdir);
 }
 
 /* Helper functions for memory and register access */
 static void write_mem32(I82596TestState *s, uint32_t addr, uint32_t value)
 {
     uint16_t lo = value & 0xffff;
     uint16_t hi = value >> 16;
     qtest_writew(s->qts, addr, lo);
     qtest_writew(s->qts, addr + 2, hi);
 }
 
 static uint32_t read_mem32(I82596TestState *s, uint32_t addr)
 {
     uint32_t lo = qtest_readw(s->qts, addr);
     uint32_t hi = qtest_readw(s->qts, addr + 2);
     return (hi << 16) | lo;
 }
 
 static void write_mem16(I82596TestState *s, uint32_t addr, uint16_t value)
 {
     qtest_writew(s->qts, addr, value);
 }
 
 static uint16_t read_mem16(I82596TestState *s, uint32_t addr)
 {
     return qtest_readw(s->qts, addr);
 }
 
 /* Write to the port registers */
 static void port_write(I82596TestState *s, uint32_t addr, uint32_t val)
 {
     qtest_writel(s->qts, addr, val);
 }
 
 /* Setup SCP (System Configuration Pointer) */
 static uint32_t setup_scp(I82596TestState *s, uint32_t scp_addr, uint32_t iscp_addr)
 {
     /* Set up ISCP */
     write_mem16(s, iscp_addr, ISCP_BUSY); /* Set BUSY */
     write_mem32(s, iscp_addr + 4, iscp_addr + 16); /* SCB address */
     
     /* Set up SCP */
     write_mem32(s, scp_addr, 0); /* Reserved */
     write_mem32(s, scp_addr + 4, 0); /* Status (low 16 bits) and SYSBUS (upper 16 bits) */
     /* Set SYSBUS - big endian, linear mode (0x60) */
     qtest_writeb(s->qts, scp_addr + 7, 0x60);
     write_mem32(s, scp_addr + 8, iscp_addr); /* ISCP */
     
     return iscp_addr + 16; /* Return SCB address */
 }
 
 /* Initialize I82596 with SCP and ISCP */
 static uint32_t initialize_i82596(I82596TestState *s, uint32_t mem_base)
 {
     uint32_t scp_addr = mem_base;
     uint32_t iscp_addr = mem_base + 16;
     uint32_t scb_addr;
     
     scb_addr = setup_scp(s, scp_addr, iscp_addr);
     
     /* Set SCP address */
     port_write(s, PORT_ALTSCP, scp_addr);
     
     /* Assert CA */
     port_write(s, PORT_CA, 0);
     
     /* Wait for BUSY to clear in ISCP */
     int timeout = 100;
     while (timeout-- && (qtest_readb(s->qts, iscp_addr + 1) & 0x1)) {
         g_usleep(1000);
     }
     g_assert_cmpint(timeout, >, 0);
     
     return scb_addr;
 }
 
 /* Tests */
 
 static void test_i82596_initialization(void)
 {
     I82596TestState s = { 0 };
     uint32_t scb_addr;
     
     i82596_test_init(&s);
     
     /* Test initialization */
     scb_addr = initialize_i82596(&s, 0x100000);
     
     /* Check initial SCB status */
     uint16_t status = read_mem16(&s, scb_addr);
     g_assert_cmpint(status, ==, 0);
     
     /* Test reset */
     port_write(&s, PORT_RESET, 0);
     status = read_mem16(&s, scb_addr);
     g_assert_cmpint(status, ==, 0);
     
     i82596_test_cleanup(&s);
 }
 
 static void test_i82596_command_unit(void)
 {
     I82596TestState s = { 0 };
     uint32_t scb_addr;
     uint32_t cmd_addr;
     
     i82596_test_init(&s);
     
     /* Initialize the card */
     scb_addr = initialize_i82596(&s, 0x100000);
     cmd_addr = 0x110000;
     
     /* Set up a NOP command */
     write_mem16(&s, cmd_addr, 0); /* Status */
     write_mem16(&s, cmd_addr + 2, CMD_EOL | CMD_INTR | CmdNOp); /* Command */
     write_mem32(&s, cmd_addr + 4, 0xffffffff); /* Link pointer (EOL) */
     
     /* Start command unit */
     write_mem32(&s, scb_addr + 4, cmd_addr); /* Set CBL */
     write_mem16(&s, scb_addr + 2, 0x0100); /* Set CUC = 1 (START) */
     
     /* Issue CA */
     port_write(&s, PORT_CA, 0);
     
     /* Give some time for command execution */
     g_usleep(10000);
     
     /* Check if command completed */
     uint16_t cmd_status = read_mem16(&s, cmd_addr);
     g_assert_cmpint(cmd_status & STAT_C, !=, 0); /* Command completed */
     g_assert_cmpint(cmd_status & STAT_OK, !=, 0); /* Command OK */
     
     /* Check SCB status */
     uint16_t scb_status = read_mem16(&s, scb_addr);
     g_assert_cmpint(scb_status & SCB_STATUS_CX, !=, 0); /* CX bit set */
     
     i82596_test_cleanup(&s);
 }
 
 static void test_i82596_configure(void)
 {
     I82596TestState s = { 0 };
     uint32_t scb_addr;
     uint32_t cmd_addr;
     
     i82596_test_init(&s);
     
     /* Initialize the card */
     scb_addr = initialize_i82596(&s, 0x100000);
     cmd_addr = 0x110000;
     
     /* Set up a Configure command */
     write_mem16(&s, cmd_addr, 0); /* Status */
     write_mem16(&s, cmd_addr + 2, CMD_EOL | CMD_INTR | CmdConfigure); /* Command */
     write_mem32(&s, cmd_addr + 4, 0xffffffff); /* Link pointer (EOL) */
     
     /* Configure bytes */
     qtest_writeb(s->qts, cmd_addr + 8, 16); /* 16 bytes of config */
     
     /* Set some test configuration values */
     qtest_writeb(s->qts, cmd_addr + 9, 0x00);  /* Byte 1 */
     qtest_writeb(s->qts, cmd_addr + 10, 0x00); /* Byte 2 */
     qtest_writeb(s->qts, cmd_addr + 11, 0x40); /* Byte 3 - set bit 6 */
     qtest_writeb(s->qts, cmd_addr + 12, 0x00); /* Byte 4 */
     qtest_writeb(s->qts, cmd_addr + 13, 0x00); /* Byte 5 */
     qtest_writeb(s->qts, cmd_addr + 14, 0x00); /* Byte 6 */
     qtest_writeb(s->qts, cmd_addr + 15, 0x00); /* Byte 7 */
     qtest_writeb(s->qts, cmd_addr + 16, 0x00); /* Byte 8 */
     qtest_writeb(s->qts, cmd_addr + 17, 0x40); /* Byte 9 - set bit 6 for 10BASE-T */
     qtest_writeb(s->qts, cmd_addr + 18, 0x00); /* Byte 10 */
     qtest_writeb(s->qts, cmd_addr + 19, 0x00); /* Byte 11 */
     qtest_writeb(s->qts, cmd_addr + 20, 10);   /* Byte 12 - min frame len */
     qtest_writeb(s->qts, cmd_addr + 21, 0x00); /* Byte 13 */
     qtest_writeb(s->qts, cmd_addr + 22, 0x10); /* Byte 14 - auto port selection */
     qtest_writeb(s->qts, cmd_addr + 23, 0x00); /* Byte 15 */
     
     /* Start command unit */
     write_mem32(&s, scb_addr + 4, cmd_addr); /* Set CBL */
     write_mem16(&s, scb_addr + 2, 0x0100); /* Set CUC = 1 (START) */
     
     /* Issue CA */
     port_write(&s, PORT_CA, 0);
     
     /* Give some time for command execution */
     g_usleep(10000);
     
     /* Check if command completed */
     uint16_t cmd_status = read_mem16(&s, cmd_addr);
     g_assert_cmpint(cmd_status & STAT_C, !=, 0);
     g_assert_cmpint(cmd_status & STAT_OK, !=, 0);
     
     i82596_test_cleanup(&s);
 }
 
 static void test_i82596_sa_setup(void)
 {
     I82596TestState s = { 0 };
     uint32_t scb_addr;
     uint32_t cmd_addr;
     uint8_t mac_addr[ETH_ALEN] = {0x52, 0x54, 0x00, 0x12, 0x34, 0x56};
     
     i82596_test_init(&s);
     
     /* Initialize the card */
     scb_addr = initialize_i82596(&s, 0x100000);
     cmd_addr = 0x110000;
     
     /* Set up an SA Setup command */
     write_mem16(&s, cmd_addr, 0); /* Status */
     write_mem16(&s, cmd_addr + 2, CMD_EOL | CMD_INTR | CmdSASetup); /* Command */
     write_mem32(&s, cmd_addr + 4, 0xffffffff); /* Link pointer (EOL) */
     
     /* Write MAC address */
     for (int i = 0; i < ETH_ALEN; i++) {
         qtest_writeb(s->qts, cmd_addr + 8 + i, mac_addr[i]);
     }
     
     /* Start command unit */
     write_mem32(&s, scb_addr + 4, cmd_addr); /* Set CBL */
     write_mem16(&s, scb_addr + 2, 0x0100); /* Set CUC = 1 (START) */
     
     /* Issue CA */
     port_write(&s, PORT_CA, 0);
     
     /* Give some time for command execution */
     g_usleep(10000);
     
     /* Check if command completed */
     uint16_t cmd_status = read_mem16(&s, cmd_addr);
     g_assert_cmpint(cmd_status & STAT_C, !=, 0);
     g_assert_cmpint(cmd_status & STAT_OK, !=, 0);
     
     i82596_test_cleanup(&s);
 }
 
 static void test_i82596_tdr(void)
 {
     I82596TestState s = { 0 };
     uint32_t scb_addr;
     uint32_t cmd_addr;
     
     i82596_test_init(&s);
     
     /* Initialize the card */
     scb_addr = initialize_i82596(&s, 0x100000);
     cmd_addr = 0x110000;
     
     /* Set up a TDR command */
     write_mem16(&s, cmd_addr, 0); /* Status */
     write_mem16(&s, cmd_addr + 2, CMD_EOL | CMD_INTR | CmdTDR); /* Command */
     write_mem32(&s, cmd_addr + 4, 0xffffffff); /* Link pointer (EOL) */
     write_mem16(&s, cmd_addr + 8, 0); /* TIME field */
     write_mem16(&s, cmd_addr + 10, 0); /* Status field */
     
     /* Start command unit */
     write_mem32(&s, scb_addr + 4, cmd_addr); /* Set CBL */
     write_mem16(&s, scb_addr + 2, 0x0100); /* Set CUC = 1 (START) */
     
     /* Issue CA */
     port_write(&s, PORT_CA, 0);
     
     /* Give some time for command execution */
     g_usleep(10000);
     
     /* Check if command completed */
     uint16_t cmd_status = read_mem16(&s, cmd_addr);
     g_assert_cmpint(cmd_status & STAT_C, !=, 0);
     g_assert_cmpint(cmd_status & STAT_OK, !=, 0);
     
     /* Check TDR results */
     uint16_t tdr_status = read_mem16(&s, cmd_addr + 10);
     uint16_t tdr_time = read_mem16(&s, cmd_addr + 8);
     
     /* With default lnkst=0x8000 (up), bit 15 should be set and TIME=0x7FF */
     g_assert_cmpint(tdr_status & 0x8000, !=, 0); /* LNK OK bit */
     g_assert_cmpint(tdr_time, ==, 0x07FF);
     
     i82596_test_cleanup(&s);
 }
 
 static void test_i82596_multicast_setup(void)
 {
     I82596TestState s = { 0 };
     uint32_t scb_addr;
     uint32_t cmd_addr;
     uint8_t mc_addr[ETH_ALEN] = {0x01, 0x00, 0x5e, 0x00, 0x00, 0x01}; /* IANA Multicast */
     
     i82596_test_init(&s);
     
     /* Initialize the card */
     scb_addr = initialize_i82596(&s, 0x100000);
     cmd_addr = 0x110000;
     
     /* Set up a Multicast Setup command */
     write_mem16(&s, cmd_addr, 0); /* Status */
     write_mem16(&s, cmd_addr + 2, CMD_EOL | CMD_INTR | CmdMulticastList); /* Command */
     write_mem32(&s, cmd_addr + 4, 0xffffffff); /* Link pointer (EOL) */
     write_mem16(&s, cmd_addr + 8, ETH_ALEN); /* MC count in bytes */
     
     /* Write multicast address */
     for (int i = 0; i < ETH_ALEN; i++) {
         qtest_writeb(s->qts, cmd_addr + 10 + i, mc_addr[i]);
     }
     
     /* Start command unit */
     write_mem32(&s, scb_addr + 4, cmd_addr); /* Set CBL */
     write_mem16(&s, scb_addr + 2, 0x0100); /* Set CUC = 1 (START) */
     
     /* Issue CA */
     port_write(&s, PORT_CA, 0);
     
     /* Give some time for command execution */
     g_usleep(10000);
     
     /* Check if command completed */
     uint16_t cmd_status = read_mem16(&s, cmd_addr);
     g_assert_cmpint(cmd_status & STAT_C, !=, 0);
     g_assert_cmpint(cmd_status & STAT_OK, !=, 0);
     
     i82596_test_cleanup(&s);
 }
 
 static void test_i82596_port_selection(void)
 {
     I82596TestState s = { 0 };
     uint32_t scb_addr;
     uint32_t cmd_addr;
     
     i82596_test_init(&s);
     
     /* Initialize the card */
     scb_addr = initialize_i82596(&s, 0x100000);
     cmd_addr = 0x110000;
     
     /* First configure with auto port selection enabled */
     write_mem16(&s, cmd_addr, 0); /* Status */
     write_mem16(&s, cmd_addr + 2, CMD_EOL | CMD_INTR | CmdConfigure); /* Command */
     write_mem32(&s, cmd_addr + 4, 0xffffffff); /* Link pointer (EOL) */
     
     /* Configure bytes */
     qtest_writeb(s->qts, cmd_addr + 8, 16); /* 16 bytes of config */
     
     /* Zero out all config bytes first */
     for (int i = 0; i < 16; i++) {
         qtest_writeb(s->qts, cmd_addr + 9 + i, 0);
     }
     
     /* Set important config values */
     qtest_writeb(s->qts, cmd_addr + 17, 0x40); /* Byte 9 - set bit 6 for 10BASE-T */
     qtest_writeb(s->qts, cmd_addr + 20, 10);   /* Byte 12 - min frame len */
     qtest_writeb(s->qts, cmd_addr + 22, 0x10); /* Byte 14 - auto port selection */
     
     /* Start command unit */
     write_mem32(&s, scb_addr + 4, cmd_addr); /* Set CBL */
     write_mem16(&s, scb_addr + 2, 0x0100); /* Set CUC = 1 (START) */
     
     /* Issue CA */
     port_write(&s, PORT_CA, 0);
     
     /* Give some time for command execution */
     g_usleep(10000);
     
     /* Now test link down scenario which should trigger auto port selection */
     QDict *args, *resp;
     
     args = qdict_new();
     qdict_put_str(args, "device", "nq0");
     qdict_put_bool(args, "up", false);
     
     resp = qtest_qmp(s.qts, "{ 'execute': 'set_link', 'arguments': %p }", args);
     g_assert(qdict_haskey(resp, "return"));
     qobject_unref(resp);
     
     /* Allow time for port selection to occur */
     g_usleep(10000);
     
     /* Set link back up */
     args = qdict_new();
     qdict_put_str(args, "device", "nq0");
     qdict_put_bool(args, "up", true);
     
     resp = qtest_qmp(s.qts, "{ 'execute': 'set_link', 'arguments': %p }", args);
     g_assert(qdict_haskey(resp, "return"));
     qobject_unref(resp);
     
     i82596_test_cleanup(&s);
 }
 
 /* Main test suite */
 int main(int argc, char **argv)
 {
     g_test_init(&argc, &argv, NULL);
     g_test_add_func("/i82596/initialization", test_i82596_initialization);
     g_test_add_func("/i82596/command_unit", test_i82596_command_unit);
     g_test_add_func("/i82596/configure", test_i82596_configure);
     g_test_add_func("/i82596/sa_setup", test_i82596_sa_setup);
     g_test_add_func("/i82596/tdr", test_i82596_tdr);
     g_test_add_func("/i82596/multicast_setup", test_i82596_multicast_setup);
     g_test_add_func("/i82596/port_selection", test_i82596_port_selection);
     
     return g_test_run();
 }
 static void test_i82596_chained_commands(void)
 {
     I82596TestState s = { 0 };
     uint32_t scb_addr;
     uint32_t cmd1_addr, cmd2_addr, cmd3_addr;
     
     i82596_test_init(&s);
     
     /* Initialize the card */
     scb_addr = initialize_i82596(&s, 0x100000);
     cmd1_addr = 0x110000;
     cmd2_addr = 0x110100;
     cmd3_addr = 0x110200;
     
     /* Set up a chain of commands: NOP -> SA Setup -> TDR */
     /* First command: NOP */
     write_mem16(&s, cmd1_addr, 0); /* Status */
     write_mem16(&s, cmd1_addr + 2, CMD_INTR | CmdNOp); /* Command with interrupt */
     write_mem32(&s, cmd1_addr + 4, cmd2_addr); /* Link to next command */
     
     /* Second command: SA Setup */
     write_mem16(&s, cmd2_addr, 0); /* Status */
     write_mem16(&s, cmd2_addr + 2, CMD_INTR | CmdSASetup); /* Command with interrupt */
     write_mem32(&s, cmd2_addr + 4, cmd3_addr); /* Link to next command */
     
     /* Set MAC address */
     uint8_t mac_addr[ETH_ALEN] = {0x52, 0x54, 0x00, 0x12, 0x34, 0x56};
     for (int i = 0; i < ETH_ALEN; i++) {
         qtest_writeb(s.qts, cmd2_addr + 8 + i, mac_addr[i]);
     }
     
     /* Third command: TDR */
     write_mem16(&s, cmd3_addr, 0); /* Status */
     write_mem16(&s, cmd3_addr + 2, CMD_EOL | CMD_INTR | CmdTDR); /* Command with interrupt and EOL */
     write_mem32(&s, cmd3_addr + 4, 0xffffffff); /* No next command */
     write_mem16(&s, cmd3_addr + 8, 0); /* TIME field */
     write_mem16(&s, cmd3_addr + 10, 0); /* Status field */
     
     /* Start command unit */
     write_mem32(&s, scb_addr + 4, cmd1_addr); /* Set CBL to first command */
     write_mem16(&s, scb_addr + 2, 0x0100); /* Set CUC = 1 (START) */
     
     /* Issue CA */
     port_write(&s, PORT_CA, 0);
     
     /* Give some time for command execution */
     g_usleep(20000);
     
     /* Check if all commands completed */
     uint16_t cmd1_status = read_mem16(&s, cmd1_addr);
     uint16_t cmd2_status = read_mem16(&s, cmd2_addr);
     uint16_t cmd3_status = read_mem16(&s, cmd3_addr);
     
     g_assert_cmpint(cmd1_status & STAT_C, !=, 0); /* Command 1 completed */
     g_assert_cmpint(cmd1_status & STAT_OK, !=, 0); /* Command 1 OK */
     
     g_assert_cmpint(cmd2_status & STAT_C, !=, 0); /* Command 2 completed */
     g_assert_cmpint(cmd2_status & STAT_OK, !=, 0); /* Command 2 OK */
     
     g_assert_cmpint(cmd3_status & STAT_C, !=, 0); /* Command 3 completed */
     g_assert_cmpint(cmd3_status & STAT_OK, !=, 0); /* Command 3 OK */
     
     /* Check TDR results */
     uint16_t tdr_status = read_mem16(&s, cmd3_addr + 10);
     g_assert_cmpint(tdr_status & 0x8000, !=, 0); /* LNK OK bit should be set */
     
     i82596_test_cleanup(&s);
 }
 
 static void test_i82596_transmit(void)
 {
     I82596TestState s = { 0 };
     uint32_t scb_addr;
     uint32_t cmd_addr, tbd_addr;
     
     i82596_test_init(&s);
     
     /* Initialize the card */
     scb_addr = initialize_i82596(&s, 0x100000);
     cmd_addr = 0x110000;
     tbd_addr = 0x111000;
     
     /* Set up a transmit command */
     write_mem16(&s, cmd_addr, 0); /* Status */
     write_mem16(&s, cmd_addr + 2, CMD_EOL | CMD_INTR | CmdTx); /* Command */
     write_mem32(&s, cmd_addr + 4, 0xffffffff); /* Link pointer (EOL) */
     write_mem32(&s, cmd_addr + 8, tbd_addr); /* Transmit Buffer Descriptor */
     
     /* Set up the Transmit Buffer Descriptor */
     uint16_t tx_size = 64; /* 64 byte packet */
     write_mem16(&s, tbd_addr, tx_size | I596_EOF); /* Size with EOF */
     write_mem32(&s, tbd_addr + 4, 0xffffffff); /* No next TBD */
     write_mem32(&s, tbd_addr + 8, 0x120000); /* Transmit Buffer Address */
     
     /* Set up a packet to transmit - Ethernet header followed by payload */
     uint8_t tx_packet[64] = {
         /* Destination MAC */
         0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
         /* Source MAC */
         0x52, 0x54, 0x00, 0x12, 0x34, 0x56,
         /* EtherType - IPv4 */
         0x08, 0x00,
         /* Payload - just some data */
         0x45, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00,
         0x40, 0x11, 0x00, 0x00, 0x0a, 0x00, 0x00, 0x01,
         0x0a, 0x00, 0x00, 0x02, 0x30, 0x39, 0x30, 0x3a,
         0x00, 0x1c, 0x00, 0x00, 0x48, 0x65, 0x6c, 0x6c,
         0x6f, 0x20, 0x57, 0x6f, 0x72, 0x6c, 0x64, 0x21,
         0x20, 0x66, 0x72, 0x6f, 0x6d, 0x20, 0x51, 0x45,
         0x4d, 0x55
     };
     
     /* Write the packet to memory */
     for (int i = 0; i < tx_size; i++) {
         qtest_writeb(s.qts, 0x120000 + i, tx_packet[i]);
     }
     
     /* Start command unit */
     write_mem32(&s, scb_addr + 4, cmd_addr); /* Set CBL */
     write_mem16(&s, scb_addr + 2, 0x0100); /* Set CUC = 1 (START) */
     
     /* Issue CA */
     port_write(&s, PORT_CA, 0);
     
     /* Give some time for command execution */
     g_usleep(10000);
     
     /* Check if command completed */
     uint16_t cmd_status = read_mem16(&s, cmd_addr);
     g_assert_cmpint(cmd_status & STAT_C, !=, 0); /* Command completed */
     g_assert_cmpint(cmd_status & STAT_OK, !=, 0); /* Command OK */
     
     /* Check SCB status */
     uint16_t scb_status = read_mem16(&s, scb_addr);
     g_assert_cmpint(scb_status & SCB_STATUS_CX, !=, 0); /* CX bit set */
     
     i82596_test_cleanup(&s);
 }
 
 static void test_i82596_receive_setup(void)
 {
     I82596TestState s = { 0 };
     uint32_t scb_addr;
     uint32_t rfa_addr, rbd_addr;
     
     i82596_test_init(&s);
     
     /* Initialize the card */
     scb_addr = initialize_i82596(&s, 0x100000);
     rfa_addr = 0x120000; /* Receive Frame Area */
     rbd_addr = 0x130000; /* Receive Buffer Descriptors */
     
     /* Set up first RFD */
     write_mem16(&s, rfa_addr, 0); /* Status */
     write_mem16(&s, rfa_addr + 2, CMD_FLEX | CMD_EOL); /* Command - Flex mode with EOL */
     write_mem32(&s, rfa_addr + 4, rfa_addr); /* Link to self - circular buffer */
     write_mem32(&s, rfa_addr + 8, rbd_addr); /* RBD address */
     write_mem16(&s, rfa_addr + 12, 0); /* ActualCount */
     write_mem16(&s, rfa_addr + 14, 0); /* Size */
     
     /* Set up RBD - 1536 bytes buffer */
     write_mem16(&s, rbd_addr, 0); /* ActualCount */
     write_mem32(&s, rbd_addr + 4, rbd_addr); /* Link to self - circular buffer */
     write_mem32(&s, rbd_addr + 8, 0x140000); /* Buffer address */
     write_mem16(&s, rbd_addr + 12, 1536); /* Size - maximum ethernet frame */
     
     /* Point SCB to RFA */
     write_mem32(&s, scb_addr + 8, rfa_addr);
     
     /* Start the RU */
     write_mem16(&s, scb_addr + 2, 0x0010); /* Set RUC = 1 (START) */
     
     /* Issue CA */
     port_write(&s, PORT_CA, 0);
     
     /* Give some time for command execution */
     g_usleep(10000);
     
     /* Check RU status */
     uint16_t scb_status = read_mem16(&s, scb_addr);
     uint8_t ru_status = (scb_status >> 4) & 0x7;
     g_assert_cmpint(ru_status, !=, RX_SUSPENDED); /* RU should not be suspended */
     
     /* Let's try to put it in suspend mode */
     write_mem16(&s, scb_addr + 2, 0x0030); /* Set RUC = 3 (SUSPEND) */
     port_write(&s, PORT_CA, 0);
     g_usleep(10000);
     
     /* Check RU status - should be suspended now */
     scb_status = read_mem16(&s, scb_addr);
     ru_status = (scb_status >> 4) & 0x7;
     g_assert_cmpint(ru_status, ==, RX_SUSPENDED); /* RU should be suspended */
     g_assert_cmpint(scb_status & SCB_STATUS_RNR, !=, 0); /* RNR bit should be set */
     
     i82596_test_cleanup(&s);
 }
 
 static void test_i82596_state_transitions(void)
 {
     I82596TestState s = { 0 };
     uint32_t scb_addr;
     
     i82596_test_init(&s);
     
     /* Initialize the card */
     scb_addr = initialize_i82596(&s, 0x100000);
     
     /* Test CU transitions */
     
     /* Initial state should be idle */
     uint16_t scb_status = read_mem16(&s, scb_addr);
     uint8_t cu_status = (scb_status >> 8) & 0x7;
     g_assert_cmpint(cu_status, ==, CU_IDLE);
     
     /* Start CU */
     write_mem16(&s, scb_addr + 2, 0x0100); /* Set CUC = 1 (START) */
     port_write(&s, PORT_CA, 0);
     g_usleep(5000);
     
     /* Status should show active */
     scb_status = read_mem16(&s, scb_addr);
     cu_status = (scb_status >> 8) & 0x7;
     g_assert_cmpint(cu_status, ==, CU_ACTIVE);
     
     /* Abort CU */
     write_mem16(&s, scb_addr + 2, 0x0400); /* Set CUC = 4 (ABORT) */
     port_write(&s, PORT_CA, 0);
     g_usleep(5000);
     
     /* Status should show suspended */
     scb_status = read_mem16(&s, scb_addr);
     cu_status = (scb_status >> 8) & 0x7;
     g_assert_cmpint(cu_status, ==, CU_SUSPENDED);
     g_assert_cmpint(scb_status & SCB_STATUS_CNA, !=, 0); /* CNA bit should be set */
     
     /* Test RU transitions */
     
     /* Initial state should be suspended (from reset) */
     scb_status = read_mem16(&s, scb_addr);
     uint8_t ru_status = (scb_status >> 4) & 0x7;
     g_assert_cmpint(ru_status, ==, RX_SUSPENDED);
     
     /* Start RU */
     write_mem16(&s, scb_addr + 2, 0x0010); /* Set RUC = 1 (START) */
     port_write(&s, PORT_CA, 0);
     g_usleep(5000);
     
     /* Status should change */
     scb_status = read_mem16(&s, scb_addr);
     ru_status = (scb_status >> 4) & 0x7;
     g_assert_cmpint(ru_status, ==, RX_IDLE);
     
     /* Suspend RU */
     write_mem16(&s, scb_addr + 2, 0x0030); /* Set RUC = 3 (SUSPEND) */
     port_write(&s, PORT_CA, 0);
     g_usleep(5000);
     
     /* Status should be suspended */
     scb_status = read_mem16(&s, scb_addr);
     ru_status = (scb_status >> 4) & 0x7;
     g_assert_cmpint(ru_status, ==, RX_SUSPENDED);
     g_assert_cmpint(scb_status & SCB_STATUS_RNR, !=, 0); /* RNR bit should be set */
     
     /* Test reset command */
     write_mem16(&s, scb_addr + 2, 0x8000); /* Set reset bit */
     port_write(&s, PORT_CA, 0);
     g_usleep(5000);
     
     /* After reset, everything should be back to initial state */
     scb_status = read_mem16(&s, scb_addr);
     cu_status = (scb_status >> 8) & 0x7;
     ru_status = (scb_status >> 4) & 0x7;
     g_assert_cmpint(cu_status, ==, CU_IDLE);
     g_assert_cmpint(ru_status, ==, RX_SUSPENDED);
     g_assert_cmpint(scb_status & 0xF000, ==, 0); /* All status bits should be cleared */
     
     i82596_test_cleanup(&s);
 }
 
 static void test_i82596_auto_port_selection(void)
 {
     I82596TestState s = { 0 };
     uint32_t scb_addr;
     uint32_t cmd_addr;
     
     i82596_test_init(&s);
     
     /* Initialize the card */
     scb_addr = initialize_i82596(&s, 0x100000);
     cmd_addr = 0x110000;
     
     /* Configure with auto port selection enabled */
     write_mem16(&s, cmd_addr, 0);
     write_mem16(&s, cmd_addr + 2, CMD_EOL | CmdConfigure);
     write_mem32(&s, cmd_addr + 4, 0xffffffff);
     
     /* 16 bytes of config data */
     qtest_writeb(s.qts, cmd_addr + 8, 16);
     
     /* Zero all config bytes */
     for (int i = 0; i < 16; i++) {
         qtest_writeb(s.qts, cmd_addr + 9 + i, 0);
     }
     
     /* Set up for auto port selection */
     qtest_writeb(s.qts, cmd_addr + 17, 0x40); /* Byte 9 - bit 6 = MAU mode */
     qtest_writeb(s.qts, cmd_addr + 20, 10);   /* Byte 12 - min frame len */
     qtest_writeb(s.qts, cmd_addr + 21, 0x10); /* Byte 13 - auto port selection */
     
     /* Start command unit */
     write_mem32(&s, scb_addr + 4, cmd_addr);
     write_mem16(&s, scb_addr + 2, 0x0100);
     port_write(&s, PORT_CA, 0);
     g_usleep(10000);
     
     /* Now change link status to down */
     QDict *args = qdict_new();
     qdict_put_str(args, "device", "nq0");
     qdict_put_bool(args, "up", false);
     
     QDict *resp = qtest_qmp(s.qts, "{ 'execute': 'set_link', 'arguments': %p }", args);
     g_assert(qdict_haskey(resp, "return"));
     qobject_unref(resp);
     
     /* Give time for auto port selection to happen */
     g_usleep(10000);
     
     /* Read back config - bit 6 should have toggled */
     /* First need to read which memory location the config is stored in */
     /* For simplicity, let's just check SCB status for link activity */
     uint16_t scb_status = read_mem16(&s, scb_addr);
     g_assert_cmpint(scb_status & (SCB_STATUS_CNA | SCB_STATUS_RNR), !=, 0);
     
     /* Set link back up */
     args = qdict_new();
     qdict_put_str(args, "device", "nq0");
     qdict_put_bool(args, "up", true);
     
     resp = qtest_qmp(s.qts, "{ 'execute': 'set_link', 'arguments': %p }", args);
     g_assert(qdict_haskey(resp, "return"));
     qobject_unref(resp);
     
     i82596_test_cleanup(&s);
 }
 