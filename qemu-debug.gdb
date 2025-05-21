set debuginfod enabled off

# target remote :1234

# ---- BREAKPOINTS FOR TRANSMIT PATH ----
break i82596.c:137     
break i82596.c:142      
break i82596.c:149      
break i82596.c:158      
break i82596.c:165     
break i82596.c:172      
break i82596.c:175      
break i82596.c:178      

# ---- CONDITIONAL BREAKPOINTS FOR LOOPBACK MODE ----
# break i82596.c:172 if I596_LOOPBACK == 1
break i82596.c:178 if s->config[3] >> 6 == 1  

# ---- BREAKPOINTS FOR RECEIVE PATH ----
break i82596.c:538     
break i82596.c:546     
break i82596.c:551     
break i82596.c:558     
break i82596.c:563     
break i82596.c:605     
break i82596.c:607     
break i82596.c:612     
break i82596.c:638      
break i82596.c:645      
break i82596.c:675     
break i82596.c:678      
break i82596.c:681     
break i82596.c:690      
break i82596.c:693     

# ---- IRQ SIGNAL HANDLING ----
break i82596.c:504    

# ---- INITIALIZATION ----
break i82596_common_init
break lasi_82596_reset
break lasi_82596_realize

# ---- On BREAKPOINT: Print backtrace and continue ----
commands
  bt
  continue
end

# ---- Run QEMU with full args ----
run -accel tcg,thread=multi \
    -m 512 \
    -drive if=scsi,bus=0,index=6,file=OS_test/hpux.img,format=raw \
    -net nic,model=lasi_82596 -net user \
    -boot c -serial mon:stdio -nographic
