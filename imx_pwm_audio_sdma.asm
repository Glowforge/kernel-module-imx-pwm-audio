# arguments:
#   r5  - sample size bitmask; (1<<bits)-1
#   r6  - offset; value subtracted from each sample (e.g. to convert signed to unsigned)
#   r7  - physical address of buffer end
#   MDA - PWMSAR register
#   MSA - physical address of buffer start
#   MS  - 0x00100000 (destination address frozen; source address postincrement; start in read mode)
#   PDA - EPIT_SR address
#   PS  - 0x000c0400 (destination address frozen; 32-bit write size; start in write mode)
#
# register usage:
#   r1  - constant, always 1
#   r2  - temporary

start:
  ldi r2, 0
  ldi r1, 1     # constant
  # Do nothing the first time around.
  # Clear the timer interrupt and wait for another timer tick.
  # This ensures we perform exactly one iteration when the script starts.
  # (The very first time the script is loaded, execution starts when the channel
  # priority is set. If there is also a pending timer event, we could wind up
  # doing two ticks worth of work in one.)
  stf r1, 0xc8  # (PD)
  done 4

update:
  # clear EPIT interrupt flag
  stf r1, 0xc8  # (PD)

  # reload the previous sample to prevent a glitch
  stf r2, 0x2b # (MD|SZ32|FL)

  # stop if we've advanced past the end of the sample data
  ldf r2, 0x00  # (MSA)
  cmphs r2, r7
  bt alldone

  # get new sample (1 or 2 bytes)
  btsti r5, 8
  bt read16bit
  ldf r2, 0x09 # (MD|SZ8) read one byte; increment source address by 1

output:
  # subtract offset
  sub r2, r6
  # mask out unused bits
  and r2, r5
  # output new sample
  stf r2, 0x2b # (MD|SZ32|FL)

  # wait for next timer interrupt
  done 4
  btsti r1, 0   # always true
  bt update

alldone:
  done 3  # trigger a host interrupt
  done 4
  # when script is restarted, jump back to the beginning
  btsti r1, 0   # always true
  bt update


read16bit:
  ldf r2, 0x0a # (MD|SZ16) read two bytes; increment source address by 2
  bt output # T is still set
