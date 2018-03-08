start:
  ldi r2, 0
  ldi r1, 1
  stf r1, 0xc8
  done 4

update:
  stf r1, 0xc8
  stf r2, 0x2b
  ldf r2, 0x00
  cmphs r2, r7
  bt alldone
  btsti r5, 8
  bt read16bit
  ldf r2, 0x09

output:
  sub r2, r6
  and r2, r5
  stf r2, 0x2b

  done 4
  btsti r1, 0
  bt update

alldone:
  done 3
  done 4
  btsti r1, 0
  bt update


read16bit:
  ldf r2, 0x0a
  bt output
