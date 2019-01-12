import random


DATA = """fosdfaoaijg\x02ABCDEFGHIJKLMN\x03yrcoynrt\x020123456789ABCD\x03lkjashdflkjh"""

STX = '\x02'
ETX = '\x03'

FIFO = 'data-pipe'

# assert that it does not matter where we start writing
pw = int(random.random() * 32)
pr = 0
vbl = 0

def gen_char():
  global vbl

  while True:
    # print("Opening FIFO...")
    with open(FIFO, 'rb', buffering=0) as fifo:
      # print("FIFO opened")
      while True:
        data = fifo.read()
        if len(data) == 0:
          # print("Writer closed")
          vbl = 0
          break
        for c in data:
          yield c
        # print('Read: "{0}"'.format(data))

buf = [' '] * 32

def fmt_buf(b):
  s = []
  for c in b:
    s.append('%02x' % (ord(c)))
  return ' '.join(s).upper()

serial = gen_char()

def loop():
  global buf, pw, pr, vbl, serial

  # "read from serial"
  buf[pw] = serial.next()

  if pw < 16:
    pr = pw + 16
    buf[pr] = buf[pw]
  else:
    pr = pw
    buf[pw - 16] = buf[pw]

  # print("PW=%02d, PR=%02d, BUF is %r" % (pw, pr, fmt_buf(buf)))

  if buf[pr - 15] == STX and buf[pr] == ETX:
    msg = ''
    dbuf = buf[pr-14:] # equivalent to pointer arithmetic
    for i in range(14):
      msg += dbuf[i]
    print("VBL%d %s" % (vbl, fmt_buf(msg)))
    vbl += 1
  
  pw = (pw + 1) % 32


if __name__ == '__main__':
  while True:
    loop()
