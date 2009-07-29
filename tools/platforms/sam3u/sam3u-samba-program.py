#!/usr/bin/env python

# Script to program the Atmel SAM3U using the sam-ba tool.

import os
import re
import sys
import time
import signal
import optparse
import tempfile
import subprocess

parser = optparse.OptionParser()

parser.add_option("-p", "--port",
        action="store",
        type="string",
        dest="port",
        default="/dev/ttyUSB0",
        help="Port where the SAM3U can be found.")
parser.add_option("-b", "--binfile",
        action="store",
        type="string",
        dest="binfile",
        default="build/sam3u_ek/main.bin.out",
        help="Binary file that should be programmed into the flash.")
parser.add_option("-t", "--target",
        action="store",
        type="string",
        dest="target",
        default="AT91SAM3U4-EK",
        help="Target board type.")
parser.add_option("-d", "--debug",
        action="store_true",
        dest="DEBUG",
        default=False,
        help="Set the debug mode.")

(cmdOptions, args) = parser.parse_args()

class samba:
    def __init__(self):
        self.expect_timeout = False

        # check to make sure binary file exists
        if not os.path.isfile(cmdOptions.binfile):
            print '"%s" does not exist. Exiting.' % cmdOptions.binfile
            sys.exit(1)
        # once we switch to python 2.6, we should do this
        #self.f = tempfile.NamedTemporaryFile(delete=False)
        self.f = file('/tmp/samba.tcl', 'w+')
        self.f.write("""FLASH::Init 0
    send_file {Flash 0} "%s" 0x80000 0
    FLASH::ScriptGPNMV 2
    """%(cmdOptions.binfile,))
        self.f.flush()

        try:
            error = False

            # check if SAMBA bootloader is here
            foundBootloader = False
            while not foundBootloader:
                print "Checking for programmer"
                lsusb_proc = subprocess.Popen('lsusb -d 03eb:6124', shell=True,
                        stdout=subprocess.PIPE)
                r = re.compile("SAMBA bootloader")
                lsusb_proc.wait()
                matches = r.findall(lsusb_proc.stdout.readline())
                if len(matches) == 0:
                    print """\n Couldn't find SAM-BA bootloader device on the USB bus.
     Please close JP1 on the development kit and reboot the system (hit NRSTB button)!\n"""
                    time.sleep(2)
                else:
                    foundBootloader = True

            print "Programmer Found!"

            print "Remove JP1 and hit [Enter]"
            a = raw_input()

            samba_cmd = "DISPLAY=:0 sam-ba %s %s %s"%(cmdOptions.port, cmdOptions.target,
                    self.f.name)
            samba_proc = subprocess.Popen(samba_cmd, shell=True, stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE)
            r = re.compile("sam-ba: not found")
            if len(r.findall(samba_proc.stderr.readline())) != 0:
                print "Couldn't find 'sam-ba'. Please make sure it is in your PATH!"
                self.cleanup()
                sys.exit(1)
            try:
                self.expect(samba_proc.stdout, "-I- Found processor : at91sam3u4")
            except RuntimeError:
                print "Couldn't find processor! Make sure the port '%s' is correct."%(cmdOptions.port)
                self.cleanup()
                sys.exit(1)
            try:
                self.expect(samba_proc.stdout, "-I- Command line mode : Execute script file")
            except:
                print "Couldn't execute script!"
                self.cleanup()
                sys.exit(1)
            try:
                self.expect(samba_proc.stdout, "-I- GPNVM1 set")
            except:
                print "Couldn't program the device!"
                self.cleanup()
                sys.exit(1)

            print "Done! Reboot your system (hit NRSTB button)."

        finally:
            pass

    def cleanup(self):
        self.f.close()
        os.unlink(self.f.name)


    def alarmHandler(self, signum, frame):
        self.expect_timeout = True

    # Wait until expected pattern is received on the given filehandle.
    def expect(self, fh, pat, timeout=3):
        r = re.compile(pat)

        expect_found = False

        if (timeout != -1):
            signal.signal(signal.SIGALRM, self.alarmHandler)
            signal.alarm(timeout)

        while (not expect_found and not self.expect_timeout):
            try:
                line = fh.readline().strip()
                if cmdOptions.DEBUG:
                    print line
                    time.sleep(0.2)
            except:
                # Possibly due to alarm
                break
            matches = r.findall(line)
            if (len(matches) != 0):
                expect_found = True
                break

        signal.alarm(0)
        if (not expect_found):
            raise RuntimeError, "Did not receive expected pattern '%s'" % pat


if __name__ == "__main__":
    s = samba()

