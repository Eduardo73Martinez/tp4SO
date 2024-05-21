from hardware import *
from so import *

#SOLucasSiCorre
import log


##
##  MAIN 
##
if __name__ == '__main__':
    log.setupLogger()
    log.logger.info('Starting emulator')

    ## setup our hardware and set memory size to 25 "cells"
    HARDWARE.setup(25)

    ## new create the Operative System Kernel
    # "booteamos" el sistema operativo
    # fifo = Fcfs_scheduler()
    #rr = Round_robin_scheduler(4)
    # priorityNoExp = PriorityScheduler(False,4)
    # priorityExp = PriorityScheduler(False,4)
    
    kernel = Kernel()

    prg1 = Program("prg1.exe", [ASM.CPU(2), ASM.IO(), ASM.CPU(3), ASM.IO(), ASM.CPU(2)])
    prg2 = Program("prg2.exe", [ASM.CPU(4), ASM.IO(), ASM.CPU(1)])
    prg3 = Program("prg3.exe", [ASM.CPU(3)])


    HARDWARE.cpu.pc = 0

    ## start
    HARDWARE.switchOn()

    # executamos los programas "concurrentemente"
    kernel.run(prg1, 3)
    kernel.run(prg2, 2)
    kernel.run(prg3, 0)

