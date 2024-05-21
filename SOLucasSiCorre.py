

# !/usr/bin/env python

from hardware import *
import log

"""
-Se arreglo el priority if de creacion de listas y el if de is empty
Nota: Cuando hagamos el aging hay que modificar el add y la creación

Recordar que por ahora corre infinitamente. - COPIADA DE TP3


En este TP:
 # FCFS
    -fifo
 # PRIORIDAD
    -prioridad expropiativo fifo
    -prioridad no expropiativo fifo
 # ROUND-ROBIN
    -quantum fifo


KERNEL POR PARAMETRO RECIBE CUAL CORRER
    #clase que herede todo lo general que comparten todos
    --ADD
    --GET
    --hasToExpropiate(pcb1, pcb2)
class Schedulers:
    generalidades

class Fcfs_scheduler
    --sus cositas
class Priority_scheduler         -->  exp     NoExp
    --sus cositas
class Round_robin_scheduler
    --sus cositas


########### Diagrama de GANNT############
    --Consultar como hacerlo con tabulate
    --Como hacer cuando esta en waiting


"""


## emulates a compiled program
class Program:

    def __init__(self, name, instructions):
        self._name = name
        self._instructions = self.expand(instructions)

    @property
    def name(self):
        return self._name

    @property
    def instructions(self):
        return self._instructions

    def addInstr(self, instruction):
        self._instructions.append(instruction)

    def expand(self, instructions):
        expanded = []
        for i in instructions:
            if isinstance(i, list):
                ## is a list of instructions
                expanded.extend(i)
            else:
                ## a single instr (a String)
                expanded.append(i)

        ## now test if last instruction is EXIT
        ## if not... add an EXIT as final instruction
        last = expanded[-1]
        if not ASM.isEXIT(last):
            expanded.append(INSTRUCTION_EXIT)

        return expanded

    def __repr__(self):
        return "Program({name}, {instructions})".format(name=self._name, instructions=self._instructions)


## emulates an Input/Output device controller (driver)
class IoDeviceController:

    def __init__(self, device):
        self._device = device
        self._waiting_queue = []
        self._currentPCB = None

    def runOperation(self, pcb, instruction):
        pair = {'pcb': pcb, 'instruction': instruction}
        # append: adds the element at the end of the queue
        self._waiting_queue.append(pair)
        # try to send the instruction to hardware's device (if is idle)
        self.__load_from_waiting_queue_if_apply()

    def getFinishedPCB(self):
        finishedPCB = self._currentPCB
        self._currentPCB = None
        self.__load_from_waiting_queue_if_apply()
        return finishedPCB

    def __load_from_waiting_queue_if_apply(self):
        if (len(self._waiting_queue) > 0) and self._device.is_idle:
            ## pop(): extracts (deletes and return) the first element in queue
            pair = self._waiting_queue.pop(0)
            # print(pair)
            pcb = pair['pcb']
            instruction = pair['instruction']
            self._currentPCB = pcb
            self._device.execute(instruction)

    def __repr__(self):
        return "IoDeviceController for {deviceID} running: {currentPCB} waiting: {waiting_queue}".format(
            deviceID=self._device.deviceId, currentPCB=self._currentPCB, waiting_queue=self._waiting_queue)


################################### Interruption Handlers ###################################
class AbstractInterruptionHandler:
    ## falta el de timeOut
    def __init__(self, kernel):
        self._kernel = kernel

    @property
    def kernel(self):
        return self._kernel

    def execute(self, irq):
        log.logger.error("-- EXECUTE MUST BE OVERRIDEN in class {classname}".format(classname=self.__class__.__name__))

    def runIfPossible(self, pcb):
        if  self._kernel.pcbTable.isActive():
            acPcb = self._kernel.pcbTable.activeProcess()
            if self._kernel.scheduler.mustExpropiate(acPcb, pcb):
                self._kernel.dispatcher.save(acPcb)
                acPcb.setState("Ready")
                self._kernel.scheduler.add(acPcb)
                self._kernel.dispatcher.load(pcb)
                print(f"intente expropiar por {pcb.path()}")
            else:
                self._kernel.scheduler.add(pcb)
                print("agregue algo a la cola")

        else:
            self._kernel.dispatcher.load(pcb)
            
    def runNextIfPossible(self):
        if not self._kernel.scheduler.isEmpty():
            next_program = self._kernel.scheduler.get()
            print(f"mande algo a correr {next_program.path()}")
            #log.logger.info(next_program)
            self._kernel.dispatcher.load(next_program)
        #log.logger.info(self._kernel.scheduler)


class KillInterruptionHandler(AbstractInterruptionHandler):

    def execute(self, irq):
        pcb = self._kernel.pcbTable.activeProcess()
        pcb.setState("Finished")
        self._kernel.pcbTable.setActiveProcess(False)
        HARDWARE.cpu.pc = -1  ## dejamos el CPU IDLE
        self.runNextIfPossible()


class NewInterruptionHandler(AbstractInterruptionHandler):

    def execute(self, irq):
        diccionario = irq.parameters
        program = diccionario["program"]
        priority = diccionario["priority"]
        base_dir = self._kernel.loader.load(program)
        pcb = self._kernel.pcbTable.newPcb(base_dir, program.name, priority)
        self.runIfPossible(pcb)


class IoInInterruptionHandler(AbstractInterruptionHandler):

    def execute(self, irq):
        operation = irq.parameters
        pcb = self._kernel.pcbTable.activeProcess()
        self._kernel.dispatcher.save(pcb)
        self._kernel.ioDeviceController.runOperation(pcb, operation)
        log.logger.info(self._kernel.ioDeviceController)
        self.runNextIfPossible()


class IoOutInterruptionHandler(AbstractInterruptionHandler):

    def execute(self, irq):
        pcb = self._kernel.ioDeviceController.getFinishedPCB()
        self.runIfPossible(pcb)

class TimeoutInterruptionHandler(AbstractInterruptionHandler):

    def execute(self, irq):
        if (not self._kernel.scheduler.isEmpty()):
            acPcb = self._kernel.pcbTable.activeProcess()
            sigPcb = self.kernel.scheduler.get()
            self._kernel.dispatcher.save(acPcb)
            acPcb.setState("Ready")
            self._kernel.scheduler.add(acPcb)
            self._kernel.dispatcher.load(sigPcb)
            print("expropie")


################################### OS Core & Pcb components ###################################


class Kernel:

    def __init__(self, scheduler):
        ## tiene que recibir un scheduler para trabajar en el init, y una interrupcion time out
        ##setup pcbTable
        self.pcbTable = Pcb_table()

        ## setup loader
        self.loader = Loader(HARDWARE.memory)

        ##setup dispacher
        self.dispatcher = Dispatcher(HARDWARE.cpu, HARDWARE.mmu, self.pcbTable)

        ##setup scheduler
        self.scheduler = scheduler


        ## setup interruption handlers
        killHandler = KillInterruptionHandler(self)
        HARDWARE.interruptVector.register(KILL_INTERRUPTION_TYPE, killHandler)

        ioInHandler = IoInInterruptionHandler(self)
        HARDWARE.interruptVector.register(IO_IN_INTERRUPTION_TYPE, ioInHandler)

        ioOutHandler = IoOutInterruptionHandler(self)
        HARDWARE.interruptVector.register(IO_OUT_INTERRUPTION_TYPE, ioOutHandler)

        newHandler = NewInterruptionHandler(self)
        HARDWARE.interruptVector.register(NEW_INTERRUPTION_TYPE, newHandler)

        timeoutHandler = TimeoutInterruptionHandler(self)
        HARDWARE.interruptVector.register(TIMEOUT_INTERRUPTION_TYPE, timeoutHandler)

        ## controls the Hardware's I/O Device
        self._ioDeviceController = IoDeviceController(HARDWARE.ioDevice)

    @property
    def ioDeviceController(self):
        return self._ioDeviceController

    def load_program(self, program):
        # loads the program in main memory
        progSize = len(program.instructions)
        for index in range(0, progSize):
            inst = program.instructions[index]
            HARDWARE.memory.write(index, inst)

    ## emulates a "system call" for programs execution
    def run(self, program, priority):
        programPriorityData = {"program": program, "priority": priority}
        newIRQ = IRQ(NEW_INTERRUPTION_TYPE, programPriorityData)
        HARDWARE.interruptVector.handle(newIRQ)

    def __repr__(self):
        return "Kernel "


"""class ReadyQueue:

    ##setup readyQueue

      def __init__(self):
        self.rq = ReadyQueue()
        self._queue = []

    def sigPrograma(self):
        return self._queue.pop(0)

    def addProgram(self, pcb):
        self._queue.append(pcb)
        pcb.setState("Ready")

    def isReadyQueueEmpty(self):
        return len(self._queue) == 0"""


################################### Queues & Auxiliaries ###################################


class Dispatcher:

    def __init__(self, cpu, mmu, pcbTable):
        self._cpu = cpu
        self._mmu = mmu
        self._pcbTable = pcbTable

    def load(self, pcb):
        ## tiene que resetear el timer
        ##HARDWARE.timer.reset()
        self._cpu.pc = pcb.pc()
        self._mmu.baseDir = pcb.baseDir()
        pcb.setState("Running")
        self._pcbTable.setActiveProcess(True)
        self._pcbTable.saveActiveProcess(pcb)
        HARDWARE.timer.reset()
        ## separa el estado del proceso activo del pcb a dos partes, el estado del pcb y el de la tabla

    def save(self, pcb):
        pcv = self._cpu.pc
        pcb.setPc(pcv)
        self._cpu.pc = -1
        pcb.setState("Waiting")
        self._pcbTable.setActiveProcess(False)



##class WaitingQueue: sin completar


class Pcb:

    def __init__(self, pidv, baseDirv, pcv, statev, pathv, priority):
        ## hay que agregar prioridad por parametro en la int de new
        self._pid = pidv
        self._baseDir = baseDirv
        self._pc = pcv
        self._state = statev
        self._path = pathv
        self._priority = priority
        self._tick = None
        self._falsaPrioridad = priority

    def pid(self):
        return self._pid

    def pidSet(self, pidNew):
        self._pid = pidNew
        return self

    def baseDir(self):
        return self._baseDir

    def pc(self):
        return self._pc

    def state(self):
        return self._state

    def path(self):
        return self._path

    def setState(self, state):
        self._state = state

    def isFinished(self):
        return self._state == "Finished"

    def __repr__(self):
        return f"{self._pc} --pc \n {self._priority} --priority"

    def setPc(self, pcv):
        self._pc = pcv

    def priority(self):
        return self._priority

    def setAddTick(self, tick):
        self._tick = tick

    def calcularFPriority(self, tick, tiempoAshing):
        prioridadFalsa = int(abs(self._tick - tick)/tiempoAshing)
        self._falsaPrioridad = min(self._priority, prioridadFalsa)

    def prioridadFalsa(self):
        return self.priority() - self._falsaPrioridad

    def pasarAQ(self, state, tick):
        self._state = state
        self._tick = tick
        self._falsaPrioridad = 0

class Pcb_table:

    def __init__(self):
        self._table = {}
        self.active_process = False
        self.pidSiguiente = 0
        self._activePcb = []

    ## toma un pcb y actualiza su pid al nuebvo pud asignado y lo agrega con su pid como key
    def add(self, pcb):
        self._table[self.pidSiguiente] = pcb.pidSet(self.pidSiguiente)
        self.pidSiguiente += 1

    ##El pid buscadp debe existir en la tabla
    def get(self, pid):
        return self._table[pid]

    def remove(self, pid):
        del self._table[pid]

    def getNewPid(self):
        return self.pidSiguiente

    def isActive(self):
        return self.active_process

    def newPcb(self, baseDir, path, priority):
        pid = self.getNewPid()
        pcb = self._table[pid] = Pcb(pid, baseDir, 0, "New", path, priority)
        self.pidSiguiente += 1
        return pcb

    def setActiveProcess(self, bool):
        self.active_process = bool

    def finishedAll(self):
        comparador = True
        for pcb in self.allPcbs():
            comparador = comparador and pcb.isFinished

        return bool

    def allPcbs(self):
        return self._table.values()

    def activeProcess(self):
        ## esto tiene de precondicion que hay uno con el estado en Running, sino explota
        return self.get(self._activePcb)

    def __repr__(self):
        return f"{self._table} --tabla \n {self.pidSiguiente}"

    def saveActiveProcess(self, pcb):
        self._activePcb = pcb.pid()


class Loader:

    def __init__(self, memory):
        self._nextCellAviable = 0
        self._memory = memory

    def next(self):
        return self._nextCellAviable

    def load(self, prg):
        baseDir = self._nextCellAviable
        for instruction in prg.instructions:
            self._memory.write(self._nextCellAviable, instruction)
            self._nextCellAviable += 1

        return baseDir



class Abstract_Schedulers:
    def mustExpropiate(self, currentPcb, NewPcb):
        ## esto retorna si se debe de expropiar, el imExpropiative esta igual definido para todos y el que cambia es el segundo
        return self.imExpropiative() and self.secondWin(currentPcb, NewPcb)

class Fcfs_scheduler(Abstract_Schedulers):

    def __init__(self):
        self._queue = []

    def get(self):
        return self._queue.pop(0)

    def add(self, pcb):
        self._queue.append(pcb)
        pcb.setState("Ready")

    def isEmpty(self):
        return len(self._queue) == 0

    def imExpropiative(self):
        return False

    def secondWin(self,currentPcb, NewPcb):
        return False

## ashing cada n ticks tiene que subir la prioridad de los pcb que llevan metidos ahi
class PriorityScheduler(Abstract_Schedulers):
    def __init__(self, tipoExp, tiempoDeAshing):
        self._tipoExp = tipoExp  ## booleano que decide si soy expropiativo
        self._pq = []
        self._tiempoAshing = tiempoDeAshing

    def imExpropiative(self):
        return self._tipoExp

    def secondWin(self, currentPcb, newPcb):
        return currentPcb.priority() < newPcb.priority()

    def get(self):
        """listaFalsa = self._pq
        print("algo")
        print(self.len())
        if len(listaFalsa) != 0:
            indice = 0
            while listaFalsa[indice].priority() != self._maximaPrioridad:
                indice += 1
                if len(listaFalsa) == 0:
                    break
            pcbABorrar = listaFalsa[0]
            self._pq.remove(pcbABorrar)
            print(self.len())
            return (pcbABorrar)"""
        self.ashing()
        pcbMaximo_maximo, posicion = min(enumerate(self._pq), key=lambda x: x[1].prioridadFalsa())
        return self._pq.pop(pcbMaximo_maximo)

    def ashing(self):
        ##for pcb in self.pq:
         ##   pcb.actFalsePriorityIfPosibble()
        tiempoAshing = self._tiempoAshing
        for pcb in self._pq:
            pcb.calcularFPriority(HARDWARE.clock.currentTick,tiempoAshing )

    def add(self, pcb):
        self._pq.append(pcb)
        pcb.pasarAQ("Ready", HARDWARE.clock.currentTick)
        print("agregar")

    def isEmpty(self):
        return len(self._pq) == 0

    def len(self):
        return len(self._pq)

    def __repr__(self):
        return f"queue: {self._pq}"



class Round_robin_scheduler(Abstract_Schedulers):
## el mustExpropiate tiene que tener una defiicion diferente aca porque sino hace cosas que no deberia
    def __init__(self, quantum):
        self._rq = []
        HARDWARE.timer.quantum = quantum

    def add(self, pcb):
        self._rq.append(pcb)
        pcb.setState("Ready")

    def get(self):
        return self._rq.pop(0)

    def isEmpty(self):
        return len(self._rq) == 0

    def mustExpropiate(self, currentPcb, NewPcb):
        return False



