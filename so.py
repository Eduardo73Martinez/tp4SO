from hardware import *
import log



## emulates a compiled program
class Program():

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
class IoDeviceController():

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
            #print(pair)
            pcb = pair['pcb']
            instruction = pair['instruction']
            self._currentPCB = pcb
            self._device.execute(instruction)


    def __repr__(self):
        return "IoDeviceController for {deviceID} running: {currentPCB} waiting: {waiting_queue}".format(deviceID=self._device.deviceId, currentPCB=self._currentPCB, waiting_queue=self._waiting_queue)

## emulates the  Interruptions Handlers
#ESTADOS
TERMINATED_STATE = "Terminated"
WAITING_STATE    = "Waiting"
READY_STATE      = "Ready"
RUNING_STATE     = "Runing"
NEW_STATE        = "New" 



class AbstractInterruptionHandler():
    def __init__(self, kernel):
        self._kernel = kernel

    @property
    def kernel(self):
        return self._kernel

    def execute(self, irq):
        log.logger.error("-- EXECUTE MUST BE OVERRIDEN in class {classname}".format(classname=self.__class__.__name__))

    
    def cargarEnCpuConModoExpropiativo(self, pcbCorriendo, pcbACargar):
          
        if self.kernel.getScheduler.mustExpropiate(pcbCorriendo, pcbACargar):
            self.kernel._dispatcher.save(pcbCorriendo)
            pcbCorriendo.setState(READY_STATE)
            self.kernel.getScheduler.addPCB(pcbCorriendo) 
            self.kernel._dispatcher.load(pcbACargar)
            print(f"intente expropiar por {pcb.path()}")
        else:
            self.kernel.getScheduler.addPCB(pcbACargar) 
            print("agregue algo a la cola")

    def loadProgramInDispacher(self):
        if  not self._kernel._scheduler.isEmpty(): 
            pcbACargar = self.kernel.getScheduler.getNext()
            #pcbCorriendo = self.getKernel.getPcbTable.getRunningPCB
            #self.cargarEnCpuConModoExpropiativo(pcbCorriendo, pcbACargar)
            self.getKernel._dispatcher.load(pcbACargar)

        #condicion de corte
        if not (not self.kernel.getScheduler.getReadyQueue.isReadyQueueEmpty()) and HARDWARE.ioDevice.is_idle:
            log.logger.info(" Program Finished ")
            HARDWARE.switchOff()
    
    def loadProgramIfNotRuningInTable(self, pcb): 
        hayAlguienCorriendo = self.getKernel.getPcbTable.getIsRunning
        if (hayAlguienCorriendo): 
            pcbCorriendo = self.getKernel.getPcbTable.getRunningPCB()
            self.cargarEnCpuConModoExpropiativo(pcbCorriendo, pcb)
        else: 
            pcb.setstate(RUNING_STATE) 
            self.kernel.getPcbTable.setRunningPCB = pcb 
            self.kernel._dispatcher.load(pcb) 

    @property
    def getKernel(self):
        return self._kernel


class KillInterruptionHandler(AbstractInterruptionHandler): #TODOS LOS HANDLERS YA ESTAN LISTOS, SE SUPONE

    def execute(self, irq):
        pcb = self.kernel.getPcbTable.getRunningPCB  # ACA ESTÁ EL ERROR, VER FUNCION RUNNINGPCB
        pcb.setstate(TERMINATED_STATE)   # cambia el estado a "TERMINATED"
        self.kernel._dispatcher.save(pcb)
        log.logger.info(" Program Finished ")
        #self.kernel._PcbTable.runningPCB = None
        self.loadProgramInDispacher()
        

class IoInInterruptionHandler(AbstractInterruptionHandler):

    def execute(self, irq):
        operation = irq.parameters
        pcb = self.kernel.getPcbTable.getRunningPCB
        pcb.setstate(WAITING_STATE)  # cambia el estado a "Waiting"
        self.kernel._dispatcher.save(pcb)
        #pcb = {'pc': HARDWARE.cpu.pc} # porque hacemos esto ???
        self.kernel.ioDeviceController.runOperation(pcb, operation)
        log.logger.info(self.kernel.ioDeviceController)
        self.loadProgramInDispacher()
            


class IoOutInterruptionHandler(AbstractInterruptionHandler):

    def execute(self, irq):
        pcb = self.kernel.ioDeviceController.getFinishedPCB()
        log.logger.info(self.kernel.ioDeviceController)
        self.loadProgramIfNotRuningInTable(pcb)

class NewInterruptionHandler(AbstractInterruptionHandler):
    #(self, pidv, baseDirv, statev, pathv):

    def execute(self,irq):
        program  = irq.parameters['program']                    # lista instrucciones 
        priority = irq.parameters['priority']                   # conseguimos la prioridad del prog
        pid      = self.kernel.getPcbTable.getNewPid()          # conseguimos el pid       
        baseDir  = self.getKernel.getLoader.getBaseDir          # conseguimos el baseDir   
        path     = program.name                                 # conseguimos el nombre del prog 
        pcb      = Pcb(pid, baseDir, NEW_STATE, path, priority) # inicializamos el pcb
        baseDir  = self.getKernel.getLoader.write(program)
        pcb._baseDir = baseDir
        self.loadProgramIfNotRuningInTable(pcb)
        self.kernel.getPcbTable.addPcb(pcb)

class TimeoutInterruptionHandler(AbstractInterruptionHandler):

    def execute(self, irq):
        if (not self._kernel._scheduler.isEmpty()):
            acPcb = self._kernel.pcbTable.runningPCB()
            sigPcb = self.kernel._scheduler.getNext()
            self._kernel.dispatcher.save(acPcb)
            acPcb.setState("Ready")
            self._kernel.scheduler.add(acPcb)
            self._kernel.dispatcher.load(sigPcb)
            print("expropie")
        
        
## ####################################### --SCHEDULERS-- ########################################################  
class AbstractScheduler():
    
    def __init__(self):
        self._tick       = HARDWARE.clock.currentTick  #preguntar que funcion va (que muestre el tick actual)[LISTO]
        self._readyQueue = ReadyQueue()
        
    def addPCB(self, pcb):
        self._readyQueue.addProgram(pcb)
        pcb.setstate(READY_STATE)
        
    def getNext(self):
        return self._readyQueue.sigPrograma()

    @property
    def getTick(self):
        return self._tick

    @getTick.setter
    def setTick(self, nuevoTick):
        self._tick = nuevoTick

    def mustExpropiate(self, pcbInCPU, pcbACargar):
        return False
    
    @property
    def getReadyQueue(self):
        return self._readyQueue
    
    def isEmpty(self):
        return self._readyQueue.isReadyQueueEmpty()
    

class FCFSScheduler(AbstractScheduler):
    def __init__(self):
        super().__init__()
    
        
class PriorityNoExpropiativoScheduler(AbstractScheduler):
    def __init__(self):
        super().__init__()
        self._prioridad    = [[], [], [], [], []]

    def addPCB(self, pcb):
        self._prioridad[pcb.priority()].append(pcb)

    def aging(self):
        for n in self._prioridad[1:4]: 
            pcb = n[0]
            if (pcb.getTickLLegada() - self.getTick()) >= 4:
                pcb.setPriority(pcb.getPriority() - 1)
                pcb.setTickLLegada(self.getTick())
                n.remove(0)
                self._prioridad[pcb.getPriority()].add(pcb) 
                
class PriorityExpropiativoScheduler(PriorityNoExpropiativoScheduler):
    def __init__(self): 
        super().__init__()

    def mustExpropiate(self, pcbInCPU, pcbACargar):
        return pcbInCPU.getPriority < pcbACargar.getPriority

class RoundRobinScheduler(FCFSScheduler):
    
    def __init__(self, quantum):
        super().__init__()
        HARDWARE.timer.quantum = quantum

# emulates the core of an Operative System
class Kernel():


    def __init__(self):
        self._PcbTable = PcbTable()
        self._loader   = Loader(HARDWARE.memory)
        self._dispatcher = Dispatcher(HARDWARE.cpu,HARDWARE.mmu)
        #self._scheduler = PriorityNoExpropiativoScheduler()
        #self._scheduler = RoundRobinScheduler(4)      
        #self._scheduler = PriorityExpropiativoScheduler()
        self._scheduler = FCFSScheduler()                          ## ACA CAMBIO EL SCHEDULER ##

        ## setup interruption handlers
        newHandler = NewInterruptionHandler(self)
        HARDWARE.interruptVector.register(NEW_INTERRUPTION_TYPE, newHandler)

        killHandler = KillInterruptionHandler(self)
        HARDWARE.interruptVector.register(KILL_INTERRUPTION_TYPE, killHandler)

        ioInHandler = IoInInterruptionHandler(self)
        HARDWARE.interruptVector.register(IO_IN_INTERRUPTION_TYPE, ioInHandler)

        ioOutHandler = IoOutInterruptionHandler(self)
        HARDWARE.interruptVector.register(IO_OUT_INTERRUPTION_TYPE, ioOutHandler)

        timeoutHandler = TimeoutInterruptionHandler(self)
        HARDWARE.interruptVector.register(TIMEOUT_INTERRUPTION_TYPE, timeoutHandler)

        ## controls the Hardware's I/O Device
        self._ioDeviceController = IoDeviceController(HARDWARE.ioDevice)


    @property
    def ioDeviceController(self):
        return self._ioDeviceController

    ## emulates a "system call" for programs execution
    def run(self, program, priority):
        parameters = {'program': program, 'priority': priority}
        newIRQ = IRQ(NEW_INTERRUPTION_TYPE, parameters)
        HARDWARE.interruptVector.handle(newIRQ)

    @property
    def getLoader(self):
        return self._loader

    def __repr__(self):
        return "Kernel "

    @property
    def getPcbTable(self):
        return self._PcbTable
    
    @getPcbTable.setter
    def setPcbTable(self, nuevoValor):
        self._PcbTable = nuevoValor

    @property 
    def getScheduler(self): 
        return self._scheduler 

    def __repr__(self):
        return "Kernel "
    
    def getDispatcher(self): 
        return self._dispatcher
    


class ReadyQueue:

    def __init__(self):
        self.queue = []

    def sigPrograma(self):
        return self.queue.pop(0)

    def addProgram(self, pcb):
        pcb.setTickLLegada = HARDWARE.clock.currentTick   #preguntar cual muestra el tick actual(LISTO)
        self.queue.append(pcb)

    def isReadyQueueEmpty(self):
        return len(self.queue) == 0
    
    def remove(self, pid):
        self._table = [pcb for pcb in self._table if pcb["pid"] != pid]

class Pcb:

    def __init__(self, pidv, baseDirv, statev, pathv, priority):
        self.pid     = pidv
        self.baseDir = baseDirv
        self.state   = statev
        self.pc      = 0
        self.path    = pathv
        self.priorityFija = priority
        self.priority     = priority
        self.tickLLegada  = 0

    def state(self):
        return self.state
    
    def setstate(self, newState):
        self.state = newState

    def pid(self):
        return self.pid

    def pidSet(self, pidNew):
        self.pid = pidNew
        return self

    def baseDir(self):
        return self.baseDir

    def pc(self):
        return self.pc

    def path(self):
        return self.path

    def isFinished(self):
        return self.state == TERMINATED_STATE

    def __repr__(self):
        return f"{self.pc} --pc \n {self.pid} --pid"

    def setPc(self, pcv):
        self.pc = pcv
    
    @property
    def getTickLLegada(self): 
        return self.tickLLegada

    @getTickLLegada.setter
    def setTickLLegada(self, nuevoTick): 
        self.tickLLegada = nuevoTick

    @property
    def getPriority(self): 
        return self.priority

    @getPriority.setter
    def setPriority(self, priority): 
        self.priority = priority


class PcbTable:
    def __init__(self):
        self.table = {}
        self.isRunning = False 
        self.pidSiguiente = 0
        self.runningPCB = None

    #agrego un pcb a la tabla y lo pongo con su pid como key
    def addPcb(self, pcb):
        self.table[self.pidSiguiente] = pcb

    def incrementarPid(self):
        self.pidSiguiente += 1    

    #buscamos un pcb con un pid dado
    def getPcb(self, pid ):
        return self.table[pid]

    def remove(self, pid):
        del self.table[pid]

    def getNewPid(self):
        self.incrementarPid()
        return self.pidSiguiente

    #Esto devuelve el pcb corriendo en la cpu 
    @property
    def getRunningPCB(self):
        return self.runningPCB
    
    @getRunningPCB.setter
    def setRunningPCB(self, pcb):
        self.runningPCB = pcb

    def __repr__(self):
        return f"{self.table} --tabla \n {self.pidSiguiente}"

    #Esto devuelve si hay algun pcb corriendo en la cpu 
    @property
    def getIsRunning(self):
        return self.isRunning

    @getIsRunning.setter
    def setIsRunning(self, nuevoBool):
        self.isRunning = nuevoBool


class Loader:
    def __init__ (self, memory):
        self.memory = memory
        self.baseDir = 0

    def write(self, program):
        # loads the program in main memory
        baseDirInicial = self.baseDir
        for instruction in program.instructions:
            self.memory.write(self.baseDir, instruction)
            self.baseDir += 1
        return baseDirInicial

    @property
    def getBaseDir(self):
        return self.baseDir


class Dispatcher:                   #LISTO  
    def __init__(self, mmu, cpu): 
        self.mmu = mmu
        self.cpu = cpu
        
    def load(self, pcb):
        self.cpu.pc = pcb.pc
        self.mmu.baseDir = pcb.baseDir

    def save(self, pcb):
        pcb.setPc(self.cpu.pc)
        self.cpu.pc = -1
