# Load module function in Python is changed
# to look for a libmacro.so in LD_LIBRARY_PATH
import sst
import sst.macro

smallLatency = "1ps"


def makeUniLink(linkType,srcComp,srcId,srcPort,dstComp,dstId,dstPort,outLat=None,inLat=None):
  if not outLat: outLat = inLat
  if not inLat: inLat = outLat
  if not outLat: sys.exit("must specify at least one latency for link")

  linkName = "%s%d:%d->%d:%d" % (linkType,srcId,srcPort,dstId,dstPort)
  link = sst.Link(linkName)
  portName = "output %d %d" % (srcPort, dstPort)
  srcComp.addLink(link,portName,outLat)
  portName = "input %d %d" % (srcPort, dstPort)
  dstComp.addLink(link,portName,inLat)

def makeBiLink(linkType,comp1,id1,port1,comp2,id2,port2,outLat=None,inLat=None):
  makeUniLink(linkType,comp1,id1,port1,comp2,id2,port2,outLat,inLat)
  makeUniLink(linkType,comp2,id2,port2,comp1,id1,port1,outLat,inLat)

def makeUniNetworkLink(srcComp,srcId,srcPort,dstComp,dstId,dstPort,outLat=None,inLat=None):
  makeUniLink("network",srcComp,srcId,srcPort,dstComp,dstId,dstPort,outLat,inLat)

def makeBiNetworkLink(comp1,id1,port1,comp2,id2,port2,outLat=None,inLat=None):
  makeBiLink("network",comp1,id1,port1,comp2,id2,port2,outLat,inLat)
  

def addNew(prefix, kw, newDict, oldDict):
  name = "%s.%s" % (prefix, kw)
  newDict[name] = oldDict[kw]

def addParams(prefix, dict, *xargs, **kwargs):
  for entry in xargs:
    if isinstance(entry, type({})):
      for kw in entry:
        addNew(prefix, kw, dict, entry)
    else: #function
      entry(prefix, dict)
    
  for kw in kwargs:
    addNew(prefix, kw, dict, kwargs)

def addSubParams(oldPrefix, newPrefix, dict, *xargs, **kwargs):
  prefix = "%s.%s" % (oldPrefix, newPrefix)
  addParams(prefix, dict, *xargs, **kwargs)

def subParams(prefix, *xargs, **kwargs):
  return lambda x,y: addSubParams(x, prefix, y, *xargs, **kwargs)

def redoSubParams_impl(nsArr, theDict, allParams):
  for key in theDict:
    val = theDict[key]
    if isinstance(val, dict):
      newNsArr = nsArr[:]
      newNsArr.append(key)
      redoSubParams_impl(newNsArr, val, allParams)
    else:
      paramArr = nsArr[:]
      paramArr.append(key)
      newParam = ".".join(paramArr)
      allParams.append((newParam, val))

def macroToCoreParams(theDict):
  allParams = []
  redoSubParams_impl([], theDict, allParams)
  newDict = {}
  for key, val in allParams:
    newDict[key] = val
  return newDict

class Interconnect:
  def __init__(self, params):
    self.params = params
    self.system = sst.macro.System(params)
    self.num_nodes = self.system.numNodes()
    self.num_switches = self.system.numSwitches()
    self.switches = [0]*self.num_switches
    self.nodes = [0]*self.num_nodes

  def numNodes(self):
    return self.num_nodes

  def numSwitches(self): 
    return self.num_switches

  def defaultEpFxn(self, nodeID):
    nodeParams = self.params["node"]
    compName = nodeParams["name"].capitalize() + "Node"
    node = sst.Component("Node %d" % nodeID, "macro.%s" % compName)
    node.addParams(macroToCoreParams(nodeParams))
    node.addParam("id", nodeID)
    return node

  def buildSwitches(self):
    for i in range(self.num_switches):
      switchParams = self.params["switch"]
      compName = switchParams["name"].capitalize() + "Switch"
      switch = sst.Component("Switch %d" % i, "macro.%s" % compName)
      switch.addParams(macroToCoreParams(switchParams))
      switch.addParam("id", i)
      self.switches[i] = (switch, switchParams)

  def buildEndpoints(self, epFxn):
    for i in range(self.num_nodes):
      self.nodes[i] = epFxn(i)

  def latency(self, params):
    if params.has_key("latency"):
      return params["latency"]
    else:
      import sys
      sys.exit("need link latency in parameters")

  def latencyAsFloat(self, params):
    import re
    lat = self.latency(params)
    match = re.compile("(\d+[.]?\d*)(.*)").search(lat)
    if not match:
      sys.exit("improperly formatted latency %s" % lat)
    num, units = match.groups()
    num = float(num)
    units = units.strip().lower()
    if units == "ms":
      num *= 1e-3
    elif units == "us":
      num *= 1e-6
    elif units == "ns":
      num *= 1e-9
    elif units == "ps":
      num *= 1e-12
    return num

  def connectSwitches(self):
    switchParams = self.params["switch"]
    for i in range(self.num_switches):
      linkParams = switchParams["link"]
      connections = self.system.switchConnections(i)
      srcSwitch, params = self.switches[i]
      lat = self.latency(linkParams)
      for srcId, dstId, srcOutport, dstInport in connections:
        dstSwitch, dstParams = self.switches[dstId]
        makeUniNetworkLink(srcSwitch,srcId,srcOutport,
                           dstSwitch,dstId,dstInport,
                           lat)

  def connectEndpoints(self):
    lat = ""
    if self.params.has_key("injection_latency"):
      lat = self.params["injection_latency"]
    else:
      lat = self.params["node"]["nic"]["injection"]["latency"]

    for swId in range(self.num_switches):
      connections = self.system.injectionConnections(swId)
      for epId, switchPort, injPort in connections:
        ep = self.nodes[epId]
        injSwitchComp, params = self.switches[swId]
        makeUniLink("injection",ep,epId,injPort,injSwitchComp,swId,switchPort,lat)

      connections = self.system.ejectionConnections(swId)
      for epId, switchPort, ejPort, in connections:
        ep = self.nodes[epId]
        ejSwitchComp, params = self.switches[swId]
        makeUniLink("ejection",ejSwitchComp,swId,switchPort,ep,epId,ejPort,
                    outLat=lat,inLat=smallLatency)

  def buildLogPNetwork(self):
    import re
    nproc = sst.getMPIRankCount() * sst.getThreadCount()
    switchParams = self.params["switch"]
    if switchParams.has_key("logp"):
      switchParams = switchParams["logp"]
    lat = switchParams["out_in_latency"]
    switches = []
    for i in range(nproc):
      switch = sst.Component("LogP %d" % i, "macro.LogPSwitch")
      switch.addParams(macroToCoreParams(switchParams))
      switch.addParam("id", i)
      switches.append(switch)

    for i in range(self.num_nodes):
      injSW = self.system.nodeToLogPSwitch(i)
      ep = self.nodes[i]
      sw = switches[injSW]
      linkName = "logPinjection%d->%d" % (i, injSW)
      link = sst.Link(linkName)
      portName = "output %d %d" % (sst.macro.NICLogPInjectionPort, i)
      ep.addLink(link, portName, smallLatency) #put no latency here
      portName = "input %d %d" % (sst.macro.NICLogPInjectionPort, i)
      sw.addLink(link, portName, smallLatency)

    for i in range(self.num_nodes):
      ep = self.nodes[i]
      for p in range(nproc):
        linkName = "logPejection%d->%d" % (i, injSW)
        link = sst.Link(linkName)
        sw = switches[p]
        portName = "output %d %d" % (i, sst.macro.NICLogPInjectionPort)
        sw.addLink(link, portName, lat)
        portName = "input %d %d" % (i, sst.macro.NICLogPInjectionPort)
        ep.addLink(link, portName, lat)

  def buildFull(self, epFxn):
    self.buildSwitches()
    self.buildEndpoints(epFxn)
    self.connectSwitches()
    self.connectEndpoints()
    self.buildLogPNetwork()

  def buildLogP(self, epFxn):
    self.buildEndpoints(epFxn)
    self.buildLogPNetwork()
  
  def build(self, epFxn=None):
    if epFxn == None:
      epFxn = self.defaultEpFxn
    if self.system.isLogP():
      self.buildLogP(epFxn)
    else:
      self.buildFull(epFxn)

def readCmdLineParams():
  import sys
  return sst.macro.readParams(sys.argv)

def setupDeprecatedParams(params, debugList=[]):
  nodeParams = params["node"]
  swParams = params["switch"]

  builtinApps = [
    "apitest",
    "global_test",
    "hello_world",
    "mpi_coverage",
    "mpi_ping_all",
    "mpi_print_nodes",
    "mpi_topology",
    "parsedumpi",
    "sstmac_mpi_testall",
    "traffic_matrix",
    "UserAppCxxEmptyMain",
    "UserAppCxxFullMain",
  ]

  for i in range(10):
    ns = "app%d" % i
    if params.has_key(ns):
      appParams = params[ns]
      nodeParams[ns] = appParams
      appName = appParams["name"]
      if not appName in builtinApps:
        cmd = "import sst.%s" % appName
        exec(cmd)
      del params[ns]

  icParams = {}
  icParams["topology"] = params["topology"]
  nodeParams["interconnect"] = icParams
  if debugList:
    nodeParams["debug"] = "[" + ",".join(debugList) + "]"
  swParams["topology"] = params["topology"]

  #move every param in the global namespace 
  #into the individal namespaces
  for ns in "node", "switch":
    nsParams = params[ns]
    for key in params:
      val = params[key]
      if isinstance(val, str):
        if not nsParams.has_key(key):
          nsParams[key] = val

  ic = Interconnect(params)
  ic.build()
  return ic

def setupDeprecated():
  import sys
  sst.setProgramOption("timebase", "100as")
  params = readCmdLineParams()
  debugList = []
  if params.has_key("debug"):
    debugList = params["debug"].strip().split()
  for i in range(len(sys.argv)):
    if sys.argv[i] == "-d" or sys.argv[i] == "--debug":
      debugList.extend(sys.argv[i+1].split(","))
  return setupDeprecatedParams(params, debugList)
