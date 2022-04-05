# High Level Analyzer
# For more information and documentation, please go to https://support.saleae.com/extensions/high-level-analyzer-extensions

from saleae.analyzers import HighLevelAnalyzer, AnalyzerFrame, StringSetting, NumberSetting, ChoicesSetting


class FRCAddress():
	broadcastTypeIndexMap = {
		0: "Disable",
		1: "Halt",
		2: "Reset",
		3: "Assign",
		4: "Query",
		5: "Heartbeat",
		6: "Sync",
		7: "Update",
		8: "Version",
		9: "Enumerate",
		10: "Resume",
	}
	
	deviceTypeMap = {
		0: "Broadcast", 
		1: "Robot Controller", 
		2: "Motor Controller", 
		3: "Relay Controller",
		4: "Gyro Sensor", 
		5: "Accelerometer", 
		6: "Ultrasonic Sensor", 
		7: "Gear Tooth Sensor", 
		8: "PDP/PDM", 
		9: "Pneumatics Controller",
		10: "Misc", 
		11: "IO Breakout",
		31: "Firmware Update",
	}
	
	manufacturerCodeMap = {
		0: "Broadcast",
		1: "NI", 
		2: "Luminary Micro", 
		3: "DEKA", 
		4: "CTR Electronics", 
		5: "REV Robotics", 
		6: "Grapple", 
		7: "MindSensors", 
		8: "Team Use", 
		9: "Kauai Labs", 
		10: "Copperforge", 
		11: "Playing with Fusion",
		12: "Sutdica",
	}
	
	# Includes an empty device ID field, but kept this way to make the numbers match
	# what shows up in CTRE headers.
	ctreMotorStatusMap = {
		0x1400: "Status 1 (General)",
		0x1440: "Status 2 (Feedback PID0)",
		0x14C0: "Status 4 (Analog sensor, temp, voltage)",
		0x1540: "Status 6 (Misc)",
		0x1580: "Status 7 (Comms)",
		0x1600: "Status 9 (Motion profile buffer)",
		0x1640: "Status 10 (Targets/MotionMagic)",
		0x16C0: "Status 12 (Feedback PID1)",
		0x1700: "Status 13 (Primary PIDF0)",
		0x1740: "Status 14 (Aux PIDF1)",
		0x1780: "Status 15 (Firmware/API status)",
		0x1C00: "Status 17 (Targets PID1)",
	}
	
	ctreMotorControlMap = {
		0x040040: "Control2 (Enable 50m)",
		0x040080: "Control3 (General)",
		0x0400C0: "Control4 (Advanced)",
		0x040100: "Control5 (Feedback override)",
		0x040140: "Control6 (Add trajectory point)",
	}
	
	ctrePDPStatusMap = {
		0x50: "Status1",
		0x51: "Status2",
		0x52: "Status3",
		0x5D: "StatusEnergy",
	}
	
	ctrePDPControlMap = {
		0x70: "Control1",
	}


	def __init__(self, address):
		self.address = address

		'''
		Decode an FRC CAN address:
		
		| DeviceType   | Mfg Code			   | API Class		 | Idx	 | Device #	 |
		 28 27 26 25 24 23 22 21 20 19 18 17 16 15 14 13 12 11 10 9 8 7 6 5 4 3 2 1 0
		'''
		self.deviceType = (address >> 24) & 0x1F
		self.manufacturerCode = (address >> 16) & 0xFF
		self.apiClass = (address >> 10) & 0x3F
		self.apiIndex = (address >> 6) & 0xF
		self.deviceNumber = (address >> 0) & 0x3F
		
	def isBroadcast(self):
		return (self.deviceType == 0 and self.manufacturerCode == 0 and self.apiClass == 0) or self.deviceNumber == 63
		
	def isDeviceBroadcast(self):
		return self.deviceNumber == 63
		
	def isHeartbeat(self):
		return self.address == 0x01011840

	def broadcastTypeString(self):
		if self.isDeviceBroadcast():
			return "Device-specific"
		else:
			return FRCAddress.broadcastTypeIndexMap.get(self.apiIndex, "UNKNOWN")
			
	def deviceTypeString(self):
		if self.deviceType >= 12 and self.deviceType <= 30:
			return "Reserved"
		else:
			return FRCAddress.deviceTypeMap.get(self.deviceType, "UNKNOWN")
			
	def manufacturerString(self):
		if self.manufacturerCode >= 13 and self.manufacturerCode <= 255:
			return "Reserved"
		else:
			return FRCAddress.manufacturerCodeMap.get(self.manufacturerCode, "UNKNOWN")
			
	def statusTypeString(self):
		if self.deviceType == 2 and self.manufacturerCode == 4:
			# CTRE motor controller
			# Preserve just the API class and index, in place
			ctreStatus = self.address & (0x3FF << 6)
			return FRCAddress.ctreMotorStatusMap.get(ctreStatus, None)
		elif self.deviceType == 8 and self.manufacturerCode == 4:
			value = self.apiClass << 4 | self.apiIndex
			return FRCAddress.ctrePDPStatusMap.get(value, None)
		else:
			return None
			
	def controlTypeString(self):
		if self.deviceType == 2 and self.manufacturerCode == 4:
			# CTRE motor controller
			# These constants have the manufacturer already in them
			ctreControl = self.address & (0x1FFFF << 6)
			return FRCAddress.ctreMotorControlMap.get(ctreControl, None)
		elif self.deviceType == 8 and self.manufacturerCode == 4:
			value = self.apiClass << 4 | self.apiIndex
			return FRCAddress.ctrePDPControlMap.get(value, None)
		else:
			# Debugging
			ctreControl = self.address & (0x3FF << 6)
			if FRCAddress.ctreMotorControlMap.get(ctreControl, None) is not None:
				print('Possible unrecognized control frame from {}:{}'.format(self.deviceType, self.manufacturerCode))

			return None
			
			
class FRCFrame:
	heartbeat4Bitmap = {
		'red_alliance': 0x80,
		'enabled': 0x40,
		'autonomous': 0x20,
		'test': 0x10,
		'watchdog_enabled': 0x08,
	}

	def __init__(self, address, isExtended, isRemote, startTime):
		self.rawAddress = address
		self.frcAddress = FRCAddress(address)
		self.frameStart  = startTime
		self.frameEnd = None
		self.isRemote = isRemote
		self.isExtended = isExtended
		self.data = []
		self.dataString = ""
		self.expectedLength = 0
		self.ack = False
		
	def isFRCFrame(self):
		# All FRC frames use extended 29-bit addressing
		return self.isExtended
		
	def addData(self, datum):
		self.data.append(int.from_bytes(datum, "big"))
		self.dataString += '{0:02X} '.format(int.from_bytes(datum, "big"))
		
	def heartbeatDict(self):
		info = {}
		if len(self.data) >= 5:
			info = {}
			importantByte = self.data[4]
			for key in FRCFrame.heartbeat4Bitmap:
				if importantByte & FRCFrame.heartbeat4Bitmap[key]:
					info[key] = True
				else:
					info[key] = False
		return info
		
	def frameInfoDict(self):
		info = {
			'identifier': self.frcAddress.address,
			'device_type': self.frcAddress.deviceType,
			'device_type_name': self.frcAddress.deviceTypeString(),
			'mfg': self.frcAddress.manufacturerCode,
			'mfg_name' : self.frcAddress.manufacturerString(),
			'api': self.frcAddress.apiClass,
			'idx': self.frcAddress.apiIndex,
			'device_id': self.frcAddress.deviceNumber,
			'data': self.dataString,
			'expected_length' : self.expectedLength,
			'length': len(self.data),
		}
		
		if self.isKnownStatusFrame():
			info['status_type'] = self.frcAddress.statusTypeString()
			
		if self.isKnownControlFrame():
			info['control_type'] = self.frcAddress.controlTypeString()
			
		if self.frcAddress.isBroadcast():
			info['broadcast_type'] = self.frcAddress.broadcastTypeString()
			
		if self.frcAddress.isHeartbeat():
			info.update(self.heartbeatDict())
	
		return info

	def isKnownStatusFrame(self):
		return self.frcAddress.statusTypeString() is not None
		
	def isKnownControlFrame(self):
		return self.frcAddress.controlTypeString() is not None
				
	def analyzerFrame(self):
		frameType = "Normal"
		
		if self.frcAddress.isHeartbeat():
			frameType = 'Heartbeat'
		elif self.frcAddress.isBroadcast():
			frameType = 'Broadcast'
		elif self.isKnownStatusFrame():
			frameType = 'Status'
		elif self.isKnownControlFrame():
			frameType = 'Control'
			
		# print(frameProperties)
			
		return AnalyzerFrame(frameType, self.frameStart, self.frameEnd, self.frameInfoDict())

			


# High level analyzers must subclass the HighLevelAnalyzer class.
class Hla(HighLevelAnalyzer):
	# List of settings that a user can set for this High Level Analyzer.

	# An optional list of types this analyzer produces, providing a way to customize the way frames are displayed in Logic 2.
	result_types = {
		'Normal': {
			'format': 'FRAME Dev: {{data.device_type_name}} Mfg: {{data.mfg_name}} API: {{data.api}} Index: {{data.idx}} DevID: {{data.device_id}} Data: <{{data.data}}>'
		},
		'Broadcast': {
			'format': 'BROADCAST Type:{{data.broadcast_type}} Data: <{{data.data}}>'
		},
		'Heartbeat': {
			'format': 'HEARTBEAT RedAlliance: {{data.red_alliance}} Enabled: {{data.enabled}} Autonomous: {{data.autonomous}} Test: {{data.test}} WatchdogEnabled: {{data.watchdog_enabled}} Data: <{{data.data}}>'
		},
		'Control': {
			'format': 'CONTROL Dev: {{data.device_type_name}} Mfg: {{data.mfg_name}} ControlType: {{data.control_type}} DevID: {{data.device_id}} Data: <{{data.data}}>'
		},
		'Status': {
			'format': 'STATUS Dev: {{data.device_type_name}} Mfg: {{data.mfg_name}} StatusType: {{data.status_type}} DevID: {{data.device_id}} Data: <{{data.data}}>'
		},
		'Error': {
			'format': 'ERROR'
		}
	}

	def __init__(self):
		'''
		Initialize HLA.

		Settings can be accessed using the same name used above.
		'''
			  
	def decode(self, frame: AnalyzerFrame):
		'''
		Process a frame from the input analyzer, and optionally return a single `AnalyzerFrame` or a list of `AnalyzerFrame`s.

		The type and data values in `frame` will depend on the input analyzer.
		'''
		
		if frame.type == "identifier_field":
			self.frcFrame = FRCFrame(address = frame.data["identifier"], 
				isExtended = frame.data.get("extended", False),
				isRemote = frame.data.get("RemoteFrame", False),
				startTime = frame.start_time)
		elif frame.type == "control_field":
			self.frcFrame.expectedLength = frame.data["num_data_bytes"]
		elif frame.type == "data_field":
			self.frcFrame.addData(frame.data["data"])
		elif frame.type == "can_error":
			self.frcFrame = None
			return AnalyzerFrame('Error', frame.start_time, frame.end_time)
		elif frame.type == "ack_field":
			self.frcFrame.frameEnd = frame.end_time
			self.frcFrame.ack = frame.data.get("ack", False)
			
			return self.frcFrame.analyzerFrame()

		return None
