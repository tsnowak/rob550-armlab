import time

"""
This is an alarm o clock
"""


ALARM_STATUS_CODE_UNSETTED  = 0
ALARM_STATUS_CODE_SETTEDANDRUNNING = 1
ALARM_STATUS_CODE_SETTEDANDREACHED = 2


class AlarmClock():

	def __init__(self):
		self.currentTime = 0
		self.startTime = 0
		self.stayTime = 0
		self.timesup = False;
		self.status = ALARM_STATUS_CODE_UNSETTED


	def alarmclock_start(self, settedtime):
		self.alarmclock_stop()
		self.startTime  = int(round(time.time() * 1000));
		self.timesup = False
		self.stayTime = settedtime
		self.status = ALARM_STATUS_CODE_SETTEDANDRUNNING
		

	def alarmclock_checktimesup(self):
		self.currentTime  = int(round(time.time() * 1000));
		if (self.currentTime - self.startTime >= self.stayTime):
			self.status = ALARM_STATUS_CODE_SETTEDANDREACHED
			return True
		else:
			self.status = ALARM_STATUS_CODE_SETTEDANDRUNNING
			return False

	def alarmclock_stop(self):
		self.currentTime = 0
		self.startTime = 0
		self.stayTime = 0
		self.timesup = 0
		self.status = ALARM_STATUS_CODE_UNSETTED


	def alarmclock_checkstatus(self):
		return self.status