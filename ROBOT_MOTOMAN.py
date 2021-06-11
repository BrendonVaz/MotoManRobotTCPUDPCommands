import os
import sys
import time
import socket
import threading
import math
import struct

class rob():

	def __init__(self, PARENT=0, dbg = 0):
	
		self.PAR 		= PARENT
		self.dbg        = dbg
		self.com1   	= 'CONNECT Robot_access\r'			#host control request
		self.com2   	= 'HOSTCTRL_REQUEST '				#command header
		self.IP_ADD 	= '192.168.1.31'					#robot IP
		self.TCP_PT 	= 80								#robot tcp port number
		self.UDP_PT 	= 10040								#robot udp port number
		self.rob_chkout = False								#socket lock flag to make sure only one message at one time
		self.sock_udp   = socket.socket(socket.AF_INET, socket.SOCK_DGRAM);self.sock_udp.settimeout(1.0)
		self.sock_tcp   = socket.socket(socket.AF_INET, socket.SOCK_STREAM);
		return


#~ -----------------------------------------------------------------------------------------------------------------------------------------
#~ TCP COMMANDS
#~ -----------------------------------------------------------------------------------------------------------------------------------------


	def runchk(self):																		#check if robot online

		if not (not os.system('ping -c 1 192.168.1.31') or not os.system('ping 192.168.1.31 -n 1')): print ("ERROR! Robot Server Off Line!");sys.exit()

		self.wrgpio() #write all gpio 0
		
		stt = self.redstt();
		saf = self.redsaf(); 
		col = self.colsaf(); col = col[0] or col[1]
		
		if saf[4] != 0:		print("ERROR! Robot Battery Low!"); 			sys.exit()
		if int(stt[0])!=1:	print("ERROR! Robot Not in Command Mode"); 		sys.exit()
		if sum(saf[0:3])!=3:print("ERROR! E Stop Triggered!");				sys.exit()
		if col:				print("ERROR! Collaborative Mode Triggered!");	sys.exit()

		print "-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------"
		print "ROBOT CHECK"
		print "-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------"
		print "Robot Server Online..."
		print "Robot Mode Check Complete..."
		print "Robot Safety Check Complete..."
		print "-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------"
		
		return

	def senreq(self):																		#host control request
		
		try:self.sock_tcp.connect((self.IP_ADD,self.TCP_PT))
		except: print ("Error! Cannot Connect Socket to Robot"); sys.exit();
			
		self.sock.send(self.com1)
		resp = self.sock.recv(256)
		
		if self.dbg: 
			print ("Sent: ", self.com1.strip()); 
			print ("Recd: ", resp.strip())
		
		return resp

	def sencom(self, comm, data, movecom = False, posf = None, resend = False):				#send command
		
		commm = comm																		#incase move wait recovery
		dataa = data																		#incase move wait recovery
		
		size = len(data)																	#if data get data size
		comm = self.com2 + comm + ' ' + str(size) + '\r'									#parse command n data
		senrq = self.senreq()																#send host control request

		while self.rob_chkout:pass															#if robot busy, wait
		
		self.rob_chkout = True;																#set robot busy
		
		self.sock_tcp.send(comm);resp  = self.sock.recv(256)								#read 256 byte comm resp
		self.sock_tcp.send(data);resp += self.sock.recv(256)								#read 256 byte data resp

		if "closing control connection" in resp:											#if robot closes port
			print("Robot Forcefully Disconnected")											#if error resp exit 
			sys.exit()

		self.rob_chkout = False																#set robot not busy
		
		if self.dbg: 
			print ("Sent: ", comm); 
			print ("Data: ", data); 
			print ("Recd: ", resp.split('\r\n')[0]+":", resp.split('\r\n')[1].strip(), "\n")

		if movecom == True: self.mvwait(commm, dataa, posf);												#loop while robot moving
			
		return resp

	def mvwait(self, comm, data, pos ,check_estop=0, check_safety_gate=0, check_collab=0):	#wait for motion command to complete

		dim = 100; saf = 4; run = 1; srv = 0; tog = 0; col = 1; ylo = False					#target;safety;runing;servof;toggle;safety gate light

		while dim > 25 or run == 1 or srv == 0 or saf != 3:									#while command not complete

			if 1:			 																#debug print
				print ("-------------------------------------------------------------")
				print ("WAITING FOR...",   comm) 
				print ("-------------------------------------------------------------")
				print ("TARGET REACHED :", dim)
				print ("RUNNING BIT ON :", run)
				print ("SERVO   BIT ON :", srv)
				print ("SAFETY  BIT SUM:", saf)
				print ("COLLABORATIVE  :", col)
				print ("-------------------------------------------------------------")

			if 1:																			#read and calculate data
				#read safety, status, position				
				saf=self.redsaf();
				stt=self.redstt();
				pt1=self.redpos();
				col=self.colsaf();
				msg = ""; 
				
				mod = int(stt[0]); gat = int(saf[3]); saf = sum(saf[0:3])													#pase mode, area scan, estop
				srv = int(stt[9]); run = int(stt[4]); slo = int(stt[3])														#parse servo, run, safegate bit
				
				col = col[0] or col[1]; pt1 = map(float, pt1.split('\n')[1].split(',')[0:6])								#parse colaborative safety trigger, position
				
				if not pos == None:																							#if check target flag is on
					dim = [pt1[0]-pos[0], pt1[1]-pos[1], pt1[2]-pos[2], pt1[3]-pos[3], pt1[4]-pos[4], pt1[5]-pos[5]]		#check if robot reached target
					dim = (dim[0]**2 + dim[1]**2 + dim[2]**2)**0.5															#calculate delta position norm
				else: dim = 0	
				
				if not check_estop: 		srv = 1;
				if not check_safety_gate:	gat = 3;
				if not check_collab:		col = 0;
				
			if 1:																			#print warnings & prompts

				if mod!=1: 	print ("Error! Robot Not in Command Mode");sys.exit()						#if not in remote mode, exit code
				if col:		print ("Error! Collaborative Safety Triggered!"); self.servof() 			#if collaborative trigger, warning, servo off

				if not srv:																				#if servo off   = trigger
					
					if 1:			print ("Error! Servo Off.")								#send message servo off
					if col:			print ("Error! Collaborative Safety Triggered")			#send message reset collaborative safety trigger

					if saf != 3:	print ("Error! E Stop Triggered.")						#send message estop trigger
					
					elif saf == 3 and not col:												#if no safety trigger, recover

						print ("Safety Clear. Restoring Servo Power.")						#read alarm,reset alarm, restore servo
						self.redalm();
						self.resets();
						self.servon();														
						print ("Resuming Motion, Please Stay Back")
						self.sencom(comm,data,movecom = True, posf = pos, resend = True)	#resend last motion command

				if not gat and srv: 		print("Safety Gate Triggered");ylo = 1;			#display message safety gate triggered
				elif gat and srv and ylo:	print ("Safety Gate Clear.");  ylo = 0;			#display message safety gate clear 

		return 1


	def redpos(self):																		#read cartesian position of robot
		comm = 'RPOSC'
		data = '0,0\r'
		return self.sencom(comm,data)

	def redpls(self):																		#read pulse position of robot
		comm = 'RPOSJ'
		data = ''
		return self.sencom(comm,data)

	def redalm(self):																		#read alarms
		comm = 'RALARM'
		data = ''
		return self.sencom(comm,data)

	def redstt(self):																		#read status bits
		comm = 'RSTATS'
		data = ''
		stt = self.sencom(comm,data).split('\n')[1].split(',')
		st1 = int(stt[0])
		st2 = int(stt[1])
		stt = '{0:08b}'.format(st1) + '{0:08b}'.format(st2)
		return stt

	def redsaf(self):																		#read safety bytes

		comm = 'IOREAD'
		
		data = '80020,8\r';stop = self.sencom(comm,data)
		data = '80400,8\r';safe = self.sencom(comm,data)
		data = '50010,8\r';batt = self.sencom(comm,data)

		stop = format(int(stop.split('\n')[1].strip()),'08b')
		safe = format(int(safe.split('\n')[1].strip()),'08b')
		batt = format(int(batt.split('\n')[1].strip()),'08b')

		if batt[5] == '1' or batt[6] == '1': print "Battery Response:\t", batt

		batt = int(batt[5]) or int(batt[6])
		pstp = int(stop[1])
		estp = int(stop[2])
		astp = int(stop[4])
		asaf = int(safe[7])
		
		return [pstp, estp, astp, asaf, 0]

	def colsaf(self):																		#check collaborative hard/soft bump
		comm = 'IOREAD'
		data = '81382,1\r'
		hard = self.sencom(comm,data)
		data = '81383,1\r'
		soft = self.sencom(comm,data)
		hard = format(int(hard.split('\n')[1].strip()),'08b')[5]
		soft = format(int(soft.split('\n')[1].strip()),'08b')[5]
		return [int(hard), int(soft)]


	def resets(self):																		#reset alarms
		comm = 'RESET'
		data = ''
		return self.sencom(comm,data)

	def cancel(self):																		#cancel request... 	useless never used
		comm = 'CANCEL'
		data = ''
		return self.sencom(comm,data)

	def holdon(self):																		#external hold... 	useless never used
		comm = 'HOLD'
		data = '1\r'
		return self.sencom(comm,data)

	def holdof(self):																		#hold off...		useless never used
		comm = 'HOLD'
		data = '0\r'
		return self.sencom(comm,data)

	def setmod(self, m):																	#useless... cannot switch to command mode without key anyway, hardware safety
		if m == 1:data = '1\r'
		if m == 2:data = '2\r'
		comm = 'MODE'
		return self.sencom(comm,data)

	def servon(self):																		#servo on
		comm = 'SVON'
		data = '1\r'
		return self.sencom(comm,data)

	def servof(self):																		#servo off
		comm = 'SVON'
		data = '0\r'
		return self.sencom(comm,data)

	def msgdis(self, msg):																	#display pendant message
		comm = 'MDSP'
		data = msg + '\r'
		return self.sencom(comm,data)


	def rdgpio(self, stt_add=30050, byt_num=1, p=1):										#read byt_num of gpio starting at stt_add
		if not (isinstance(byt_num,int) and byt_num >0): return
		byt_num = byt_num*8
		comm = 'IOREAD'
		data = str(stt_add)+','+str(byt_num)+'\r'
		return self.sencom(comm,data)
		
	def wrgpio(self, stt_add=27010, bit_num=8, bit_val=[[0,0,0,0,0,0,0,0]], p=1):			#write bit_nums starting from stt_add

		flag = 0
		comm = 'IOWRITE'
		data = str(stt_add) + "," + str(bit_num)

		if 1: 			#check input 
			if not isinstance(bit_val,list): flag = 1;print "Error", 1
			elif len(bit_val) != bit_num/8:  flag = 1;print "Error", 2
			elif bit_num % 8 != 0:			 flag = 1;print "Error", 3
			else:
				for byte in bit_val:
					if flag: break
					if len(byte) != 8:
						flag = 1;print "Error", 4
						break
					for bit in byte:
						if bit != 0 and bit != 1:
							flag = 1;print "Error", 5
							break

			if flag: return "INPUT ERROR"

		if 1:			#parse data
			bytedata = []

			for bitlist in bit_val:
				out = 0
				for bit in bitlist:
					out = (out<<1) | bit
				bytedata.append(out)

			for byte_val in bytedata: data = data + ',' + str(byte_val)
			data = data + '\r'
		
		return self.sencom(comm,data)
		
	def runjob(self,n='HOME',o=30050):														#run job name n, and read complete flag o

		"""
		NOTES:
		-> this function will run a job n on robot controller and wait for an output flag to be set if 0 != 0
		-> the function will wait a minimum of one second until the function is complete
		-> n = string name of job
		-> o = job complete flag output bit (Need to set on pendant)
		"""		
		
		comm = 'START';data = n+'\r';a  = 1
		print self.sencom(comm,data);time.sleep(1)
		while a: a = int(format(int(self.fxn.rob.rdgpio(o).split('\n')[1].strip()),'08b')[4]);
		return a

	def gohome(self):																		#move robot home position pulse = 0
		comm = 'PMOVJ'
		data = '5,0,0,0,0,0,0,0,0,0,0,0,0,0\r'
		return self.sencom(comm,data, movecom = True)


	def movjnt(self, v, px, py, pz, rx, ry, rz, tp=6):										#move joint to absolute position
		
		"""
			v  = velocity (in % Speed)
			px = position x
			py = position y
			pz = position z
			rx = rotation x
			ry = rotation y
			rz = rotation z
			tp = orientation type -> please see documentation (default to type 6)		
			frame is defaulted to "0" which is world frame
		"""
		
		comm = 'MOVJ'
		data = str(v) + ',0,' + str(px) + ',' + str(py) + ',' + str(pz) + ',' + str(rx) + ',' + str(ry) + ',' + str(rz) + ',' + str(tp) + ',0,0,0,0,0,0,0\r'
		fpos = [px,py,pz,rx,ry,rz] #final position, used to confirm motion complete using read position
		return self.sencom(comm,data, movecom = True, posf = fpos)

	def movlin(self, v, px, py, pz, rx, ry, rz, tp=6):										#linear move to absolute position
		
		"""
			v  = velocity (in mm/s)
			px = position x
			py = position y
			pz = position z
			rx = rotation x
			ry = rotation y
			rz = rotation z
			tp = orientation type -> please see documentation (default to type 6)		
			frame is defaulted to "0" which is world frame
		"""
		
		comm = 'MOVL'
		data = '0, ' + str(v) + ',0,' + str(px) + ',' + str(py) + ',' + str(pz) + ',' + str(rx) + ',' + str(ry) + ',' + str(rz) + ',' + str(tp) + ',0,0,0,0,0,0,0\r'
		fpos = [px,py,pz,rx,ry,rz] #final position, used to confirm motion complete using read position
		return self.sencom(comm,data, movecom = True, posf = fpos)

	def movinc(self,v,dx,dy,dz,da,db,dc, rv=0, lv=0):										#incremental move

		""" Use increment move command with increment data
			v  = velocity, see lv/rv flag
			dx = incremental position x
			dy = incremental position y
			dz = incremental position z
			da = incremental rotation x
			db = incremental rotation y
			dc = incremental rotation z			
			rv = force speed rotational 
			lv = force speed linear
		"""

		comm = 'IMOV'

		if dx+dy+dz == 0:	data = '1,';v = min(v, 100);									#if no linear distance,  use rotate speed		
		else:				data = '0,';v = min(v, 500);									#else 					 use linear speed
		if rv:				data = '1,';v = min(rv,100);									#if optional rv provided use linear speed
		if lv:				data = '0,';v = min(lv,500);									#if optional lv provided use rotate speed

		data = data + str(v) + ',' + '0' + ',' + str(dx) + ',' + str(dy) + ',' + str(dz) + ',' + str(da) + ',' + str(db) + ',' + str(dc) + ',0,0,0,0,0,0,0,0\r'
		posi = [float(i) for i in self.redpos().split('\n')[1].split(',')[0:6]] 			#get initial position of robot
		posm = [float(i) for i in [dx, dy, dz, da, db, dc]]									#calculate final position of robot
		fpos = map(sum,zip(posi,posm))
		return self.sencom(comm,data, movecom = True, posf = fpos)

	def movijt(self,v,dx,dy,dz,da,db,dc,p=1):												#joint incremental move with current position read

		""" Use joint move command with increment data
			v  = velocity, see lv/rv flag
			dx = incremental position x
			dy = incremental position y
			dz = incremental position z
			da = incremental rotation x
			db = incremental rotation y
			dc = incremental rotation z		
		"""

		posr = self.redpos().split('\n')[1].split(',');										#read current position...
		posi = [float(i) for i in posr[0:6]]												#get position & rotation
		posm = [float(i) for i in [dx, dy, dz, da, db, dc]];								#parse input vector
		fpos = map(sum,zip(posi,posm))														#add input vector to current positon...	
		comm = 'MOVJ'
		data = str(v)+',0,'+str(fpos[0])+','+str(fpos[1])+','+str(fpos[2])+','+str(fpos[3])+','+str(fpos[4])+','+str(fpos[5])+','+posr[6]+',0,0,0,0,0,0,0\r'
		return self.sencom(comm,data, movecom = True, posf = fpos)

	def moviln(self,v,dx,dy,dz,da,db,dc,p=1):												#linear incremental move with current position read 

		""" Use Linear move command with increment data
			v  = velocity, see lv/rv flag
			dx = incremental position x
			dy = incremental position y
			dz = incremental position z
			da = incremental rotation x
			db = incremental rotation y
			dc = incremental rotation z		
		"""
		
		posr = self.redpos().split('\n')[1].split(',');										#read current position...
		posi = [float(i) for i in posr[0:6]]												#get position & rotation
		posm = [float(i) for i in [dx, dy, dz, da, db, dc]];								#parse input vector
		fpos = map(sum,zip(posi,posm))														#add input vector to current positon...	
		comm = 'MOVL'
		data = '0, ' + str(v) + ',0,'+str(fpos[0])+','+str(fpos[1])+','+str(fpos[2])+','+str(fpos[3])+','+str(fpos[4])+','+str(fpos[5])+','+posr[6]+',0,0,0,0,0,0,0\r'
		return self.sencom(comm,data, movecom = True, posf = fpos)

	def mvpath(pts=[], inc=0, pls=0, xyz=0, jnt=0, lin=0, ind=0):							#multipoint move

		""" Send Continuous fire points
		
		pts = list of each point with v,px,py,pz,rx,ry,rz,type for absolute or pulse motion
		pts = list of each point with v,dx,dy,dz,da,db,dc,     for incremental motion
		
		ind = flag to set if motion settings are set individually 

		if 1, 

			inc = inc[i] = 1 if pts[i] is incremenetal 	else 0
			pls = pls[i] = 1 if pts[i] is pulse motion 	else 0
			xyz = xyz[i] = 1 if pts[i] is absolute move	else 0

			jnt = jnt[i] = 1 if pts[i] is joint  motion else 0
			lin = lin[i] = 1 if pts[i] is linear motion else 0

			length of point and motion definition must be length of points

		if 0, 

			all point definitions are set to either incremental 	= if inc = 1
												 or pulse			= if pls = 1
												 or absolute		= if xyz = 1
											
			all motion types are set to joint 						= if jnt = 1
									 or linear						= if lin = 1
									 
			either  jnt or lin must be set to 1
			either inc/pls/xyz must be set to 1
		
		"""

		if not len(pts) > 0:				 			return 1 	#atleast one point required						#error 1 not enough points
		if not all(len(a) == 7 for a in pts):			return 2 	#atleast v + 6axis required						#error 2 points incompletely defined
		if xyz and not all(len(a) == 8 for a in pts):	return 3 	#orientation types required						#error 3 type variable not sent for absolute motion

		if not ind:													#if individual motion not specified
			inc = [inc]*len(pts);
			pls = [pls]*len(pts);
			xyz = [xyz]*len(pts);
			jnt = [jnt]*len(pts);
			lin = [lin]*len(pts);
		else:														#ensure individual motion for each point in path
			if not all(len(a) == len(pts) for a in [inc,pls,xyz,jnt,lin]): return 4									#error 4 motion types for each point not specified

		path    = [[],[]]											#create path point list
		path[0] = ['']*len(pts)										#comm list
		path[1] = ['']*len(pts)										#data list

		com1    = 'CONNECT Robot_access Keep-Alive:-1\r'			#host control request -> infinite continuous fire
		com2    = 'HOSTCTRL_REQUEST '								#command header

		for i in range(0,len(pts)):									#parse each command and data in path

			v = str(pt[0])+','
			p = ', '.join(map(str,pts[1:6])) + ', '
			
			if inc[i]:

				if jnt[i]:

					path[i][1] = '1,' + v + '0,' + p + '0,0,0,0,0,0,0,0\r'
					path[i][0] = com2 + 'IMOV ' + str(len(path[1][i])) + '\r' 

				elif lin[i]:
					
					path[i][1] = '1,' + v + '0,' + p + '0,0,0,0,0,0,0,0\r'
					path[i][0] = com2 + 'IMOV ' + str(len(path[1][i])) + '\r' 

			elif pls[i]:

				if   jnt[i]:

					path[i][1] = v + p + '0,0,0,0,0,0,0\r'
					path[i][0] = com2 + 'PMOVJ ' + str(len(path[1][i])) + '\r' 

				elif lin[i]:

					path[i][1] = '0, ' + v + p + '0,0,0,0,0,0,0\r'
					path[i][0] = com2 + 'PMOVL ' + str(len(path[1][i])) + '\r' 

			elif xyz[i]:

				if   jnt:	

					t = str(pts[7]) + ',' if len(pts)<8 else '6,' 
					path[i][1] = v + '0,' + p + t + '0,0,0,0,0,0,0\r'
					path[i][0] = com2 + 'PMOVL ' + str(len(path[1][i])) + '\r' 

				elif lin:	

					t = str(pts[7]) + ','
					path[i][1] = '0, ' + v + '0,' + p + t + '0,0,0,0,0,0,0\r'
					path[i][0] = com2 + 'PMOVL ' + str(len(path[1][i])) + '\r' 

		sock    = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  #open socket to robot for continuous fire
		
		try:    sock.connect((self.IP_ADD,self.TCP_PT))
		except: print("Error! Cannot Connect Socket to Robot"); sys.exit()

		self.sock.send(com1); resp = self.sock.recv(256);
		if not 'Keep-Alive:-1' in resp: print("Error! Cannot Connect Socket to Robot");sys.exit();

		i=0;
		while i < len(path):										#send each command
			
			j=1;													#Monitor Running Bit Status
			while j:
				self.sock.send(com1 + 'RSTATS 0');resp  = self.sock.recv(256);resp += self.sock.recv(256)
				j = int(''.join(['{0:08b}'.format(int(q)) for q in resp.split('\n')[1].split(',')])[4])

			self.sock.send(path[i][0]);resp  = self.sock.recv(256)	#Send Next Path Command 
			self.sock.send(path[i][1]);resp += self.sock.recv(256)	#Send Next Path Command Data
			print(resp)
			i+=1;
		
		return 0


#~ -----------------------------------------------------------------------------------------------------------------------------------------
#~ UDP COMMANDS
#~ -----------------------------------------------------------------------------------------------------------------------------------------
	
	
	def udp_rtrq(self):																		#udp read joint torque

		"""Doc

			#~ ----------------------------
			#~ Note: Read Joint Torques
			#~ ----------------------------

		"""

		comm = '\x59\x45\x52\x43\x20\x00\x00\x00\x03\x01\x00\x00\x00\x00\x00\x00\x39\x39\x39\x39\x39\x39\x39\x39\x77\x00\x01\x00\x00\x01\x00\x00'
		data = ''

		# ~ --------------------------------------------------------------------------------------------------------------------------------
		# ~ Send Command Receive Data
		# ~ --------------------------------------------------------------------------------------------------------------------------------

		while self.rob_chkout: pass
		self.rob_chkout = True;
		self.sock_udp.sendto(comm,("192.168.1.31",10040))
		data,addr = self.sock_udp.recvfrom(512)
		self.rob_chkout = False;

		# ~ --------------------------------------------------------------------------------------------------------------------------------
		# ~ Parse Received Data
		# ~ --------------------------------------------------------------------------------------------------------------------------------

		nib = []				
		axs = []				
		
		if len(data) > 32:		
			
			reqdat = data[32:]
			for i in xrange(0,len(reqdat),4): nib.append(reqdat[i:i+4])
			for i in range(5,11): axs.append(struct.unpack('<i',nib[i])[0])

		if not ord(data[25]) + ord(data[26]): return float(ax[0]),float(ax[1]),float(ax[2]),float(ax[3]),float(ax[4]),float(ax[5])
		else: print("Error with Torque Read Command")

		if self.dbg or not(not ord(data[25]) + ord(data[26])): self.udp_dbug(comm,data)
		
		return -1

	def udp_iorw(self, addr=27010, wrfl = 0, bits=[0,0,0,0,0,0,0,0]):						#udp i.o. readwrite 	

		"""doc
		# ~ wrfl = read or write flag,
		#~ 	   0 = Read
		#~     1 = Write
		# ~ addr = io register specified as addr, divied by 10 to fit 2 bytes
		# ~ bits = set values, must write 8 bits at a time.

		"""

		# ~ ------------------------------------------------------------------------------------------------------------------------------------
		# ~ Parse Command
		# ~ ------------------------------------------------------------------------------------------------------------------------------------

		if wrfl:
			a = 0
			for bit in bits: a = (a<<1) | bit
			comm = '\x59\x45\x52\x43\x20\x00\x04\x00\x03\x01\x00\x00\x00\x00\x00\x00\x39\x39\x39\x39\x39\x39\x39\x39\x78\x00' + struct.pack('<H',addr/10) + '\x01\x10\x00\x00' + struct.pack('<B',a) + '\x00\x00\x00'
		else:
			comm = '\x59\x45\x52\x43\x20\x00\x00\x00\x03\x01\x00\x00\x00\x00\x00\x00\x39\x39\x39\x39\x39\x39\x39\x39\x78\x00' + struct.pack('<H',addr/10) + '\x01\x0e\x00\x00'

		data = ''

		# ~ --------------------------------------------------------------------------------------------------------------------------------
		# ~ Send Command Receive Data
		# ~ --------------------------------------------------------------------------------------------------------------------------------

		while self.rob_chkout: pass
		self.rob_chkout = True;
		self.sock_udp.sendto(comm,("192.168.1.31",10040))
		data,addr = self.sock_udp.recvfrom(512)
		self.rob_chkout = False;

		# ~ --------------------------------------------------------------------------------------------------------------------------------
		# ~ Parse Received Data
		# ~ --------------------------------------------------------------------------------------------------------------------------------

		if not wrfl:													#if not write, return data recv
			bit = [-1,-1,-1,-1,-1,-1,-1,-1]								#No response
			if len(data) > 32:											#parse if response
				dt = struct.unpack('B',data[32])						#unpack response byte
				bit = [int(x) for x in '{0:08b}'.format(dt[0])] 		#parse bits
				if not ord(data[25]) + ord(data[26]):return bit			#return result if no errror
				else: print("Error with IO Write Command")

		else:															#if write,     return data sent
			if not ord(data[25]) + ord(data[26]): return bits			
			else: print("Error with IO Read Command")

		if self.dbg or not(not ord(data[25]) + ord(data[26])): self.udp_dbug(comm,data)

		return -1

	def get_word(self, w,o):																#get 32-bit int, 16		

		""" Doc
		#~ Notes:
		#~ w = number to create word packet (32 bit signed integer)
		#~ o = order multiplier to number to create integer 10e^o
		"""

		a = w
		b = math.modf(a);
		c = b[1]*10**o;
		d = b[0]*10**o;
		e = int(c+d);
		f = struct.pack('<i',e)

		return f

	def udp_rpos(self, p=0):																#udp read position 		

		"""doc
		# ~ read robot position using udp server
			command hard coded to return cartesian data
			possible to request pulse data with flag p = 1
				
		if 0: #debug.print Parsed Data

				print "----------------------------------------------------------------------------"
				print "Parsed Data..."
				print "----------------------------------------------------------------------------"

				if not p:
					print "  PX: ", axs[0]
					print "  PY: ", axs[1]
					print "  PZ: ", axs[2]
					print "  AX: ", axs[3]
					print "  AY: ", axs[4]
					print "  AZ: ", axs[5]
					print "  TP: ", t
					print "  ET: ", e
				else:
					print "  PS: ", axs[0]
					print "  PL: ", axs[1]
					print "  PU: ", axs[2]
					print "  PR: ", axs[3]
					print "  PB: ", axs[4]
					print "  PT: ", axs[5]

				print "----------------------------------------------------------------------------"

		"""

		if not p: 	comm = '\x59\x45\x52\x43\x20\x00\x00\x00\x03\x01\x00\x00\x00\x00\x00\x00\x39\x39\x39\x39\x39\x39\x39\x39\x75\x00\x65\x00\x00\x01\x00\x00'
		else:		comm = '\x59\x45\x52\x43\x20\x00\x00\x00\x03\x01\x00\x00\x00\x00\x00\x00\x39\x39\x39\x39\x39\x39\x39\x39\x75\x00\x01\x00\x00\x01\x00\x00'

		data = ''

		# ~ --------------------------------------------------------------------------------------------------------------------------------
		# ~ Send Command Receive Data
		# ~ --------------------------------------------------------------------------------------------------------------------------------

		while self.rob_chkout: pass
		self.rob_chkout = True;
		self.sock_udp.sendto(comm,("192.168.1.31",10040))
		data,addr = self.sock_udp.recvfrom(512)
		self.rob_chkout = False;

		# ~ --------------------------------------------------------------------------------------------------------------------------------
		# ~ Parse Received Data
		# ~ --------------------------------------------------------------------------------------------------------------------------------
				
		nib = []				#list of 4byte chunks
		axs = []				#list of axis coordinates
		
		if len(data) > 32:		
			
			reqdat = data[32:]														#get data part of packet
			for i in xrange(0,len(reqdat),4): nib.append(reqdat[i:i+4]) 			#separate data words and extract requested data
			for i in range(5,11): axs.append(struct.unpack('<i',nib[i])[0])			#unpack 4 byte packets as signed 32 bit integer

			if not p:																#Parse cartesian data
				for i in range(0,3): axs[i] = axs[i]/1000.							#10e-3 for position
				for i in range(3,6): axs[i] = axs[i]/10000.							#10e-4 for orientation
				t = [hex(ord(x))[2:].zfill(2) for x in nib[1]]						#get pose type for cartesian
				e = [hex(ord(x))[2:].zfill(2) for x in nib[4]]						#extended type for cartesian

		if not ord(data[25]) + ord(data[26]):	
			if not p: 	return [axs[0],axs[1],axs[2],axs[3],axs[4],axs[5],t,e]
			else:		return [axs[0],axs[1],axs[2],axs[3],axs[4],axs[5]]
		else: print(msg="Error with Position Read Command")

		if self.dbg or not(not ord(data[25]) + ord(data[26])): self.udp_dbug(comm,data)

		return -1

	def udp_rstt(self): 																	#-> read status			

		"""doc
		# ~ Read Robot Status Byte 1 & 2

		#~ byte 1:
		
			#~ bit 0: Mode Step
			#~ bit 1: Mode Cycle
			#~ bit 2: Mode Continuous
			#~ bit 3: Is Running
			#~ bit 4: Is Safety
			#~ bit 5: Mode Teach
			#~ bit 6: Mode Play
			#~ bit 7: Mode Remote

		#~ byte 2:

			#~ bit 0: Unused
			#~ bit 1: Hold Pendant
			#~ bit 2: Hold External
			#~ bit 3: Hold Remote
			#~ bit 4: Alarm Flag
			#~ bit 5: Error Flag
			#~ bit 6: Servo Status
			#~ bit 7: Unused
		
		"""

		comm = '\x59\x45\x52\x43\x20\x00\x00\x00\x03\x01\x00\x00\x00\x00\x00\x00\x39\x39\x39\x39\x39\x39\x39\x39\x72\x00\x01\x00\x00\x01\x00\x00'
		data = ''

		# ~ --------------------------------------------------------------------------------------------------------------------------------
		# ~ Send Command Receive Data
		# ~ --------------------------------------------------------------------------------------------------------------------------------

		while self.rob_chkout: pass
		self.rob_chkout = True;
		self.sock_udp.sendto(comm,("192.168.1.31",10040))
		data,addr = self.sock_udp.recvfrom(512)
		self.rob_chkout = False;

		# ~ --------------------------------------------------------------------------------------------------------------------------------
		# ~ Parse Received Data
		# ~ --------------------------------------------------------------------------------------------------------------------------------
		
		if len(data) > 32:
			dt1 = struct.unpack('B',data[32])
			dt2 = struct.unpack('B',data[36])
			stt = [int(x) for x in '{0:08b}'.format(dt1[0])] + [int(x) for x in '{0:08b}'.format(dt2[0])]

		if not ord(data[25]) + ord(data[26]): return stt
		else: print("Error with Status Command")

		if self.dbg or not(not ord(data[25]) + ord(data[26])): self.udp_dbug(comm,data)

		return -1

	def udp_ralm(self): 																	#-> read alarm			

		""" Doc
			----------------------------------------
			Notes:
			----------------------------------------
			Function to Read Last Alarm
			----------------------------------------
		"""

		# ~ ------------------------------------------------------------------------------------------------------------------------------------
		# ~ Parse Command
		# ~ ------------------------------------------------------------------------------------------------------------------------------------

		comm = '\x59\x45\x52\x43\x20\x00\x00\x00\x03\x01\x00\x00\x00\x00\x00\x00\x39\x39\x39\x39\x39\x39\x39\x39\x70\x00\x01\x00\x01\x0e\x00\x00'
		data = ''
		
		# ~ --------------------------------------------------------------------------------------------------------------------------------
		# ~ Send Command Receive Data
		# ~ --------------------------------------------------------------------------------------------------------------------------------

		while self.rob_chkout: pass
		self.rob_chkout = True;
		self.sock_udp.sendto(comm,("192.168.1.31",10040))
		data,addr = self.sock_udp.recvfrom(512)
		self.rob_chkout = False;

		# ~ --------------------------------------------------------------------------------------------------------------------------------
		# ~ Parse Received Data
		# ~ --------------------------------------------------------------------------------------------------------------------------------
		
		a = [-1,-1,-1,-1]
		if len(data) > 32: a = [hex(ord(x))[2:].zfill(2) for x in data[32:36]]

		if not ord(data[25]) + ord(data[26]): return a
		else: print("Error with Alarm Read Command")

		if self.dbg or not(not ord(data[25]) + ord(data[26])): self.udp_dbug(comm,data)

		return -1

	def udp_rset(self):																		#-> reset alarm	& error	

		""" Doc
			----------------------------------------
			Notes:
			----------------------------------------
			Function: 	Cancel Alarm & Error Status
						Required to Resume Servo On
			----------------------------------------
		"""

		# ~ ------------------------------------------------------------------------------------------------------------------------------------
		# ~ Parse Command Comm1 = Cancel Alarm, Comm2 = Cancel Error
		# ~ ------------------------------------------------------------------------------------------------------------------------------------

		comm1 = '\x59\x45\x52\x43\x20\x00\x04\x00\x03\x01\x00\x00\x00\x00\x00\x00\x39\x39\x39\x39\x39\x39\x39\x39\x82\x00\x01\x00\x01\x10\x00\x00\x01\x00\x00\x00'
		comm2 = '\x59\x45\x52\x43\x20\x00\x04\x00\x03\x01\x00\x00\x00\x00\x00\x00\x39\x39\x39\x39\x39\x39\x39\x39\x82\x00\x02\x00\x01\x10\x00\x00\x01\x00\x00\x00'

		data = ''
		
		# ~ --------------------------------------------------------------------------------------------------------------------------------
		# ~ Send Command Receive Data
		# ~ --------------------------------------------------------------------------------------------------------------------------------

		while self.rob_chkout: pass
		self.rob_chkout = True;

		self.sock_udp.sendto(comm1,("192.168.1.31",10040))
		data1,addr = self.sock_udp.recvfrom(512)

		self.sock_udp.sendto(comm2,("192.168.1.31",10040))
		data2,addr = self.sock_udp.recvfrom(512)	

		self.rob_chkout = False;

		# ~ --------------------------------------------------------------------------------------------------------------------------------
		# ~ Parse Received Data
		# ~ --------------------------------------------------------------------------------------------------------------------------------

		recStatusByte1 = ord(data1[25]) + ord(data1[26])	
		recStatusByte2 = ord(data2[25]) + ord(data2[26])	

		if not recStatusByte1 and not recStatusByte2: return 1
		else: print("Error with Reset Command")

		if self.dbg or not(not ord(data[25]) + ord(data[26])): self.udp_dbug(comm1,data1)
		if self.dbg or not(not ord(data[25]) + ord(data[26])): self.udp_dbug(comm2,data2)

		return -1
		
	def udp_serv(self,on=1): 																#-> servo on off		

		if on:	comm = '\x59\x45\x52\x43\x20\x00\x04\x00\x03\x01\x00\x00\x00\x00\x00\x00\x39\x39\x39\x39\x39\x39\x39\x39\x83\x00\x02\x00\x01\x10\x00\x00\x01\x00\x00\x00'
		else:	comm = '\x59\x45\x52\x43\x20\x00\x04\x00\x03\x01\x00\x00\x00\x00\x00\x00\x39\x39\x39\x39\x39\x39\x39\x39\x83\x00\x02\x00\x01\x10\x00\x00\x02\x00\x00\x00'

		data = ''

		# ~ --------------------------------------------------------------------------------------------------------------------------------
		# ~ Send Command Receive Data
		# ~ --------------------------------------------------------------------------------------------------------------------------------

		while self.rob_chkout: pass
		self.rob_chkout = True;
		self.sock_udp.sendto(comm,("192.168.1.31",10040))
		data,addr = self.sock_udp.recvfrom(512)
		self.rob_chkout = False;

		# ~ --------------------------------------------------------------------------------------------------------------------------------
		# ~ Parse Received Data
		# ~ --------------------------------------------------------------------------------------------------------------------------------
		
		if not ord(data[25]) + ord(data[26]):return 1
		else: print("Error with Servo Command")

		if self.dbg or not(not ord(data[25]) + ord(data[26])): self.udp_dbug(comm,data)

		return -1

	def udp_rsaf(self,s=1): 																#read Safety Bits implementation of iorw

		""" Doc

		Read the Safety IO Bits
		Note the Registers May Be Dependent on Wiring & Logical Setup

		For All Robots:

		E-stop 			Status at Reg 80020
		Area Scanner	Status at Reg 80400

		For Collaborative Robots Only:
		
		Bump			Status at Reg 81380
		Hard Bump		Status at Reg 81382
		Soft Bump		Status at Reg 81383

		Input s: s=0 non collaborative robot, s=1 collaborative safe robot
		"""

		a = self.udp_iorw(addr = 80020)
		b = self.udp_iorw(addr = 80400)

		if s: c = self.udp_iorw(addr = 81380)

		pstp = a[1]
		estp = a[2]
		astp = a[4]
		asaf = b[7]

		if s: hard=c[5];soft=c[6];
		else: hard= -1 ;soft= -1 ;

		return [pstp,estp,astp,asaf,hard,soft]

	def udp_movj(self,args):																#udp move cartesian

		""" Doc
		# ~ --------------------------------------------------------------------------------------------------------------------
		# ~ Notes
		# ~ --------------------------------------------------------------------------------------------------------------------
		
		# ~ this function uses the yaskawa hi-speed udp server to move robot
		
		# ~ inputs:

		# ~ 	m = motion Type, 	
		# ~			1 = joint, 		
		# ~			2 = linear, 
		# ~			3 = linear increment

		# ~ 	s = speed  Type,	
		# ~			1 = Percentage of Max Speed, 		for m = 1 only
		# ~			2 = Linear   speed in 0.1 mm/s, 	for m = 2,3 only
		# ~			3 = Rotation speed in 0.1 deg/s, 	for m = 2,3 only

		# ~		v = Speed Value, must be specified in the type specified by s, no checks performed
		
		# ~		px= X Coordinate, 	specified in milimeters and converted to micro meters (10e-6)
		# ~ 	py= Y Coordinate, 	specified in milimeters and converted to micro meters (10e-6) 	
		# ~ 	py= Z Coordinate, 	specified in milimeters and converted to micro meters (10e-6) 	
		# ~		rx= X Rotation, 	specified in degrees    and converted to 0.1 mili deg (10e-4) 	
		# ~ 	ry= Y Rotation, 	specified in degrees    and converted to 0.1 mili deg (10e-4) 	
		# ~ 	rz= Z Rotation, 	specified in degrees    and converted to 0.1 mili deg (10e-4) 	

		# ~		t = Orientation Type, axis coordinate and flip conditions (Hard Coded)

		"""

		m, s, v, px, py, pz, rx, ry, rz, t, e = args;
		
		# ~ ------------------------------------------------------------------------------------------------------------------------------------
		# ~ Parse Header
		# ~ ------------------------------------------------------------------------------------------------------------------------------------

		if 1:
			comm = '\x59\x45\x52\x43\x20\x00\x68\x00\x03\x01\x00\x00\x00\x00\x00\x00\x39\x39\x39\x39\x39\x39\x39\x39'
			# ~ ------------------------------------------------------------------------------------------------------------------------------------
			comm = comm + '\x8a\x00'										#-> Command ID Number for Move Command
			# ~ ------------------------------------------------------------------------------------------------------------------------------------
			if 		m == 1:	comm = comm + '\x01\x00'						#-> Command Instance: Motion Type 1: Joint
			elif 	m == 2:	comm = comm + '\x02\x00'						#-> Command Instance: Motion Type 2: Linear Absolute
			elif 	m == 3:	comm = comm + '\x03\x00'						#-> Command Instance: Motion Type 2: Linear Increment
			comm = comm + '\x01\x02\x00\x00'

		# ~ ------------------------------------------------------------------------------------------------------------------------------------
		# ~ Parse Data
		# ~ ------------------------------------------------------------------------------------------------------------------------------------

		if 1:
			
			#Robot & Station ID-----------------------------------------------------------------------------------------------------------------
			comm = comm + '\x01\x00\x00\x00'								#-> Data word  1: Robot   Number (Hard Coded to 1)
			comm = comm + '\x00\x00\x00\x00'								#-> Data word  2: Station Number (Hard Coded to 0)
			#speed type-------------------------------------------------------------------------------------------------------------------------
			if 		s == 1:	comm = comm + '\x00\x00\x00\x00'				#-> Data word  3: Speed Type 1: % Max  speed in 0.01 %
			elif 	s == 2:	comm = comm + '\x01\x00\x00\x00'				#-> Data word  3: Speed Type 2: Linear Speed in 0.1  mm/s
			elif 	s == 3:	comm = comm + '\x02\x00\x00\x00'				#-> Data word  3: Speed Type 3: Rotate Speed in 0.1 deg/s
			#speed for speed type---------------------------------------------------------------------------------------------------------------
			if 		s == 1: comm = comm + self.get_word(max(min(v,100),0.01),2)	#-> Data word  4: Robot Motion Speed in 0.01%
			elif 	s == 2: comm = comm + self.get_word(max(min(v,999),0.10),1)	#-> Data word  4: Robot Motion Speed in 0.1mm/s
			elif 	s == 3: comm = comm + self.get_word(max(min(v,499),0.10),1)	#-> Data word  4: Robot Motion Speed in 0.1deg/s
			#Co-ordinate Frame------------------------------------------------------------------------------------------------------------------
			comm = comm + self.get_word(16,0)									#-> Data word  5: Coordinate Frame Hard Coded to Base Frame
			#Robot Position & Tool Orientation--------------------------------------------------------------------------------------------------
			comm = comm + self.get_word(px,3)									#-> Data word  6: Robot X position in 1e-3 mm
			comm = comm + self.get_word(py,3)									#-> Data word  7: Robot Y position in 1e-3 mm
			comm = comm + self.get_word(pz,3)									#-> Data word  8: Robot Z position in 1e-3 mm
			comm = comm + self.get_word(rx,4)									#-> Data word  9: Robot X rotation in 1e-4 deg
			comm = comm + self.get_word(ry,4)									#-> Data word 10: Robot Y rotation in 1e-4 deg
			comm = comm + self.get_word(rz,4)									#-> Data word 11: Robot Z rotation in 1e-4 deg
			#0 padding for words 12 to 13 (reserve)---------------------------------------------------------------------------------------------
			comm = comm + self.get_word(0,0)										#-> Data word 12: Pad Reserve with 0s
			comm = comm + self.get_word(0,0)										#-> Data word 13: Pad Reserve with 0s
			#0 padding for words 12 to 13 (unused)----------------------------------------------------------------------------------------------
			comm = comm + self.get_word(3,0)										#-> Data word 14: Hard coded Orientation Type to \x03
			comm = comm + self.get_word(0,0)										#-> Data word 15: Hard coded    Extended Type to \x00	
			#0 padding for words 15 to 22 (unused)----------------------------------------------------------------------------------------------
			for i in range(16,27): comm = comm + self.get_word(0,0)				#-> Data word 16-26: Pad Unused with 0s

		# ~ --------------------------------------------------------------------------------------------------------------------------------
		# ~ Send Command Receive Data
		# ~ --------------------------------------------------------------------------------------------------------------------------------

		data = '';

		while self.rob_chkout: pass
		self.rob_chkout = True;
		self.sock_udp.sendto(comm,("192.168.1.31",10040))
		data,addr = self.sock_udp.recvfrom(512)
		self.rob_chkout = False;

		# ~ --------------------------------------------------------------------------------------------------------------------------------
		# ~ Parse Received Data
		# ~ --------------------------------------------------------------------------------------------------------------------------------

		#~ if not ord(data[25]) + ord(data[26]):

		if m == 3: # do not re-send increment move because of move wait		
			m = 2; cur_pos = self.udp_rpos()[0:6];	
		
			px = px + cur_pos[0];py = py + cur_pos[1];pz = pz + cur_pos[2];
			rx = rx + cur_pos[3];ry = ry + cur_pos[4];rz = rz + cur_pos[5];

			args = (m, s, v, px, py, pz, rx, ry, rz, t, e); 

		pos = [px, py, pz, rx, ry, rz]

		self.udp_wait(self.udp_movj,args,pos);

		if self.dbg or not(not ord(data[25]) + ord(data[26])): print("Error with Joint Move Command");self.udp_dbug(comm,data);return -1;
		return 1

	def udp_movp(self,args): 																#udp move pulse

		m, s, v, ps, pl, pu, pr, pb, pt, pos = args

		""" Doc
		# ~ --------------------------------------------------------------------------------------------------------------------
		# ~ Notes
		# ~ --------------------------------------------------------------------------------------------------------------------
		
		# ~ this function uses the yaskawa hi-speed udp server to move robot using pulse 
		
		# ~ inputs:

		# ~ 	m = motion Type, 	
		# ~			1 = joint, 		
		# ~			2 = linear, 

		# ~ 	s = speed  Type,	
		# ~			1 = Percentage of Max Speed, 		for m = 1 only 		
		# ~			2 = Linear   speed in 0.1 mm/s, 	for m = 2,3 only
		# ~			3 = Rotation speed in 0.1 deg/s, 	for m = 2,3 only

		# ~		v = Speed Value, must be specified in the type specified by s, no checks performed
		
		# ~		ps= S Rotation, 	specified in pulse
		# ~ 	pl= L Rotation, 	specified in pulse
		# ~ 	pu= U Rotation, 	specified in pulse
		# ~		pr= R Rotation, 	specified in pulse
		# ~ 	pb= B Rotation, 	specified in pulse
		# ~ 	pt= T Rotation, 	specified in pulse

		#~ 		pos = List of cartesian Position Equivalent of Pulse Rotations
		"""
		
		# ~ --------------------------------------------------------------------------------------------------------------------------------
		# ~ Parse Header
		# ~ --------------------------------------------------------------------------------------------------------------------------------

		if 1:
			
			#~ # ~ -------------------------------------------------------------------------------------------------------------------------
			comm = '\x59\x45\x52\x43\x20\x00\x58\x00\x03\x01\x00\x00\x00\x00\x00\x00\x39\x39\x39\x39\x39\x39\x39\x39'
			# ~ ----------------------------------------------------------------------------------------------------------------------------
			comm = comm + '\x8b\x00'										#-> Command ID Number for Move Command		
			# ~ ----------------------------------------------------------------------------------------------------------------------------
			if 		m == 1:	comm = comm + '\x01\x00'						#-> Command Instance: Motion Type 1: Joint
			elif 	m == 2:	comm = comm + '\x02\x00'						#-> Command Instance: Motion Type 2: Linear 
			# ~ ----------------------------------------------------------------------------------------------------------------------------
			comm = comm + '\x01\x02\x00\x00'
			
		# ~ --------------------------------------------------------------------------------------------------------------------------------
		# ~ Parse Data
		# ~ --------------------------------------------------------------------------------------------------------------------------------

		if 1:

			#Robot & Station ID-------------------------------------------------------------------------------------------------------------
			comm = comm + '\x01\x00\x00\x00'								#-> Data word  1: Robot   Number (Hard Coded to 1)
			comm = comm + '\x00\x00\x00\x00'								#-> Data word  2: Station Number (Hard Coded to 0)
			#speed type---------------------------------------------------------------------------------------------------------------------
			if 		s == 1:	comm = comm + '\x00\x00\x00\x00'				#-> Data word  3: Speed Type 1: % Max  speed in 0.01 %
			elif 	s == 2:	comm = comm + '\x01\x00\x00\x00'				#-> Data word  3: Speed Type 2: Linear Speed in 0.1  mm/s
			elif 	s == 3:	comm = comm + '\x02\x00\x00\x00'				#-> Data word  3: Speed Type 3: Rotate Speed in 0.1 deg/s
			#speed for speed type-----------------------------------------------------------------------------------------------------------
			if 		s == 1: comm = comm + self.get_word(max(min(v,100),0.01),2)	#-> Data word  4: Robot Motion Speed in 0.01%
			elif 	s == 2: comm = comm + self.get_word(max(min(v,999),0.10),1)	#-> Data word  4: Robot Motion Speed in 0.1mm/s
			elif 	s == 3: comm = comm + self.get_word(max(min(v,499),0.10),1)	#-> Data word  4: Robot Motion Speed in 0.1deg/s
			#Robot Position & Tool Orientation----------------------------------------------------------------------------------------------
			comm = comm + self.get_word(ps,0)									#-> Data word  5: Robot X position in 1e-3 mm
			comm = comm + self.get_word(pl,0)									#-> Data word  6: Robot Y position in 1e-3 mm
			comm = comm + self.get_word(pu,0)									#-> Data word  7: Robot Z position in 1e-3 mm
			comm = comm + self.get_word(pr,0)									#-> Data word  8: Robot X rotation in 1e-4 deg
			comm = comm + self.get_word(pb,0)									#-> Data word  9: Robot Y rotation in 1e-4 deg
			comm = comm + self.get_word(pt,0)									#-> Data word 10: Robot Z rotation in 1e-4 deg
			#0 padding for words 11 to 22 (unused)------------------------------------------------------------------------------------------
			for i in range(11,23): comm = comm + self.get_word(0,0)				#-> Data word 11-22: Pad with 0s

		# ~ --------------------------------------------------------------------------------------------------------------------------------
		# ~ Send Command Receive Data
		# ~ --------------------------------------------------------------------------------------------------------------------------------

		data = ''

		while self.rob_chkout: pass
		self.rob_chkout = True;
		self.sock_udp.sendto(comm,("192.168.1.31",10040))
		data,addr = self.sock_udp.recvfrom(512)
		self.rob_chkout = False;

		# ~ --------------------------------------------------------------------------------------------------------------------------------
		# ~ Parse Received Data
		# ~ --------------------------------------------------------------------------------------------------------------------------------

		#~ if not ord(data[25]) + ord(data[26]):

		self.udp_wait(self.udp_movp,args,pos);

		if self.dbg or not(not ord(data[25]) + ord(data[26])): print("Error with Pulse Move Command");self.udp_dbug(comm,data);return -1;
		return 1

	def udp_wait(self,command, args, pos):													#wait for motion command

		#~ print "-----------------------------------------------------------------------------------------------------------------------------"
		#~ print "STARTING MOVE WAIT"
		#~ print "-----------------------------------------------------------------------------------------------------------------------------"

		dim = 100; saf = 4; run = 1; srv = 0; tog = 0; col = 1; ylo = False; ang = 100		#target;safety;runing;servof;toggle;light

		while dim > 10 or ang > 5 or run == 1 or srv == 0 or saf != 3:						#while command not complete

			if self.dbg:
				print "Position Error:   \t", dim
				print "Orientation Error:\t", ang, "(Discarded)"
				print "Running Bit:      \t", run
				print "Servo Bit:	     \t", srv
				print "Safe Bit:         \t", saf
				pass

			if 1:																			#read and calculate data

				msg = "";

				a = self.udp_rsaf();			
				b = self.udp_rstt();			
				c = self.udp_rpos(p=0)[0:6];	

				if a != -1:
					saf = a;
					col = saf[4] or saf[5];
					gat = saf[3];
					saf = sum(saf[0:3]);
				
				if b != -1:
					stt = b;
					mod = stt[0];
					srv = stt[9];
					run = stt[4];
					slo = stt[3];													

				if c != -1:
					pt1 = c;
					if not pos == None:																							#if check target flag is on
						dim = [pt1[0]-pos[0], pt1[1]-pos[1], pt1[2]-pos[2]]														#check if robot reached target
						dim = (dim[0]**2 + dim[1]**2 + dim[2]**2)**0.5															#calculate delta position norm
						#~ ang = [pt1[3]-pos[3], pt1[4]-pos[4], pt1[5]-pos[5]]													#check if robot reached target
						#~ ang = (ang[0]**2 + ang[1]**2 + ang[2]**2)**0.5														#calculate delta position norm
						ang = 0;																								#didnt work as well as i thought... 
					else: dim = 0;ang = 0																						#if not target check set to 0				

			if 1:																			#parse warnings if warning
				if mod!=1:	print(" Error! Robot Not in Command Mode");sys.exit()
				if col:		print("Error! Collaborative Safety Triggered.");self.udp_serv(on=0);srv = 0;

				if not srv:																	#if servo off   = trigger
					
					if 1:			print("Error! Servo Off.")								#send message servo off
					if col:			print("Error! Collaborative Safety Triggered")			#if collaborative trigger 
					if saf != 3: 	print("Error! E Stop Triggered.")						#if emergency stop trigger
						
					elif saf == 3 and not col:												#if off and safe

						print ("Safety Clear. Restoring Servo Power.")						#read alarm,reset alarm, restore servo
						self.udp_ralm();
						self.udp_rset();
						self.udp_serv();
						print ("Resuming Motion, Please Stay Back")
						command(args); return 1;

				if not gat and srv:			print("Safety Gate Triggered");	ylo = 1;			
				elif gat and srv and ylo:	print("Safety Gate Clear");		ylo = 0;

		#~ print "-----------------------------------------------------------------------------------------------------------------------------"
		#~ print "ENDING MOVE WAIT"; time.sleep(0.025);
		#~ print "-----------------------------------------------------------------------------------------------------------------------------"

		return 1

	def udp_dbug(self,comm,data):															#print udp command and response
		
		if 1:#split header & data
				
			senReqestData = comm[32:len(comm)]
			recReqestData = data[32:len(data)]
			
			datasize = len(data)
			commsize = len(comm)
			
		if 1: #comm head
			senIdentifier = comm[0:4]		#bytes 0,1,2,3					4 bytes
			senHeaderSize = comm[4:6]		#bytes 4,5						2 bytes
			senDataPartsz = comm[6:8]		#bytes 6,7						2 bytes
			senReserveBt1 = comm[8]			#bytes 8						1 bytes
			senPricessDiv = comm[9]			#bytes 9						1 bytes
			senAcknowledg = comm[10]		#bytes 10						1 bytes
			senRequest_ID = comm[11]		#bytes 11						1 bytes
			senBlock_numb = comm[12:16]		#bytes 12,13,14,15				4 bytes
			senReservebt2 = comm[16:24]		#bytes 16,17,18,19,20,21,22,23	8 bytes
			senCommandnum = comm[24:26]		#bytes 24,25					2 bytes
			senInstanceID = comm[26:28]		#bytes 26,27					2 bytes
			senAttributes = comm[28]		#bytes 28						1 bytes
			senServicsreq = comm[29]		#bytes 29						1 bytes
			senPaddingbyt = comm[30:32]		#bytes 30,31					2 bytes

		if 1: #resp head
			recIdentifier = data[0:4]		#bytes 0,1,2,3					4 bytes
			recHeaderSize = data[4:6]		#bytes 4,5						2 bytes
			recDataPartsz = data[6:8]		#bytes 6,7						2 bytes
			recReserveBt1 = data[8]			#bytes 8						1 bytes
			recPricessDiv = data[9]			#bytes 9						1 bytes
			recAcknowledg = data[10]		#bytes 10						1 bytes
			recRequest_ID = data[11]		#bytes 11						1 bytes
			recBlock_numb = data[12:16]		#bytes 12,13,14,15				4 bytes
			recReservebt2 = data[16:24]		#bytes 16,17,18,19,20,21,22,23	8 bytes
			recServiceByt = data[24]		#bytes 24						1 bytes
			recStatusByte = data[25]		#bytes 25						1 bytes
			recAddStatbyt = data[26]		#bytes 26						1 bytes
			recPaddingbyt = data[27]		#bytes 27						1 bytes
			recAddStatsiz = data[28:30]		#bytes 28,29					1 bytes
			recPaddingsiz = data[30:32]		#bytes 30,31					1 bytes
			
		if 1: #comm sent

			print "----------------------------------------------------------------------------"
			print "Total Bytes Sent: ", commsize 
			print "----------------------------------------------------------------------------"
			print "Identifier: ", [hex(ord(x))[2:].zfill(2) for x in senIdentifier]
			print "HeaderSize: ", [hex(ord(x))[2:].zfill(2) for x in senHeaderSize]
			print "DataPartsz: ", [hex(ord(x))[2:].zfill(2) for x in senDataPartsz]
			print "Reservebt1: ", [hex(ord(x))[2:].zfill(2) for x in senReserveBt1]
			print "ProcessDiv: ", [hex(ord(x))[2:].zfill(2) for x in senPricessDiv]
			print "Acknowledg: ", [hex(ord(x))[2:].zfill(2) for x in senAcknowledg]
			print "Request_ID: ", [hex(ord(x))[2:].zfill(2) for x in senRequest_ID]
			print "Block_numb: ", [hex(ord(x))[2:].zfill(2) for x in senBlock_numb]
			print "Reservebt2: ", [hex(ord(x))[2:].zfill(2) for x in senReservebt2]
			print "Commandnum: ", [hex(ord(x))[2:].zfill(2) for x in senCommandnum]
			print "InstanceID: ", [hex(ord(x))[2:].zfill(2) for x in senInstanceID]
			print "Attributes: ", [hex(ord(x))[2:].zfill(2) for x in senAttributes]
			print "Servicsreq: ", [hex(ord(x))[2:].zfill(2) for x in senServicsreq]
			print "Paddingsiz: ", [hex(ord(x))[2:].zfill(2) for x in senPaddingbyt]	

		if 1: #data sent

			print "----------------------------------------------------------------------------"
			print "SENT DATA: ", len(comm)-32, " bytes"
			print "----------------------------------------------------------------------------"
			if len(comm) > 32:
				comdat = [hex(ord(x))[2:].zfill(2) for x in senReqestData]
				for i in xrange(0,len(comdat),4):
					print comdat[i:i+4]

		if 1: #resp recd

			print "----------------------------------------------------------------------------"	
			print "Total Bytes Recd: ", datasize
			print "----------------------------------------------------------------------------"
			print "Identifier: ", [hex(ord(x))[2:].zfill(2) for x in recIdentifier]
			print "HeaderSize: ", [hex(ord(x))[2:].zfill(2) for x in recHeaderSize]
			print "DataPartsz: ", [hex(ord(x))[2:].zfill(2) for x in recDataPartsz]
			print "Reservebt1: ", [hex(ord(x))[2:].zfill(2) for x in recReserveBt1]
			print "ProcessDiv: ", [hex(ord(x))[2:].zfill(2) for x in recPricessDiv]
			print "Acknowledg: ", [hex(ord(x))[2:].zfill(2) for x in recAcknowledg]
			print "Request_ID: ", [hex(ord(x))[2:].zfill(2) for x in recRequest_ID]
			print "Block_numb: ", [hex(ord(x))[2:].zfill(2) for x in recBlock_numb]
			print "Reservebt2: ", [hex(ord(x))[2:].zfill(2) for x in recReservebt2]
			print "ServiceByt: ", [hex(ord(x))[2:].zfill(2) for x in recServiceByt]
			print "StatusByte: ", [hex(ord(x))[2:].zfill(2) for x in recStatusByte]
			print "AddStatbyt: ", [hex(ord(x))[2:].zfill(2) for x in recAddStatbyt]
			print "Paddingbyt: ", [hex(ord(x))[2:].zfill(2) for x in recPaddingbyt]
			print "AddStatsiz: ", [hex(ord(x))[2:].zfill(2) for x in recAddStatsiz]
			print "Paddingsiz: ", [hex(ord(x))[2:].zfill(2) for x in recPaddingsiz]

		if 1: #data recd

			print "----------------------------------------------------------------------------"
			print "RECD DATA: ", len(data)-32, " bytes"
			print "----------------------------------------------------------------------------"
			if len(data) > 32:
				reqdat = [hex(ord(x))[2:].zfill(2) for x in recReqestData]
				for i in xrange(0,len(reqdat),4):
					print reqdat[i:i+4]

		return 0

#~ -----------------------------------------------------------------------------------------------------------------------------------------
	#VAR READ WRITE FOR ON THE FLY JOB ***INCOMPLETE***
#~ -----------------------------------------------------------------------------------------------------------------------------------------

	def udp_pvar(self): #get set point
		
		""" Doc
		# ~ --------------------------------------------------------------------------------------------------------------------
		# ~ Notes
		# ~ --------------------------------------------------------------------------------------------------------------------
		
		# ~ this function uses the yaskawa hi-speed udp server to set or get Point Variable Data
		
		"""
		
		comm = ''
		data = ''

		if not ord(data[25]) + ord(data[26]):return 1
		return -1

	def udp_dvar(self): #get set double
		
		""" Doc
		# ~ --------------------------------------------------------------------------------------------------------------------
		# ~ Notes
		# ~ --------------------------------------------------------------------------------------------------------------------
		
		# ~ this function uses the yaskawa hi-speed udp server to set or get Double Variable Data
		
		"""
		
		comm = ''
		data = ''

		if not ord(data[25]) + ord(data[26]):return 1
		return -1

	def udp_ivar(self): #get set integer

		""" Doc
		# ~ --------------------------------------------------------------------------------------------------------------------
		# ~ Notes
		# ~ --------------------------------------------------------------------------------------------------------------------
		
		# ~ this function uses the yaskawa hi-speed udp server to set or get Integer Variable Data
		
		"""

		comm = ''
		data = ''

		if not ord(data[25]) + ord(data[26]):return 1
		return -1
		
	def udp_bvar(self): #get set byte
		
		""" Doc
		# ~ --------------------------------------------------------------------------------------------------------------------
		# ~ Notes
		# ~ --------------------------------------------------------------------------------------------------------------------
		
		# ~ this function uses the yaskawa hi-speed udp server to set or get Byte Variable Data
		
		"""
		
		comm = ''
		data = ''

		if not ord(data[25]) + ord(data[26]):return 1
		return -1

