'''
Motor Module example program.
'''

import motormodule as mm
import time
import matplotlib.pyplot as plt
import numpy as np
import csv
import datetime
class TestModule:
	def __init__(self,can_id,stand_time,COM):
		self.COM=COM
		self.can_id=can_id
		self.stand_time=stand_time
		self.mmc = mm.MotorModuleController(self.COM)		# Connect to the controller's serial port
		self.mmc.disable_motor(self.can_id)
		self.mmc.enable_motor(self.can_id)							# Enable motor with CAN ID 1
		# Send a command:  
		# (CAN ID, position setpoint, velocity setpoint, position gain, velocity gain, feed-forward torque)
		# Units are:  Radians, Rad/s, N-m/rad, N-m/rad/s, N-m
		self.mmc.send_command(self.can_id, 0, 0, 0, 0,  0)
		time.sleep(.1)
	def Read_Reply_Data(self,p_des,v_des,t_des,can_id):

		# This example reads the start position of the motor
		# Then sends position commands to ramp the motor position to zero 
		# With a 1st-order response
		# rx_values are ordered (CAN ID, Position, Velocity, Current)
		if can_id==1:
			real_position = self.mmc.rx_values_1[1]
			real_Velocity = self.mmc.rx_values_1[2]
			real_Current = self.mmc.rx_values_1[3]
		elif can_id==2:
			real_position = self.mmc.rx_values_2[1]
			real_Velocity = self.mmc.rx_values_2[2]
			real_Current = self.mmc.rx_values_2[3]
		else:
			real_position = self.mmc.rx_values_3[1]
			real_Velocity = self.mmc.rx_values_3[2]
			real_Current = self.mmc.rx_values_3[3]
		print("real_position:[ "+str(real_position[0])+" ]real_Velocity[ "+str(real_Velocity[0])+" ]real_Current[ "+str(real_Current[0])+" ]")
		print("Pos_Error"+str(p_des-real_position[0])+"Ves_Error"+str(v_des-real_Velocity[0])+"Current_Error"+str(t_des-real_Current[0]))
		#p_des = start_position[0]
		error=[p_des-real_position[0],v_des-real_Velocity[0],t_des-real_Current[0]]
		return error,[real_position,real_Velocity,real_Current]
	def disable_motor(self): 
		self.mmc.disable_motor(self.can_id) # Disable motor with CAN ID 1
	def cubicBezier(self,y0,yf,x):
		#y0,yf,x is 3 list
		bezier=[0,0,0]
		ydiff=[0,0,0]
		if x[0]<0.0 and x[1]<0.0 and x[2]<0.0:
			x=[0,0,0]
		if x[1]>1.0 and x[0]>1.0 and x[2]>1.0:
			x=[1.0,1.0,1.0]
		ydiff[0]=yf[0]-y0[0]
		ydiff[1]=yf[1]-y0[1]
		ydiff[2]=yf[2]-y0[2]

		bezier[0]=x[0]*x[0]*x[0]+3.0*(x[0]*x[0]*(1.0-x[0]))
		bezier[1]=x[1]*x[1]*x[1]+3.0*(x[1]*x[1]*(1.0-x[1]))
		bezier[2]=x[2]*x[2]*x[2]+3.0*(x[2]*x[2]*(1.0-x[2]))
		res=[0,0,0]
		res[0]=y0[0]+bezier[0]*ydiff[0]
		res[1]=y0[1]+bezier[1]*ydiff[1]
		res[2]=y0[2]+bezier[2]*ydiff[2]
		return res

	def cubicBezierFirstDerivative(self,y0,yf,x):
		#y0,yf,x is 3 list
		bezier=[0,0,0]
		ydiff=[0,0,0]
		if x[0]<0.0 and x[1]<0.0 and x[2]<0.0:
			x=[0,0,0]
		if x[1]>1.0 and x[0]>1.0 and x[2]>1.0:
			x=[1.0,1.0,1.0]
		ydiff[0]=yf[0]-y0[0]
		ydiff[1]=yf[1]-y0[1]
		ydiff[2]=yf[2]-y0[2]

		bezier[0]=6.0*(x[0]*(1.0-x[0]))
		bezier[1]=6.0*(x[1]*(1.0-x[1]))
		bezier[2]=6.0*(x[2]*(1.0-x[2]))
		res=[0,0,0]
		res[0]=bezier[0]*ydiff[0]/self.stand_time
		res[1]=bezier[1]*ydiff[1]/self.stand_time
		res[2]=bezier[2]*ydiff[2]/self.stand_time
		return res
	def speed_PID_control( self,PP, II, DD, desired_speed, real_velocity,send_speed):
		speedError = desired_speed - real_velocity
		if speedError >=0 :
			if speedError < 0.35 :
				speedError = 0
				# print("speed",speedError)
			output = PP * speedError + II * speedError + DD * speedError + send_speed
		elif speedError <= 0:
			if speedError > -0.35 :
				speedError = 0
				# print("speed",speedError)
			output = send_speed + PP * speedError + II * speedError + DD * speedError 
		# print (output)
		print("output",output,"speedError",speedError)
		return output
	def speed_PID_control2( self,PP, II, DD, desired_speed, real_velocity,send_speed,LastError,PrevError):
		speedError = desired_speed - real_velocity
		if speedError >=0 :
			if speedError < 0.4 :
				speedError = 0
				PrevError = 0
				LastError = 0
				# print("speed",speedError)
			output = PP * speedError - II * LastError + DD * PrevError + send_speed
			
		elif speedError <= 0:
			if speedError > -0.4 :
				speedError = 0
				PrevError = 0
				LastError = 0
				# print("speed",speedError)
			output = send_speed + PP * speedError - II * LastError + DD * PrevError  
		
		# print (output)
		print("output",output,"speedError",speedError)
		return output,LastError,PrevError

def main():
	can_id=1
	stand_time=2
	COM="COM19"
	tm=TestModule(can_id,stand_time,COM)
	# tm.mmc.zero_motor(can_id)
	# time.sleep(3)
	# tm.mmc.enable_motor(2)
	# tm.mmc.zero_motor(1)
	# time.sleep(3)
	# tm.mmc.zero_motor(3)
	# time.sleep(3)
	#tm.mmc.disable_motor(can_id)
	flag_1=0
	if flag_1==0:
		p_start=[0,0,0]
		p_des=[2,2,2]
	else:
		p_start=[2,2,2]
		p_des=[0,0,0]
	v_des=0
	pdres= [0]
	t_des=0.0
	kp=0
	kd=4
	# # kp=5
	# # kd=0.1

	P = 0.3
	I = 0
	D = 0.1
	send_speed_cmd = 0
	send_speed = 0
	LastError_cmd = 0
	PrevError_cmd = 0
	
	t_p_e=0
	error=[]
	dt=0.005
	cnt=0

	count=0
	detat=0.005
	flag=0
	#dt=0.01
	xx=[]#t
	yy=[]#pos
	realp=[]
	realv=[]
	desirev=[]
	open_draw_flag=0
	xx_v=[]
	yy_v=[]
	zz_v=[]
	open_draw_flag_v=0
	f = open('m50_0.csv','w',encoding='utf-8',newline='')
	csv_writer = csv.writer(f)
	csv_writer.writerow(["Time","P","V","C"])
	try:
		while(1):
			starttime=time.time()

			if flag==1:
				pres=tm.cubicBezier(p_start,p_des,[detat,detat,detat])
				# pdres=tm.cubicBezierFirstDerivative(p_start,p_des,[detat,detat,detat])
				# print(pres,pdres)
				if cnt>3:
					tm.mmc.send_command(can_id, pres[0] , pdres[0], kp, kd, t_des)
					error,realdata=tm.Read_Reply_Data(pres[0] , pdres[0],t_des,can_id)
					xx.append(detat)
					yy.append(pres)
					desirev.append(pdres)
					realp.append(realdata[0])
					realv.append(realdata[1])
					
					t_p_e=abs(p_des[0]-realdata[0][0])
					print("Desire",pres,pdres,realdata[0][0],t_p_e)
					# if t_p_e<0.1:
					# 	print("ok---- t_p_e",t_p_e)
					# 	open_draw_flag=1
					# 	break

			else:
				p_des_=0.0
				v_des_=5.26#200RPM/ 0.5-42 rad/s with
				t_des=0.0
				resdata=[0,0,0,0]
				error,realdata=tm.Read_Reply_Data(p_des_,v_des_,t_des,can_id)
				# send_speed = tm.speed_PID_control(P,I,D,v_des_,realdata[1][0], send_speed_cmd)

				send_speed,LastError,PrevError = tm.speed_PID_control2(P,I,D,v_des_,realdata[1][0],send_speed_cmd,LastError_cmd,PrevError_cmd)

				tm.mmc.send_command(can_id, 0, send_speed, kp, kd, t_des)
				send_speed_cmd = send_speed
				
				LastError_cmd = LastError
				PrevError_cmd = PrevError

				# tm.mmc.send_command(can_id, 0, v_des_, kp, kd, t_des)

				# error,realdata=tm.Read_Reply_Data(p_des_,v_des_,t_des,can_id)
				resdata[0]=str(datetime.datetime.now().day)+str("-")+str(datetime.datetime.now().hour)+str("-")+str(datetime.datetime.now().minute)+str("-")+str(datetime.datetime.now().second)
				resdata[1]=realdata[0][0]
				resdata[2]=realdata[1][0]
				resdata[3]=realdata[2][0]
				csv_writer.writerow(resdata)
				xx_v.append(detat)
				yy_v.append(v_des_)
				zz_v.append(realdata[1])
				# if cnt>300:
				# 	open_draw_flag_v=1
				# 	break
				t_p_e=abs(error[0])

			

				
			#endtime=time.time()
			#detat=endtime-start
			detat=cnt*dt
			cnt+=1
			print("----------",detat)
			time.sleep(.05)
			endtime=time.time()
			print("loop time",endtime-starttime)
		if open_draw_flag_v==1:
			open_draw_flag_v=0
			plt.figure(figsize=(10,5))
			plt.plot(np.array(xx_v),np.array(yy_v),color="deeppink",linewidth=2,linestyle=':',label='Vd', marker='o')
			plt.plot(np.array(xx_v),np.array(zz_v),color="goldenrod",linewidth=2,linestyle=':',label='Vr', marker='o')
			plt.show()
			tm.disable_motor()
		if open_draw_flag==1:
			open_draw_flag=0
			plt.figure(figsize=(10,5))
			plt.plot(np.array(xx),np.array(yy),color="deeppink",linewidth=2,linestyle=':',label='DPos', marker='o')
			plt.plot(np.array(xx),np.array(realp),color="darkblue",linewidth=1,linestyle='--',label='RPos', marker='+')
			plt.plot(np.array(xx),np.array(realv),color="goldenrod",linewidth=1.5,linestyle='-',label='RVelocity', marker='*')
			plt.plot(np.array(xx),np.array(desirev),color="yellow",linewidth=1.5,linestyle='-',label='DVelocity', marker='x')
			plt.legend(loc=2,labels=['DPos','RPos','RVelocity','DVelocity'])
			plt.show()
			tm.disable_motor()
	except KeyboardInterrupt:
		tm.disable_motor()
		f.close()
	tm.disable_motor()
	
if __name__=='__main__':
	main()

	
	