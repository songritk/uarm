class Uarm():

	# 1.attach and detach certain servo
	servoAttach(servo_number)
	servoDetach(servo_number)
	

	# 2. detach and attch uarm (recommend)
	uarmAttach()
	uarmDetach()


	# 3. control certain angle and control 4 angles at the same time
	# Nneed to attach servo first
	writeServoAngle(servo_number,angle) 
	writeAngles(servo_1,servo_2,servo_3,servo_4)


	# 4. read current angle
	# return type - float 
	readAngle(servo_number)
	

	# 5. read current position
	# return type - float 
	currentX()
	currentY()
	currentZ()
	currentCoord()

	# 6. all moveTo s	
	moveTo(x,y,z)
	# add time parameter
	moveToWithTime(x,y,z,timeSpend)
	# control servo_4 and time at the same time. servo_4_relative should be 1 or 0
	moveToWithS4(x,y,z,timeSpend,servo_4_relative,servo_4)

	# 7. pump on and pump off
	pumpOn()
	pumpOff()

	# 8. stopper status
	# return type - True/False
	stopperStatus()

