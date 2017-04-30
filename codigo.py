#!/usr/bin/env python
# Autor> Ruben Claveria Vega
# Curso> EL5206 Laboratorio de Inteligencia Computacional y Robotica

# En lugar de 'testpy', deberia ir el nombre del paquete creado.
import roslib; roslib.load_manifest('EL5206')
import robot_utilities
import time
import rospy
import math
import numpy
from sensor_msgs.msg import LaserScan 
from nav_msgs.msg import Odometry     
from geometry_msgs.msg import Twist
from std_msgs.msg import String 
import matplotlib.pyplot as plt
import random

class Controller:
	def __init__(self):

		# inicializa nodo ROS
		rospy.init_node('Simulation');

		# robotID = ID del robot que se desea controlar
		# si hay solo un robot, robotID debe ser ""
		# si hay mas de un robot, robotID debe ser de la forma "/robot_0", "/robot_1", etc.
		robotID = ""

		# Posicion y orientacion iniciales.
		# IMPORTANTE: deben ser consistentes con la del archivo .world
		init_x = -3.0
		init_y = -4.0
		init_angle = 90.0

		# creacion de un objeto de tipo Robot
		R = robot_utilities.Robot(robotID, init_x, init_y, init_angle)

		# Subscripcion a los topicos de interes: odometria (Odometry) y sensor de distancia (LaserScan)
		rospy.Subscriber(robotID+'/odom', Odometry, R.odom_callback)
		rospy.Subscriber(robotID+'/base_scan', LaserScan, R.ranger_callback)
		# Observacion: Las funciones de callback se ejecutan cada vez que el programa recibe informacion 
		# desde un topico. Se utilizan, generalmente, para actualizar datos.
		# Pueden utilizarse funciones de callback programadas por el alumno, 
		# diferentes a las provistas por la clase Robot

		# Se recomienda que este ciclo vaya antes de todas las instrucciones de control
		# Sirve para asegurar que el Robot recibio su primera lectura de sensor
		while not rospy.is_shutdown() and len(R.distances)==0:
			R.rate.sleep()

		# Ciclo de ejemplo. 
		# Si se va a controlar el robot con una funcion de robot_utilities en vez de con el ciclo:
		# opcion 1: cambiar True por False en la condicion del while
		# opcion 2: comentar el bloque de codigo
		while not rospy.is_shutdown() and False:
			x=R.pose_x
			y=R.pose_y
			theta=R.angle
			# Define velocidad lineal [metros/segundo] del robot durante la iteracion
			#R.cmd.linear.x = 0.2
			R.nSteps(0,1500)

			# Velocidad angular [radianes/segundo]; rango aceptable: [-pi/2, pi/2 ], aprox. [-1.57, 1.57]
			R.cmd.angular.z = 0.0 

			# indica a Stage la velocidad que el robot tendra durante un ciclo
			R.cmd_pub.publish(R.cmd)
			
			# Ejemplo de como rescatar datos captados por los sensores
			# En este caso solo se imprimen, pero la idea es utilizarlos en decisiones de control
			print "_______________________"
			#print 'Primera lectura del laser [m]: '+ str(R.distances[0])
			#print 'Posicion robot [m]: x = ' + str(R.pose_x)+'; y = ' +str(R.pose_y)
			print 'Delta theta: ' + str(R.angle-theta)
			print 'Delta X: ' + str(R.pose_x-x)
			print 'Delta Y: ' + str(R.pose_y-y)
			#R.show_distance()
			#print str(R.angle)
			# Mantiene la regularidad temporal (de acuerdo al rate definido en robot_utilities.Robot)
			R.rate.sleep()
	

		##DATOS PARA DESARROLLO DE REPORTE 3
		
		#DATOS ANTERIORES
		xa=-3.0    #Posicion Inicial
		ya=-4.0
		ta=90.0	
		Xg=1.0   #Posicion Final
		Yg=-4.0
		w=0.404
		rel=1.818*10**(-4)  ##r/N
		
		#DATOS SOBRE ULTIMO REPORTE
	
		Katt=1.0
		Krep=0.01
		Rmax=1.1
		
		Pmin=0.1
		prop=3
		contador=0

		RecX=[]
		RecY=[]
	
		##Inicio de Parte Iterativa Para llegar a la posicion
		while True:
			
			#Posicion y Direccion de Robot al inicio del ciclo			
			Trob =  math.radians(R.angle)
			X=R.pose_x
			Y=R.pose_y
			
			RecX.append(X)	
			RecY.append(Y)	
			
			#Calculo de Componentes de Atraccion
			RR=math.sqrt((X-Xg)**2+(Y-Yg)**2)
			if RR > Rmax:
				Xatt = Katt*(Rmax/RR)*(Xg-X)
				Yatt = Katt*(Rmax/RR)*(Yg-Y)
			else:
				Xatt = Katt*(Xg-X)
 				Yatt = Katt*(Yg-Y)
			

			#Calculo de Componente Repulsiva
			n = numpy.asarray(R.distances)
			l = numpy.asarray(R.laser_angles)
			nz=numpy.zeros(len(n))
			lz=numpy.zeros(len(n))
			c = 0
				
			for i in range(len(n)):
				if n[i-1]<5:	#Buscar los datos de objetos detectados
					nz[c]= n[i-1]
					lz[c]= l[i-1]
					c = 1 + c
			nz=nz[0:c]
			lz=lz[0:c]
			dd=numpy.square(nz)
			lc=numpy.cos(lz)
			ls=numpy.sin(lz)	
			F=numpy.sum(lc/dd)
			S=numpy.sum(ls/dd)
			
			Xrep=Krep*F*math.cos(Trob)-Krep*S*math.sin(Trob)
			Yrep=Krep*F*math.sin(Trob)+Krep*S*math.cos(Trob)

			#Calculo de Fuerza P
			Px=Xatt-Xrep
			Py=Yatt-Yrep
			
			P=math.sqrt(Px**2+Py**2)

			if P<Pmin:
				break

			#Movimiento

			#---Mov en el Eje---
			Zeta=math.degrees(math.atan(Py/Px))
			delta=(Zeta-R.angle)
			if delta>180:
				delta=delta-360
			elif delta<-180:
				delta=delta+360

			n1=((math.radians(delta)*w)/(4*math.pi*rel))
			R.nSteps(-n1,n1)
			
			#---Movimiento Recto---
			vel=prop*P
			Dist=vel*0.01
			R.moveTill(Dist,vel , 0)
	
			contador += 1
			print contador,P,vel,Dist,R.pose_x,R.pose_y
			R.show_distance()
			R.reset_perim()
			
        
		#Imprimir Recorrido
		print 'Cantidad de Procesos (Tiempo de Mov)= '+ str(contador)
		
		plt.plot(xa,ya,'o',Xg,Yg,'g^',RecX,RecY)
		plt.title('Trayectoria Robot')
		plt.xlabel('Eje X [m]')
		plt.ylabel('Eje Y [m]')
		plt.xlim([-8,8])
		plt.ylim([-8,8])
		plt.grid()
		plt.show()


if __name__ == "__main__": Controller()
