#!/usr/bin/env python3
#
# CONCURSO IBEROAMERICANO DE ROBOTICA ESPACIAL, PEU-UNAM, 2022
# ETAPA 02 - ROS Y EL ROBOT HSR
# EJERCICIO 01
#
# Instrucciones:
# Complete el programa para mover el robot hacia el frente
# hasta que el laser detecte un obstaculo al frente.
# Los publicadores y suscriptores requeridos ya se encuentran
# declarados e inicializados
#

from telnetlib import TN3270E
from traceback import print_list
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
PI = 3.1415
EQUIPO = "COLOQUE AQUI EL NOMBRE DE SU EQUIPO"


def callback_scan(msg):
    global centralObstacle_detected, obstacle_right, obstacle_left, obstacle_center, distanceFromCentralObstacle
    centerIndexLecture = int((0 - msg.angle_min)/msg.angle_increment)
    #centralObstacle_detected = msg.ranges[centerIndexLecture ] < 1.0
    #No considera la lectura del centro
    centralObstacle_detected = False

    rightCenterAreaLimit= int(((-20*PI/180) - msg.angle_min)/msg.angle_increment)
    leftCenterAreaLimit= int(((20*PI/180)  - msg.angle_min)/msg.angle_increment)
    rightAreaLimit= int(((-40*PI/180)  - msg.angle_min)/msg.angle_increment)
    leftAreaLimit= int(((40*PI/180)  - msg.angle_min)/msg.angle_increment)
    distanceFromCentralObstacle = msg.ranges[centerIndexLecture ]
    if len(msg.ranges)> 0:
       # print(str(leftAreaLimit)+" "+str(leftCenterAreaLimit)+" "+ str(rightCenterAreaLimit)+" "+str(rightAreaLimit)+" "+str(len(msg.ranges))+ " "+str(centerIndexLecture))
        obstacle_left=False
        for indexLecture in range( leftCenterAreaLimit, leftAreaLimit):
            if msg.ranges[indexLecture] <1:
                obstacle_left=True
             
        obstacle_center=False
        for indexLecture in range(rightCenterAreaLimit, leftCenterAreaLimit):
            if msg.ranges[indexLecture]<1:
                obstacle_center =True
        #No considera obstaculos al frente
        obstacle_center=False

        obstacle_right=False
        for indexLecture in range(rightAreaLimit, rightCenterAreaLimit):
            
            if msg.ranges[indexLecture]<1:
               
                obstacle_right = True
                
    return 
def GoBack ():
    BackTwist= Twist()
    BackTwist.linear.x= -0.2
    pub_cmd_vel.publish(BackTwist)
    return

def main():
    print("EJERCICIO 01 - " + EQUIPO)
    rospy.init_node("ejercicio01")
    rospy.Subscriber("/hsrb/base_scan", LaserScan, callback_scan)
    pub_cmd_vel = rospy.Publisher(
        "/hsrb/command_velocity", Twist, queue_size=10)
    loop = rospy.Rate(10)

    global centralObstacle_detected, obstacle_right, obstacle_left, obstacle_center, distanceFromCentralObstacle
    centralObstacle_detected = False
    obstacle_left = False
    obstacle_right = False
    obstacle_center=False
    myTwist = Twist()
    myTwist.linear.x = 0
    myTwist.linear.y = 0
    myTwist.linear.z = 0
    myTwist.angular.x = 0
    myTwist.angular.y = 0
    myTwist.angular.z = 0
    while not rospy.is_shutdown():
        #si se detecta al centro da media vuelta
        #retrocede antes de dar la vuelta
        #Gira al lado contrario
        
        if centralObstacle_detected or (obstacle_right and obstacle_left):

            while distanceFromCentralObstacle <1.2:
                myTwist.angular.z=0
                myTwist.linear.x=-2
                print("linear" + str(myTwist.linear.x ))
                pub_cmd_vel.publish(myTwist)

            currentTime= rospy.get_time()
            transcurredTime=0
            print(currentTime)
            while transcurredTime < 1.4:
                transcurredTime=rospy.get_time()-currentTime
                myTwist.angular.z=2
                myTwist.linear.x=0
                print("angular" + str(myTwist.angular.z ))
                pub_cmd_vel.publish(myTwist)
                rospy.sleep(0.3)

        elif obstacle_center and not obstacle_left:
            while obstacle_center:
                myTwist.linear.x=0
                myTwist.angular.z=2
                print("angular" + str(myTwist.angular.z ))
                pub_cmd_vel.publish(myTwist)
                rospy.sleep(0.3)
          
                
            myTwist.linear.x=0
            myTwist.angular.z=2
            print("angular" + str(myTwist.angular.z ))
            pub_cmd_vel.publish(myTwist)
            rospy.sleep(0.3)

        elif obstacle_center and not obstacle_right:
            while obstacle_center:
                myTwist.linear.x=0
                myTwist.angular.z=-2
                print("angular" + str(myTwist.angular.z ))
                pub_cmd_vel.publish(myTwist)
                rospy.sleep(0.3)
            
            myTwist.linear.x=0
            print("angular" + str(myTwist.angular.z ))
            myTwist.angular.z=-2
            pub_cmd_vel.publish(myTwist)
            rospy.sleep(0.3)
        elif not obstacle_center:
            
            myTwist.linear.x=2
            myTwist.angular.z=0
               
        else:
            print("Este caso no esta considerado: obstacle: {}, frente: {}, derecha: {}, izquierda: {}".format(
                centralObstacle_detected, obstacle_center, obstacle_right, obstacle_left))

        print("Angular" + str(myTwist.angular.z))

        print("linear" + str(myTwist.linear.x ))
        pub_cmd_vel.publish(myTwist)

        loop.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
