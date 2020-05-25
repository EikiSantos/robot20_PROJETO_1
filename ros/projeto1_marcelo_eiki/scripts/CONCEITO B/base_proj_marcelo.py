#! /usr/bin/env python
# -*- coding:utf-8 -*-

from __future__ import print_function, division
import rospy
import numpy as np
import numpy
import tf
import math
import cv2
import time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from numpy import linalg
from tf import transformations
from tf import TransformerROS
import tf2_ros
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from numpy import linalg
from tf import transformations
from tf import TransformerROS
import tf2_ros
import math
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan

import visao_module_marcelo

confirm = False
while confirm != True:
    color = raw_input("Cor (blue, green, pink):")
    station = raw_input("Estacao (dog, cat, bicycle e bird): ")
    ID = input("ID: ")


    goal = [color, ID, station]

    dic = {}
    dic["Color"] = color
    dic["Station"] = station
    dic["ID"] = ID
    print("--------------------------------------------------")
    for k,v in dic.items():
        print(k, " : ", v)
    print("--------------------------------------------------")
    resp = raw_input("Confirmar (y/n): ")
    if resp == "y":
        confirm = True
    else:
        confirm = False





bridge = CvBridge()

cv_image = None
media_verde = []
media_amarelo = []
centro = []
atraso = 1.5E9 # 1 segundo e meio. Em nanossegundos


area = 0.0 # Variavel com a area do maior contorno

# Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados. 
# Descarta imagens que chegam atrasadas demais
check_delay = False 

resultados = [] # Criacao de uma variavel global para guardar os resultados vistos

x = 0
y = 0
z = 0 
id = 0

# INTERSECÇÃO
xii = 0
yii = 0

frame = "camera_link"
# frame = "head_camera"  # DESCOMENTE para usar com webcam USB via roslaunch tag_tracking usbcam
scan_frente = []
tfl = 0

tf_buffer = tf2_ros.Buffer()


def scaneou(dado):
	#print("Faixa valida: ", dado.range_min , " - ", dado.range_max )
	#print("Leituras:")
	#print(np.array(dado.ranges).round(decimals=2))
	k = np.array(dado.ranges).round(decimals=2)
	global scan_frente
	scan_frente = [k[-8],k[-7],k[-6],k[-5],k[-4],k[-3],k[-2],k[-1],k[0],k[1],k[2],k[3],k[4],k[5],k[6],k[7],k[8]]
	#print("Intensities")
	#print(np.array(dado.intensities).round(decimals=2))



def recebe(msg):
	global x # O global impede a recriacao de uma variavel local, para podermos usar o x global ja'  declarado
	global y
	global z
	global id

	for marker in msg.markers:
		id = marker.id
		marcador = "ar_marker_" + str(id)

		print(tf_buffer.can_transform(frame, marcador, rospy.Time(0)))
		header = Header(frame_id=marcador)
		# Procura a transformacao em sistema de coordenadas entre a base do robo e o marcador numero 100
		# Note que para seu projeto 1 voce nao vai precisar de nada que tem abaixo, a 
		# Nao ser que queira levar angulos em conta
		trans = tf_buffer.lookup_transform(frame, marcador, rospy.Time(0))
		
		# Separa as translacoes das rotacoes
		x = trans.transform.translation.x
		y = trans.transform.translation.y
		z = trans.transform.translation.z
		# ATENCAO: tudo o que vem a seguir e'  so para calcular um angulo
		# Para medirmos o angulo entre marcador e robo vamos projetar o eixo Z do marcador (perpendicular) 
		# no eixo X do robo (que e'  a direcao para a frente)
		t = transformations.translation_matrix([x, y, z])
		# Encontra as rotacoes e cria uma matriz de rotacao a partir dos quaternions
		r = transformations.quaternion_matrix([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
		m = numpy.dot(r,t) # Criamos a matriz composta por translacoes e rotacoes
		z_marker = [0,0,1,0] # Sao 4 coordenadas porque e'  um vetor em coordenadas homogeneas
		v2 = numpy.dot(m, z_marker)
		v2_n = v2[0:-1] # Descartamos a ultima posicao
		n2 = v2_n/linalg.norm(v2_n) # Normalizamos o vetor
		x_robo = [1,0,0]
		cosa = numpy.dot(n2, x_robo) # Projecao do vetor normal ao marcador no x do robo
		angulo_marcador_robo = math.degrees(math.acos(cosa))

		# Terminamos
		print("id: {} x {} y {} z {} angulo {} ".format(id, x,y,z, angulo_marcador_robo))



# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
    print("frame")
    global cv_image
    global media_verde
    global media_amarelo
    global centro

    global resultados

    global xii
    global yii

    #global maior_area_verde


    #Variaveis para intersecção
    


    now = rospy.get_rostime()
    imgtime = imagem.header.stamp
    lag = now-imgtime # calcula o lag
    delay = lag.nsecs
    # print("delay ", "{:.3f}".format(delay/1.0E9))
    if delay > atraso and check_delay==True:
        print("Descartando por causa do delay do frame:", delay)
        return 
    try:
        antes = time.clock()
        cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        # Note que os resultados já são guardados automaticamente na variável
        # chamada resultados
        centro, imagem, resultados =  visao_module_marcelo.processa(cv_image)
        #COR VERDE
        media_verde, maior_area_verde =  visao_module_marcelo.identifica_cor(cv_image,color)
        #COR AMARELO
        media_amarelo, maior_area_amarelo =  visao_module_marcelo.identifica_cor_amarelo(cv_image)




        for r in resultados:
            # print(r) - print feito para documentar e entender
            # o resultado            
            pass

        depois = time.clock()
        # Desnecessário - Hough e MobileNet já abrem janelas
        #cv2.imshow("Camera", cv_image)




    except CvBridgeError as e:
        print('ex', e)
    
if __name__=="__main__":
    rospy.init_node("cor")

    topico_imagem = "/camera/rgb/image_raw/compressed"

    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
    recebedor = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, recebe) # Para recebermos notificacoes de que marcadores foram vistos

    recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou) #SCAN

    print("Usando ", topico_imagem)

    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)


    tolerancia = 25
    pegou_verde = False
    # Exemplo de categoria de resultados
    # [('chair', 86.965459585189819, (90, 141), (177, 265))]

    try:
        # Inicializando - por default gira no sentido anti-horário
        vel = Twist(Vector3(0,0,0), Vector3(0,0,math.pi/10.0))
        
        while not rospy.is_shutdown():
            #vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
            # o valor mínimo lido na lista do scan sera utilizado como método de aproximação
            if len(scan_frente) > 0:
                minimo = min(scan_frente)
            else:
                minimo = 5
            # Iniciação da rota. O robo primeiro procurará a cor,
            # se aproximará e assumiremos o controle da garra
            if pegou_verde == False:
                # Primeira condição se refere a não ter cor do creeper na tela mas ter cor amarela
                if len(media_amarelo) != 0 and len(centro) != 0 and media_verde[1] <1:
                    print ("NÃO PEGOU VERDEEE, NA TRILHA")
                    vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
                    tolerancia = 50
                    if (media_amarelo[0] < centro[0] - tolerancia):
                        # Vira à esquerda
                        vel = Twist(Vector3(0,0,0), Vector3(0,0,+math.pi/15.0))
                        #print("ESQUERDA")
                    elif (media_amarelo[0] > centro[0] + tolerancia):
                        # Vira à direita
                        vel = Twist(Vector3(0,0,0), Vector3(0,0,-math.pi/15.0))                    
                        #print("DIREITA")
                    elif (centro[0]- tolerancia < media_amarelo[0] < centro[0] + tolerancia): # Gosto de usar a < b < c do Python. Não seria necessário neste caso
                        # Segue em frente
                        vel = Twist(Vector3(0.2,0,0), Vector3(0,0,0))
                        #print("FRENTE")
                # Tem o creeper que desejamos na tela, o robo irá em sua direção
                elif len(media_verde) != 0 and len(centro) != 0:
                    print ("PROCURANDO VERDEE")
                    tolerancia = 15
                    vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
                    print("Média dos verdes: {0}, {1}".format(media_verde[0], media_verde[1]))
                    print ("Scann")
                    print ("MENOR TERMOOOOOOO: ",minimo)
                    # saber aproximação do robo
                    if minimo>0.20:
                        x_objeto = media_verde[0]
                        if (x_objeto < centro[0] - tolerancia):
                            # Vira à esquerda
                            vel = Twist(Vector3(0,0,0), Vector3(0,0,math.pi/15.0))
                            #print("ESQUERDA")
                        elif (x_objeto > centro[0] + tolerancia):
                            # Vira à direita
                            vel = Twist(Vector3(0,0,0), Vector3(0,0,-math.pi/15.0))                    
                            #print("DIREITA")
                        elif (centro[0]- tolerancia < x_objeto < centro[0] + tolerancia): # Gosto de usar a < b < c do Python. Não seria necessário neste caso
                            # Segue em frente
                            vel = Twist(Vector3(0.3,0,0), Vector3(0,0,0))
                            #print("FRENTE")
                    # quando estiver próximo, robo ficará parado e poderemos assumir a garra
                    else:#Valor mínimo do scan (Saber se o robo bateu no verde)
                        vel = Twist(Vector3(0,0,0),Vector3(0,0,0))
                        velocidade_saida.publish(vel)
                        rospy.sleep(0.1)
                        raw_input()
                        pegou_verde = True
                        rospy.sleep(0.1)
                        # Após pegar o robo, creeper dará meia volta e 
                        # voltará a para a pista.
                        # o código abaixo é necessário para o robo ficar próximo o suficiente da pista 
                        # e decidir a direção que irá seguir
                        vel = Twist(Vector3(-0.4,0,0),Vector3(0,0,-2))
                        velocidade_saida.publish(vel)
                        rospy.sleep(1.5)
                        vel = Twist(Vector3(0.5,0,0),Vector3(0,0,0))
                        velocidade_saida.publish(vel)
                        rospy.sleep(1.5)
                        

            # Esse else se refere a ja ter pego o creeper,
            # robo irá procurar a base agora e se não encontrar, continuará no projeto
            else:
                for r in resultados:
                    # Encontrou a base, irá em sua direção
                    if station in r[0]:
                        print ("ACHOU A BASEEE")
                        x_objeto = (r[2][0] + r[3][0])/2
                        tolerancia = 100
                        # método de aproximação, se ficar mt próximo a camera não detecta a figura
                        if minimo > 0.7:
                            if x_objeto < (centro[0] - tolerancia):
                                # Vira à esquerda
                                vel = Twist(Vector3(0,0,0), Vector3(0,0,math.pi/8.0))
                                print("ESQUERDA")
                            elif x_objeto > (centro[0] + tolerancia):
                                # Vira à direita
                                vel = Twist(Vector3(0,0,0), Vector3(0,0,-math.pi/8.0))                    
                                print("DIREITA")
                            elif (centro[0]- tolerancia) < x_objeto < (centro[0] + tolerancia): # Gosto de usar a < b < c do Python. Não seria necessário neste caso
                                # Segue em frente
                                vel = Twist(Vector3(0.4,0,0), Vector3(0,0,0))
                                print("FRENTE")
                        else:
                            # nesse momento, devemos abrir a garra, pode ser feito pelo código mesmo no futuro
                            vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
                            velocidade_saida.publish(vel)
                            rospy.sleep(0.1)
                            raw_input()
                    elif len(media_amarelo) != 0 and len(centro) != 0:
                        print ("PEGOU CREEPER. PROCURANDO A BASE")
                        vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
                        tolerancia = 40
                        if (media_amarelo[0] < centro[0] - tolerancia):
                            # Vira à esquerda
                            vel = Twist(Vector3(0,0,0), Vector3(0,0,+math.pi/10))
                            print("ESQUERDA")
                        elif (media_amarelo[0] > centro[0] + tolerancia):
                            # Vira à direita
                            vel = Twist(Vector3(0,0,0), Vector3(0,0,-math.pi/10))                    
                            print("DIREITA")
                        elif (centro[0]- tolerancia < media_amarelo[0] < centro[0] + tolerancia): # Gosto de usar a < b < c do Python. Não seria necessário neste caso
                            # Segue em frente
                            vel = Twist(Vector3(0.2,0,0), Vector3(0,0,0))
                            print("FRENTE")
                    else:
                        # SE NÃO TIVER AMARELO ou base na tela, RODA ATE ACHAR UM
                        print("PROCURANDO AMARELOO")
                        vel = Twist(Vector3(-0.7,0,0), Vector3(0,0,-1))
                        #TEMPO DO ROBO RODAR QUANDO TIVER PEGO O CREEPER
                        rospy.sleep(0.5)
                        vel = Twist(Vector3(0.5,0,0), Vector3(0,0,0))
                        rospy.sleep(0,5)

            velocidade_saida.publish(vel)
            rospy.sleep(0.1)
    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")


