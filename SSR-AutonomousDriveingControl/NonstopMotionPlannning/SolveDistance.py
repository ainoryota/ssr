from re import M
import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from Utility import ConvertXYZCoodinate,cross,CalcCylinderNP
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D


def PlotCylinder(a,b,c,O1,P1,P2,r,R,a1=1,b1=1,c1=1,d1=1,a2=1,b2=1,c2=1,d2=1,other_points=[],names=[],other_lines=[]):
	# Figure��ǉ�
	fig = plt.figure(figsize = (8, 8))

	# 3DAxes��ǉ�
	ax = fig.add_subplot(111, projection='3d')

	# Axes�̃^�C�g����ݒ�
	ax.set_title("Helix", size = 20)

	# �����x����ݒ�
	ax.set_xlabel("x", size = 14)
	ax.set_ylabel("y", size = 14)
	ax.set_zlabel("z", size = 14)

	ax.cla()
	ax.set_xlim(0,1000)
	ax.set_ylim(0,1000)
	ax.set_zlim(0,1000)

	xlim = ax.get_xlim()
	ylim = ax.get_ylim()

	#�~���̒��S��
	x = np.array([P1[0],P2[0]])
	y = np.array([P1[1],P2[1]])
	z = np.array([P1[2],P2[2]])
	ax.plot(x, y, z, c = "blue")

	if(len(other_lines) > 0):
		other_lines = np.array(other_lines)
		ax.plot(other_lines[:,0], other_lines[:,1], other_lines[:,2], c = "green")

	for i in range(len(other_points)):
		data = other_points[i]
		ax.scatter(np.array(data[0]), np.array(data[1]),np.array(data[2]), c = "red",s=50)

		if(i >= len(names)):
			continue
		name = names[i]
		ax.text(data[0][0],data[1][0],data[2][0], name,size=15,c="red")

	
	#円柱側
	points = CalcCylinderNP(P1,P2,r)
	ax.plot(np.array(points)[:,0,0],np.array(points)[:,1,0],np.array(points)[:,2,0], c="green")

	#円側
	points = CalcCylinderNP(O1,O1 + np.array([[a,b,c]]).T / np.linalg.norm(np.array([[a,b,c]])),R)
	ax.plot(np.array(points)[:,0,0],np.array(points)[:,1,0],np.array(points)[:,2,0], c="green")


	#ax.plot(np.array([O1[0],10]), np.array([O1[1],10]), np.array([O1[2],10]), c =
	#"red")

	X,Y = np.meshgrid(np.arange(xlim[0], xlim[1],50),
					np.arange(ylim[0], ylim[1],50))
	Z = np.zeros(X.shape)


	#平面上にO1がある
	#d=-(ax+by+cz)
	d = -(a * O1[0][0] + b * O1[1][0] + c * O1[2][0])
	for hoge1 in range(X.shape[0]):
		for hoge2 in range(X.shape[1]):
		    Z[hoge1,hoge2] = -(a / c * X[hoge1,hoge2] + b / c * Y[hoge1,hoge2] + d / c)
	ax.plot_wireframe(X,Y,Z, color='k')


	a = a1
	b = b1
	c = c1
	d = d1
	X,Y = np.meshgrid(np.arange(xlim[0], xlim[1],50),
					np.arange(ylim[0], ylim[1],50))
	Z = np.zeros(X.shape)


	#平面上にO1がある
	#d=-(ax+by+cz)
	#d = -(a * O1[0][0] + b * O1[1][0] + c * O1[2][0])
	for hoge1 in range(X.shape[0]):
		for hoge2 in range(X.shape[1]):
		    Z[hoge1,hoge2] = -(a / c * X[hoge1,hoge2] + b / c * Y[hoge1,hoge2] + d / c)

	ax.plot_wireframe(X,Y,Z, color='blue')

	a = a2
	b = b2
	c = c2
	d = d2
	X,Y = np.meshgrid(np.arange(xlim[0], xlim[1],50),
					np.arange(ylim[0], ylim[1],50))
	Z = np.zeros(X.shape)


	#平面上にO1がある
	#d=-(ax+by+cz)
	#d = -(a * O1[0][0] + b * O1[1][0] + c * O1[2][0])
	for hoge1 in range(X.shape[0]):
		for hoge2 in range(X.shape[1]):
		    Z[hoge1,hoge2] = -(a / c * X[hoge1,hoge2] + b / c * Y[hoge1,hoge2] + d / c)

	ax.plot_wireframe(X,Y,Z, color='red')


	plt.show()
	pass

def Aligined(a,b):
	#np.concatenate([a, b], axis=1)
	return np.array([a.tolist(),b.tolist()])[:,:,0].T



def D(k,PQ):
	n = PQ / np.linalg.norm(PQ)
	return np.linalg.norm(cross(k,n))

def L(n,P,n2,P2):
	X0 = Aligined(n,n2) @ np.linalg.inv(Aligined(n,n2).T @ Aligined(n,n2)) @ np.array([[np.vdot(n,P),np.vdot(n2,P2)]]).T
	return X0,cross(n,n2) / np.linalg.norm(cross(n,n2))


def SolveDistance(a,b,c,O1,P1,P2,r,R):
	distance = 1000000

	PlotCylinder(a,b,c,O1,P1,P2,r,R)

	line = np.array([[a,b,c]]).T
	k = np.vdot(line , (P1 - O1)) / np.vdot(line , (P1 - P2))

	if(np.vdot(line,(P2 - P1)) == 0):#円柱と平面が平行
		Q = O1 + R * (P2 - P1) / np.linalg.norm(P2 - P1)
		print("A:",D(P1 - P2,Q - P1))
		return D(P1 - P2,Q - P1)

	E = P1 + k * (P2 - P1)
	v1 = (E - O1) / np.linalg.norm(E - O1)
	v2 = 1 / np.linalg.norm(line) * (cross(line,v1))
	V1 = R * v1 + O1

	foo = 1 / np.linalg.norm(P2 - P1) ** 2 * np.vdot((V1 - P1),(P2 - P1))
	H = P1 + foo * (P2 - P1)

	if(0 <= foo and foo <= 1):
		print("B:",np.linalg.norm(H - P1) - r)
		return np.linalg.norm(H - P1) - r
	else:
		if(foo < 0):P = P1
		else:P = P2
		for theta in [a * math.pi / 100000 for a in  range(0,100000)]:
			Q = R * (math.cos(theta) * v1 + math.sin(theta) * v2) + O1
			Q2 = -R * (math.cos(theta) * v1 + math.sin(theta) * v2) + O1

			X0,m = L(P2 - P1,P,-math.sin(theta) * v1 + math.cos(theta) * v2,O1)


			Det = np.vdot(m,(X0 - P)) ** 2 - (np.linalg.norm(X0 - P) ** 2 - r ** 2)
			#print(theta,m.T)
			#print("P1P2*line/norm(P1P2)/norm(line)",np.vdot(P2 - P1,line) /
			#(np.linalg.norm(P2 - P1) * np.linalg.norm(line)))
			hoge = -math.sin(theta) * v1 + math.cos(theta) * v2

			a1 = hoge[0][0]
			b1 = hoge[1][0]
			c1 = hoge[2][0]
			d1 = -np.vdot(hoge,O1)

			hogehoge = P2 - P1
			a2 = hogehoge[0][0]
			b2 = hogehoge[1][0]
			c2 = hogehoge[2][0]
			d2 = -np.vdot(hogehoge,P)

			#print("check:",np.vdot())

			#print("hoge:",np.vdot(hoge,P2 - P1))
			#print(theta,m.T,(P2 - P1).T,(-math.sin(theta) * v1 + math.cos(theta) * v2).T)
			#PlotCylinder(a,b,c,O1,P1,P2,r,R,a1,b1,c1,d1,a2,b2,c2,d2,[H,E,P1,P2,V1,Q,X0],["H","E","P1","P2","V1","Q","X0"],[X0 - 1000 * m,X0 + 1000 * m])

			if(theta > 1.5):
				pass


				#PlotCylinder(a,b,c,O1,P1,P2,r,R,a1,b1,c1,d1,a2,b2,c2,d2,[H,E,P1,P2,V1,Q,X0],["H","E","P1","P2","V1","Q","X0"],[X0 - 1000 * m,X0 + 1000 * m])
			if(Det < 0):
				pass
				#break
			elif(Det == 0):
				X1 = X0 - np.vdot(m , (X0 - P)) * m
				X2 = X1
				distance = min(distance,np.linalg.norm(X1 - Q),np.linalg.norm(X2 - Q))
				distance = min(distance,np.linalg.norm(X1 - Q2),np.linalg.norm(X2 - Q2))
			else:
				#PlotCylinder(a,b,c,O1,P1,P2,r,R,a1,b1,c1,d1,a2,b2,c2,d2,[H,E,P1,P2,V1,Q,X0],["H","E","P1","P2","V1","Q","X0"],[X0 - 1000 * m,X0 + 1000 * m])
				X1 = X0 + (np.vdot(-m , (X0 - P)) + math.sqrt(Det)) * m
				X2 = X0 + (np.vdot(-m , (X0 - P)) - math.sqrt(Det)) * m
				distance = min(distance,np.linalg.norm(X1 - Q),np.linalg.norm(X2 - Q))
				distance = min(distance,np.linalg.norm(X1 - Q2),np.linalg.norm(X2 - Q2))
			print("Det:",Det,theta,distance)
	print(distance)

	

#print(SolveDistance(1,1,-1,np.array([[250,700,200]]).T,np.array([[200,200,300]]).T,np.array([[150,600,400]]).T,5,50))
print(SolveDistance(1,1,-1,np.array([[500,900,200]]).T,np.array([[400,200,300]]).T,np.array([[400,501,600]]).T,5,50))
#print(SolveDistance(1,1,-1,np.array([[500,200,200]]).T,np.array([[400,200,300]]).T,np.array([[400,250,600]]).T,5,50))